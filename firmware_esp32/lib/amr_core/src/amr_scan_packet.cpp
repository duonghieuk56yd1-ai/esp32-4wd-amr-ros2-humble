#include "amr_scan_packet.h"

#include <cmath>
#include <cstring>

namespace amr {

namespace {

constexpr float kTwoPi = 6.283185307179586f;

uint8_t* putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  return p + 2;
}

uint8_t* putU32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
  return p + 4;
}

uint8_t* putU64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
  return p + 8;
}

uint8_t* putF32(uint8_t* p, float f) {
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(f), "float phải 32 bit");
  std::memcpy(&bits, &f, sizeof(bits));
  return putU32(p, bits);
}

uint16_t getU16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t getU32(const uint8_t* p) {
  uint32_t v = 0;
  for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(p[i]) << (8 * i);
  return v;
}

uint64_t getU64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(p[i]) << (8 * i);
  return v;
}

float getF32(const uint8_t* p) {
  const uint32_t bits = getU32(p);
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

}  // namespace

ScanAssembler::ScanAssembler(const ScanAssemblerConfig& config) : config_(config) {
  if (config_.bins == 0) config_.bins = 1;
  if (config_.bins > kMaxBins) config_.bins = kMaxBins;
  bin_width_deg_ = 360.0f / static_cast<float>(config_.bins);
  range_min_mm_ = config_.range_min_m * 1000.0f;
  range_max_mm_ = config_.range_max_m * 1000.0f;
  if (range_min_mm_ < 1.0f) range_min_mm_ = 1.0f;          // 0 mm = "không phản hồi"
  if (range_max_mm_ > 65535.0f) range_max_mm_ = 65535.0f;  // giới hạn uint16
  clear();
}

void ScanAssembler::clear() {
  std::memset(ranges_mm_, 0, sizeof(ranges_mm_));
  std::memset(intensity_, 0, sizeof(intensity_));
  valid_ = 0;
  points_ = 0;
}

float ScanAssembler::angleIncrementRad() const {
  return kTwoPi / static_cast<float>(config_.bins);
}

int ScanAssembler::binIndex(float raw_angle_deg) const {
  if (!std::isfinite(raw_angle_deg)) return -1;
  float a = config_.clockwise ? -raw_angle_deg : raw_angle_deg;
  a += config_.yaw_offset_deg;
  a = std::fmod(a, 360.0f);
  if (a < 0.0f) a += 360.0f;
  // Làm tròn tới tia gần nhất: tia i phủ [i - 0.5, i + 0.5) * bin_width
  int idx = static_cast<int>(std::lround(a / bin_width_deg_));
  if (idx >= config_.bins) idx -= config_.bins;
  if (idx < 0 || idx >= config_.bins) return -1;
  return idx;
}

bool ScanAssembler::addPoint(float angle_deg, float distance_mm, float quality) {
  ++points_;
  if (!std::isfinite(distance_mm) || distance_mm < range_min_mm_ ||
      distance_mm > range_max_mm_) {
    return false;  // gồm cả distance <= 0: điểm không hợp lệ theo thư viện LDS
  }
  const int idx = binIndex(angle_deg);
  if (idx < 0) return false;

  long mm = std::lround(distance_mm);
  if (mm < 1) mm = 1;
  if (mm > 65535) mm = 65535;

  uint16_t& slot = ranges_mm_[idx];
  if (slot == 0) {
    ++valid_;
  } else if (static_cast<uint16_t>(mm) >= slot) {
    return false;  // đã có điểm gần hơn trong tia này
  }
  slot = static_cast<uint16_t>(mm);

  long q = std::isfinite(quality) ? std::lround(quality) : 0;
  if (q < 0) q = 0;
  if (q > 255) q = 255;
  intensity_[idx] = static_cast<uint8_t>(q);
  return true;
}

size_t encodeScanPacket(const ScanAssembler& scan, const ScanMeta& meta,
                        bool with_intensity, uint8_t* buf, size_t buf_size) {
  const uint16_t n = scan.bins();
  const size_t need = scanPacketSize(n, with_intensity);
  if (buf == nullptr || need > buf_size || need > kMaxUdpPayload) return 0;

  uint16_t flags = 0;
  if (with_intensity) flags |= kScanFlagIntensity;
  if (meta.ros_time) flags |= kScanFlagRosTime;

  uint8_t* p = buf;
  std::memcpy(p, kScanMagic, sizeof(kScanMagic));
  p += sizeof(kScanMagic);
  *p++ = kScanPacketVersion;
  *p++ = static_cast<uint8_t>(meta.model);
  p = putU16(p, flags);
  p = putU32(p, meta.seq);
  p = putU64(p, meta.stamp_ns);
  p = putF32(p, meta.scan_time_s);
  p = putF32(p, 0.0f);  // angle_min
  p = putF32(p, scan.angleIncrementRad());
  p = putF32(p, scan.config().range_min_m);
  p = putF32(p, scan.config().range_max_m);
  p = putU16(p, n);
  p = putU16(p, scan.validCount());
  for (uint16_t i = 0; i < n; ++i) p = putU16(p, scan.rangeMm(i));
  if (with_intensity) {
    for (uint16_t i = 0; i < n; ++i) *p++ = scan.intensity(i);
  }
  return static_cast<size_t>(p - buf);
}

uint16_t ScanPacketView::rangeMm(uint16_t i) const {
  return (ranges_le != nullptr && i < num_ranges) ? getU16(ranges_le + 2u * i) : 0;
}

bool decodeScanPacket(const uint8_t* buf, size_t len, ScanPacketView& out) {
  if (buf == nullptr || len < kScanHeaderSize) return false;
  if (std::memcmp(buf, kScanMagic, sizeof(kScanMagic)) != 0) return false;

  ScanPacketView v;
  v.version = buf[4];
  if (v.version != kScanPacketVersion) return false;
  v.model = static_cast<LidarModel>(buf[5]);
  v.flags = getU16(buf + 6);
  v.seq = getU32(buf + 8);
  v.stamp_ns = getU64(buf + 12);
  v.scan_time_s = getF32(buf + 20);
  v.angle_min_rad = getF32(buf + 24);
  v.angle_increment_rad = getF32(buf + 28);
  v.range_min_m = getF32(buf + 32);
  v.range_max_m = getF32(buf + 36);
  v.num_ranges = getU16(buf + 40);
  v.num_valid = getU16(buf + 42);

  const bool with_intensity = (v.flags & kScanFlagIntensity) != 0;
  if (len != scanPacketSize(v.num_ranges, with_intensity)) return false;
  // Giống amr_lidar_bridge/packet.py: phải có ít nhất 1 tia và thông tin góc hợp lệ
  if (v.num_ranges == 0 || !std::isfinite(v.angle_min_rad) ||
      !std::isfinite(v.angle_increment_rad) || !(v.angle_increment_rad > 0.0f)) {
    return false;
  }

  v.ranges_le = buf + kScanHeaderSize;
  v.intensities = with_intensity ? v.ranges_le + 2u * v.num_ranges : nullptr;
  out = v;
  return true;
}

}  // namespace amr
