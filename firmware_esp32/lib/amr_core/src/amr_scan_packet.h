// =============================================================================
//  amr_scan_packet.h — Gom 1 vòng quét LiDAR thành N tia đều nhau và đóng gói
//  nhị phân gửi qua UDP (logic thuần, test được trên PC).
//
//  ĐỊNH DẠNG GÓI (little-endian, không padding, 1 gói UDP = 1 vòng quét):
//
//   offset kiểu      trường               ghi chú
//   0      char[4]   magic                "AMRL"
//   4      uint8     version              = 1
//   5      uint8     lidar_model          1 = YDLIDAR X2, 2 = Camsense X1
//   6      uint16    flags                bit0: có intensities, bit1: stamp là ROS time
//   8      uint32    seq                  số thứ tự vòng quét
//   12     uint64    stamp_ns             thời điểm bắt đầu vòng quét (ns)
//   20     float32   scan_time_s          thời gian 1 vòng (s)
//   24     float32   angle_min_rad        = 0
//   28     float32   angle_increment_rad  = 2*pi / num_ranges
//   32     float32   range_min_m
//   36     float32   range_max_m
//   40     uint16    num_ranges           N
//   42     uint16    num_valid            số tia có phản hồi
//   44     uint16[N] ranges_mm            0 = không có phản hồi (-> +inf trong LaserScan)
//   44+2N  uint8[N]  intensities          chỉ có khi flags bit0 = 1
//
//  Tia i nằm ở góc angle_min + i*angle_increment trong frame LiDAR, quy ước ROS
//  (x phía trước, góc dương ngược chiều kim đồng hồ) -> chép thẳng sang
//  sensor_msgs/LaserScan. Python: struct.unpack_from('<4sBBHIQfffffHH', buf).
// =============================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace amr {

enum class LidarModel : uint8_t {
  kUnknown = 0,
  kYdlidarX2 = 1,
  kCamsenseX1 = 2,
};

constexpr char kScanMagic[4] = {'A', 'M', 'R', 'L'};
constexpr uint8_t kScanPacketVersion = 1;
constexpr uint16_t kScanFlagIntensity = 1u << 0;
constexpr uint16_t kScanFlagRosTime = 1u << 1;
constexpr size_t kScanHeaderSize = 44;
// 1500 (MTU Ethernet/Wi-Fi) - 20 (IPv4) - 8 (UDP): lớn hơn sẽ bị phân mảnh IP.
constexpr size_t kMaxUdpPayload = 1472;

constexpr size_t scanPacketSize(uint16_t bins, bool with_intensity) {
  return kScanHeaderSize + static_cast<size_t>(bins) * 2u +
         (with_intensity ? static_cast<size_t>(bins) : 0u);
}

struct ScanAssemblerConfig {
  uint16_t bins = 360;         // số tia / vòng
  float range_min_m = 0.10f;   // điểm gần hơn bị bỏ
  float range_max_m = 8.0f;    // điểm xa hơn bị bỏ
  float yaw_offset_deg = 0.0f; // bù góc lắp (độ, CCW dương)
  bool clockwise = true;       // góc gốc của LiDAR tăng theo chiều kim đồng hồ
};

class ScanAssembler {
 public:
  static constexpr uint16_t kMaxBins = 720;

  explicit ScanAssembler(const ScanAssemblerConfig& config);

  // Bắt đầu vòng quét mới (xoá toàn bộ tia).
  void clear();

  // Thêm 1 điểm theo quy ước gốc của LiDAR (góc độ, khoảng cách mm, chất lượng).
  // Mỗi tia giữ điểm GẦN NHẤT rơi vào nó. Trả về true nếu điểm được ghi nhận.
  bool addPoint(float angle_deg, float distance_mm, float quality);

  // Góc gốc (độ) -> chỉ số tia [0, bins) theo quy ước ROS; -1 nếu không hợp lệ.
  int binIndex(float raw_angle_deg) const;

  uint16_t bins() const { return config_.bins; }
  uint16_t validCount() const { return valid_; }
  uint32_t pointCount() const { return points_; }
  uint16_t rangeMm(uint16_t i) const { return i < config_.bins ? ranges_mm_[i] : 0; }
  uint8_t intensity(uint16_t i) const { return i < config_.bins ? intensity_[i] : 0; }
  float angleIncrementRad() const;
  const ScanAssemblerConfig& config() const { return config_; }

 private:
  ScanAssemblerConfig config_;
  float bin_width_deg_;
  float range_min_mm_;
  float range_max_mm_;
  uint16_t valid_ = 0;
  uint32_t points_ = 0;
  uint16_t ranges_mm_[kMaxBins];
  uint8_t intensity_[kMaxBins];
};

struct ScanMeta {
  LidarModel model = LidarModel::kUnknown;
  uint32_t seq = 0;
  uint64_t stamp_ns = 0;
  bool ros_time = false;  // stamp_ns là ROS time (đã đồng bộ) hay uptime của ESP32
  float scan_time_s = 0.0f;
};

// Ghi gói vào buf. Trả về số byte đã ghi, 0 nếu buf quá nhỏ hoặc gói vượt
// kMaxUdpPayload.
size_t encodeScanPacket(const ScanAssembler& scan, const ScanMeta& meta,
                        bool with_intensity, uint8_t* buf, size_t buf_size);

// Giải mã (dùng cho test / tham khảo phía Laptop). Con trỏ trỏ vào buf gốc.
struct ScanPacketView {
  uint8_t version = 0;
  LidarModel model = LidarModel::kUnknown;
  uint16_t flags = 0;
  uint32_t seq = 0;
  uint64_t stamp_ns = 0;
  float scan_time_s = 0.0f;
  float angle_min_rad = 0.0f;
  float angle_increment_rad = 0.0f;
  float range_min_m = 0.0f;
  float range_max_m = 0.0f;
  uint16_t num_ranges = 0;
  uint16_t num_valid = 0;
  const uint8_t* ranges_le = nullptr;    // num_ranges x uint16 little-endian
  const uint8_t* intensities = nullptr;  // nullptr nếu không có

  uint16_t rangeMm(uint16_t i) const;
};

bool decodeScanPacket(const uint8_t* buf, size_t len, ScanPacketView& out);

}  // namespace amr
