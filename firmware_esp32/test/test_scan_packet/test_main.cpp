// Unit test: gom tia LiDAR + đóng gói / giải mã gói UDP "AMRL" v1.
// Chạy trên PC: pio test -e native -f test_scan_packet
//
// Gói "vàng" kPythonPacket do amr_lidar_bridge/packet.py (phía Laptop) tạo ra:
// đảm bảo firmware và Laptop hiểu cùng một định dạng theo CẢ HAI chiều
// (chiều ngược lại: ros2_ws/src/amr_lidar_bridge/test/fixtures/scan_ydlidar_x2_v1.bin).
#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "amr_scan_packet.h"
#include "robot_config.h"

void setUp() {}
void tearDown() {}

namespace {

// encode([0, 1000, 65535, 1, 0, 2500, 300, 0], seq=0xDEADBEEF,
//        stamp_ns=1_700_000_000_123_456_789, ros_time=True, model=2, scan_time=0.19,
//        range_min=0.08, range_max=8.0, intensities=[0, 10, 255, 1, 0, 128, 7, 0])
const uint8_t kPythonPacket[] = {
    0x41, 0x4D, 0x52, 0x4C, 0x01, 0x02, 0x03, 0x00, 0xEF, 0xBE, 0xAD, 0xDE,
    0x15, 0xCD, 0x85, 0x3D, 0xFE, 0x9C, 0x97, 0x17, 0x5C, 0x8F, 0x42, 0x3E,
    0x00, 0x00, 0x00, 0x00, 0xDB, 0x0F, 0x49, 0x3F, 0x0A, 0xD7, 0xA3, 0x3D,
    0x00, 0x00, 0x00, 0x41, 0x08, 0x00, 0x05, 0x00, 0x00, 0x00, 0xE8, 0x03,
    0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0xC4, 0x09, 0x2C, 0x01, 0x00, 0x00,
    0x00, 0x0A, 0xFF, 0x01, 0x00, 0x80, 0x07, 0x00,
};

amr::ScanAssemblerConfig defaultConfig() {
  amr::ScanAssemblerConfig cfg;
  cfg.bins = 360;
  cfg.range_min_m = 0.1f;
  cfg.range_max_m = 8.0f;
  cfg.clockwise = true;
  return cfg;
}

uint32_t readU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

float readF32(const uint8_t* p) {
  const uint32_t bits = readU32(p);
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

void writeF32(uint8_t* p, float f) {
  uint32_t bits;
  std::memcpy(&bits, &f, sizeof(bits));
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(bits >> (8 * i));
}

// Gói hợp lệ 360 tia (có cường độ) dùng chung cho các test giải mã.
size_t makePacket(uint8_t* buf, size_t size, bool with_intensity = true) {
  amr::ScanAssembler scan(defaultConfig());
  for (int deg = 0; deg < 360; deg += 2) {
    scan.addPoint(static_cast<float>(deg), 1000.0f + static_cast<float>(deg), 7.0f);
  }
  amr::ScanMeta meta;
  meta.model = amr::LidarModel::kYdlidarX2;
  meta.seq = 42;
  meta.stamp_ns = 1700000000123456789ULL;
  meta.ros_time = true;
  meta.scan_time_s = 0.142f;
  return amr::encodeScanPacket(scan, meta, with_intensity, buf, size);
}

}  // namespace

// ---------------------------------------------------------------------------
// Gom điểm thành tia
// ---------------------------------------------------------------------------
void test_scan_bin_mapping_clockwise_to_ros() {
  amr::ScanAssemblerConfig cfg = defaultConfig();
  amr::ScanAssembler scan(cfg);
  TEST_ASSERT_EQUAL_INT(0, scan.binIndex(0.0f));
  TEST_ASSERT_EQUAL_INT(270, scan.binIndex(90.0f));  // bên phải robot = -90° (ROS)
  TEST_ASSERT_EQUAL_INT(90, scan.binIndex(270.0f));  // bên trái = +90°
  TEST_ASSERT_EQUAL_INT(180, scan.binIndex(180.0f));
  TEST_ASSERT_EQUAL_INT(0, scan.binIndex(0.4f));     // làm tròn về tia gần nhất
  TEST_ASSERT_EQUAL_INT(0, scan.binIndex(359.6f));   // quấn qua 0°
  TEST_ASSERT_EQUAL_INT(1, scan.binIndex(359.0f));
  TEST_ASSERT_EQUAL_INT(1, scan.binIndex(-1.0f));    // góc âm / > 360 vẫn hợp lệ
  TEST_ASSERT_EQUAL_INT(270, scan.binIndex(450.0f));
  TEST_ASSERT_EQUAL_INT(-1, scan.binIndex(NAN));
  TEST_ASSERT_EQUAL_INT(-1, scan.binIndex(INFINITY));

  cfg.clockwise = false;
  cfg.yaw_offset_deg = 10.0f;
  amr::ScanAssembler ccw(cfg);
  TEST_ASSERT_EQUAL_INT(10, ccw.binIndex(0.0f));
  TEST_ASSERT_EQUAL_INT(0, ccw.binIndex(350.0f));
  cfg.yaw_offset_deg = -90.0f;
  amr::ScanAssembler rotated(cfg);
  TEST_ASSERT_EQUAL_INT(270, rotated.binIndex(0.0f));

  cfg = defaultConfig();
  cfg.bins = 720;  // 0.5°/tia
  amr::ScanAssembler fine(cfg);
  TEST_ASSERT_EQUAL_INT(718, fine.binIndex(1.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, static_cast<float>(M_PI) / 360.0f, fine.angleIncrementRad());
}

void test_scan_keeps_closest_point_and_filters_range() {
  amr::ScanAssembler scan(defaultConfig());
  TEST_ASSERT_TRUE(scan.addPoint(0.0f, 1500.0f, 10.0f));
  TEST_ASSERT_TRUE(scan.addPoint(0.2f, 1200.0f, 99.0f));    // cùng tia, gần hơn
  TEST_ASSERT_FALSE(scan.addPoint(-0.2f, 1300.0f, 5.0f));   // cùng tia, xa hơn
  TEST_ASSERT_FALSE(scan.addPoint(10.0f, 0.0f, 10.0f));     // 0 = không hợp lệ
  TEST_ASSERT_FALSE(scan.addPoint(10.0f, -5.0f, 10.0f));
  TEST_ASSERT_FALSE(scan.addPoint(10.0f, 50.0f, 10.0f));    // < range_min
  TEST_ASSERT_FALSE(scan.addPoint(10.0f, 9000.0f, 10.0f));  // > range_max
  TEST_ASSERT_FALSE(scan.addPoint(10.0f, NAN, 10.0f));
  TEST_ASSERT_TRUE(scan.addPoint(180.0f, 800.4f, 300.0f));  // quality kẹp về 255
  TEST_ASSERT_TRUE(scan.addPoint(90.0f, 2000.0f, -3.0f));   // quality âm -> 0
  TEST_ASSERT_EQUAL_UINT16(1200, scan.rangeMm(0));
  TEST_ASSERT_EQUAL_UINT8(99, scan.intensity(0));
  TEST_ASSERT_EQUAL_UINT16(800, scan.rangeMm(180));
  TEST_ASSERT_EQUAL_UINT8(255, scan.intensity(180));
  TEST_ASSERT_EQUAL_UINT8(0, scan.intensity(270));
  TEST_ASSERT_EQUAL_UINT16(3, scan.validCount());
  TEST_ASSERT_EQUAL_UINT32(10, scan.pointCount());
  TEST_ASSERT_EQUAL_UINT16(0, scan.rangeMm(400));  // ngoài mảng -> 0
  scan.clear();
  TEST_ASSERT_EQUAL_UINT16(0, scan.validCount());
  TEST_ASSERT_EQUAL_UINT32(0, scan.pointCount());
  TEST_ASSERT_EQUAL_UINT16(0, scan.rangeMm(0));
}

// Mô phỏng 1 vòng YDLIDAR X2 (~0.5°/điểm, chiều kim đồng hồ): mọi tia đều có dữ liệu
// và mỗi tia giữ điểm gần nhất trong các điểm rơi vào nó.
void test_scan_dense_revolution_fills_every_bin() {
  amr::ScanAssembler scan(defaultConfig());
  for (int k = 0; k < 720; ++k) {
    const float raw = 0.5f * static_cast<float>(k);
    scan.addPoint(raw, 1000.0f + raw, 50.0f);  // khoảng cách tăng theo góc gốc
  }
  TEST_ASSERT_EQUAL_UINT16(360, scan.validCount());
  TEST_ASSERT_EQUAL_UINT32(720, scan.pointCount());
  // Tia ROS 90° (bên trái) phủ góc gốc (269.5°, 270.5°]: điểm 270° (1270 mm) và
  // 270.5° (1270.5 mm) -> giữ điểm gần hơn
  TEST_ASSERT_EQUAL_UINT16(1270, scan.rangeMm(90));
  TEST_ASSERT_EQUAL_UINT16(1000, scan.rangeMm(0));
}

void test_scan_assembler_clamps_config() {
  amr::ScanAssemblerConfig cfg = defaultConfig();
  cfg.bins = 0;
  TEST_ASSERT_EQUAL_UINT16(1, amr::ScanAssembler(cfg).bins());
  cfg.bins = 5000;
  TEST_ASSERT_EQUAL_UINT16(amr::ScanAssembler::kMaxBins, amr::ScanAssembler(cfg).bins());
  cfg = defaultConfig();
  cfg.range_min_m = 0.0f;   // 0 mm dành cho "không phản hồi" -> tối thiểu 1 mm
  cfg.range_max_m = 100.0f; // giới hạn uint16 = 65.535 m
  amr::ScanAssembler scan(cfg);
  TEST_ASSERT_TRUE(scan.addPoint(0.0f, 1.0f, 0.0f));
  TEST_ASSERT_EQUAL_UINT16(1, scan.rangeMm(0));
  TEST_ASSERT_TRUE(scan.addPoint(90.0f, 65535.0f, 0.0f));
  TEST_ASSERT_FALSE(scan.addPoint(180.0f, 65536.0f, 0.0f));
}

// ---------------------------------------------------------------------------
// Đóng gói
// ---------------------------------------------------------------------------
void test_scan_packet_header_layout() {
  uint8_t buf[amr::kMaxUdpPayload];
  const size_t len = makePacket(buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT32(44 + 360 * 3, len);
  TEST_ASSERT_EQUAL_MEMORY("AMRL", buf, 4);
  TEST_ASSERT_EQUAL_UINT8(1, buf[4]);                     // version
  TEST_ASSERT_EQUAL_UINT8(1, buf[5]);                     // model YDLIDAR X2
  TEST_ASSERT_EQUAL_UINT8(0x03, buf[6]);                  // flags: intensity | ros_time
  TEST_ASSERT_EQUAL_UINT8(0x00, buf[7]);
  TEST_ASSERT_EQUAL_UINT32(42, readU32(buf + 8));         // seq (little-endian)
  const uint64_t stamp = readU32(buf + 12) | (static_cast<uint64_t>(readU32(buf + 16)) << 32);
  TEST_ASSERT_TRUE(stamp == 1700000000123456789ULL);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.142f, readF32(buf + 20));  // scan_time
  TEST_ASSERT_EQUAL_FLOAT(0.0f, readF32(buf + 24));             // angle_min
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, 2.0f * static_cast<float>(M_PI) / 360.0f, readF32(buf + 28));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, readF32(buf + 32));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 8.0f, readF32(buf + 36));
  TEST_ASSERT_EQUAL_UINT8(360 & 0xFF, buf[40]);           // num_ranges = 360
  TEST_ASSERT_EQUAL_UINT8(360 >> 8, buf[41]);
  TEST_ASSERT_EQUAL_UINT8(180, buf[42]);                  // num_valid = 180
  // ranges_mm[0] = 1000 (điểm gốc 0°) tại offset 44
  TEST_ASSERT_EQUAL_UINT8(1000 & 0xFF, buf[44]);
  TEST_ASSERT_EQUAL_UINT8(1000 >> 8, buf[45]);
}

void test_scan_packet_round_trip() {
  uint8_t buf[amr::kMaxUdpPayload];
  const size_t len = makePacket(buf, sizeof(buf));

  amr::ScanPacketView v;
  TEST_ASSERT_TRUE(amr::decodeScanPacket(buf, len, v));
  TEST_ASSERT_EQUAL_UINT8(1, v.version);
  TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(v.model));
  TEST_ASSERT_EQUAL_UINT16(amr::kScanFlagIntensity | amr::kScanFlagRosTime, v.flags);
  TEST_ASSERT_EQUAL_UINT32(42, v.seq);
  TEST_ASSERT_TRUE(v.stamp_ns == 1700000000123456789ULL);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.142f, v.scan_time_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f * static_cast<float>(M_PI) / 360.0f, v.angle_increment_rad);
  TEST_ASSERT_EQUAL_UINT16(360, v.num_ranges);
  TEST_ASSERT_EQUAL_UINT16(180, v.num_valid);
  // Điểm gốc 2° theo chiều kim đồng hồ -> tia 358 theo ROS
  TEST_ASSERT_EQUAL_UINT16(1002, v.rangeMm(358));
  TEST_ASSERT_EQUAL_UINT16(0, v.rangeMm(359));
  TEST_ASSERT_EQUAL_UINT16(0, v.rangeMm(360));  // ngoài mảng
  TEST_ASSERT_NOT_NULL(v.intensities);
  TEST_ASSERT_EQUAL_UINT8(7, v.intensities[358]);

  // Không kèm cường độ: gói ngắn hơn, intensities = nullptr, flag ros_time giữ nguyên
  const size_t len2 = makePacket(buf, sizeof(buf), false);
  TEST_ASSERT_EQUAL_UINT32(44 + 360 * 2, len2);
  TEST_ASSERT_TRUE(amr::decodeScanPacket(buf, len2, v));
  TEST_ASSERT_NULL(v.intensities);
  TEST_ASSERT_EQUAL_UINT16(amr::kScanFlagRosTime, v.flags);
  TEST_ASSERT_EQUAL_UINT16(1002, v.rangeMm(358));
}

void test_scan_packet_size_limits() {
  // 44 + 476*3 = 1472 byte: vừa khít 1 gói UDP không phân mảnh
  amr::ScanAssemblerConfig cfg = defaultConfig();
  static uint8_t buf[4096];
  amr::ScanMeta meta;
  cfg.bins = 476;
  TEST_ASSERT_EQUAL_UINT32(1472, amr::encodeScanPacket(amr::ScanAssembler(cfg), meta, true, buf,
                                                       sizeof(buf)));
  cfg.bins = 477;
  TEST_ASSERT_EQUAL_UINT32(0, amr::encodeScanPacket(amr::ScanAssembler(cfg), meta, true, buf,
                                                    sizeof(buf)));
  cfg.bins = 500;  // không cường độ: 44 + 500*2 = 1044
  TEST_ASSERT_EQUAL_UINT32(44 + 500 * 2, amr::encodeScanPacket(amr::ScanAssembler(cfg), meta,
                                                               false, buf, sizeof(buf)));
  // Bộ đệm quá nhỏ
  cfg.bins = 360;
  TEST_ASSERT_EQUAL_UINT32(0, amr::encodeScanPacket(amr::ScanAssembler(cfg), meta, true, buf,
                                                    44 + 360 * 3 - 1));
  TEST_ASSERT_EQUAL_UINT32(0, amr::encodeScanPacket(amr::ScanAssembler(cfg), meta, true, nullptr,
                                                    sizeof(buf)));
}

// Cấu hình trong robot_config.h phải tạo được gói hợp lệ.
void test_scan_packet_fits_robot_config() {
  TEST_ASSERT_TRUE(LIDAR_SCAN_BINS <= amr::ScanAssembler::kMaxBins);
  TEST_ASSERT_TRUE(amr::scanPacketSize(LIDAR_SCAN_BINS, LIDAR_SEND_INTENSITY != 0) <=
                   amr::kMaxUdpPayload);
  TEST_ASSERT_TRUE(LIDAR_RANGE_MIN_M > 0.0f && LIDAR_RANGE_MIN_M < LIDAR_RANGE_MAX_M);
  TEST_ASSERT_EQUAL_INT(8889, LIDAR_UDP_PORT);
}

// ---------------------------------------------------------------------------
// Giải mã
// ---------------------------------------------------------------------------
void test_scan_packet_decodes_python_encoder_output() {
  amr::ScanPacketView v;
  TEST_ASSERT_TRUE(amr::decodeScanPacket(kPythonPacket, sizeof(kPythonPacket), v));
  TEST_ASSERT_EQUAL_UINT8(2, static_cast<uint8_t>(v.model));  // Camsense X1
  TEST_ASSERT_EQUAL_UINT16(amr::kScanFlagIntensity | amr::kScanFlagRosTime, v.flags);
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, v.seq);
  TEST_ASSERT_TRUE(v.stamp_ns == 1700000000123456789ULL);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.19f, v.scan_time_s);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, v.angle_min_rad);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f * static_cast<float>(M_PI) / 8.0f, v.angle_increment_rad);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.08f, v.range_min_m);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 8.0f, v.range_max_m);
  TEST_ASSERT_EQUAL_UINT16(8, v.num_ranges);
  TEST_ASSERT_EQUAL_UINT16(5, v.num_valid);
  const uint16_t ranges[8] = {0, 1000, 65535, 1, 0, 2500, 300, 0};
  const uint8_t intensities[8] = {0, 10, 255, 1, 0, 128, 7, 0};
  for (uint16_t i = 0; i < 8; ++i) {
    TEST_ASSERT_EQUAL_UINT16(ranges[i], v.rangeMm(i));
    TEST_ASSERT_EQUAL_UINT8(intensities[i], v.intensities[i]);
  }
}

void test_scan_packet_rejects_malformed() {
  uint8_t buf[amr::kMaxUdpPayload + 1];
  const size_t len = makePacket(buf, sizeof(buf));
  amr::ScanPacketView v;
  TEST_ASSERT_TRUE(amr::decodeScanPacket(buf, len, v));

  TEST_ASSERT_FALSE(amr::decodeScanPacket(nullptr, len, v));
  for (size_t n = 0; n < len; ++n) {  // mọi độ dài bị cắt cụt
    TEST_ASSERT_FALSE(amr::decodeScanPacket(buf, n, v));
  }
  buf[len] = 0;
  TEST_ASSERT_FALSE(amr::decodeScanPacket(buf, len + 1, v));  // thừa byte

  uint8_t bad[amr::kMaxUdpPayload];
  std::memcpy(bad, buf, len);
  bad[0] = 'X';  // sai magic
  TEST_ASSERT_FALSE(amr::decodeScanPacket(bad, len, v));

  std::memcpy(bad, buf, len);
  bad[4] = 2;  // version lạ
  TEST_ASSERT_FALSE(amr::decodeScanPacket(bad, len, v));

  std::memcpy(bad, buf, len);
  bad[6] &= static_cast<uint8_t>(~amr::kScanFlagIntensity);  // cờ không khớp độ dài
  TEST_ASSERT_FALSE(amr::decodeScanPacket(bad, len, v));

  std::memcpy(bad, buf, len);
  writeF32(bad + 28, 0.0f);  // angle_increment = 0
  TEST_ASSERT_FALSE(amr::decodeScanPacket(bad, len, v));
  writeF32(bad + 28, NAN);
  TEST_ASSERT_FALSE(amr::decodeScanPacket(bad, len, v));
  writeF32(bad + 28, -0.01f);
  TEST_ASSERT_FALSE(amr::decodeScanPacket(bad, len, v));

  std::memcpy(bad, buf, len);
  writeF32(bad + 24, INFINITY);  // angle_min
  TEST_ASSERT_FALSE(amr::decodeScanPacket(bad, len, v));

  // 0 tia: header đúng 44 byte nhưng không có dữ liệu
  std::memcpy(bad, buf, amr::kScanHeaderSize);
  bad[40] = bad[41] = 0;
  TEST_ASSERT_FALSE(amr::decodeScanPacket(bad, amr::kScanHeaderSize, v));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_scan_bin_mapping_clockwise_to_ros);
  RUN_TEST(test_scan_keeps_closest_point_and_filters_range);
  RUN_TEST(test_scan_dense_revolution_fills_every_bin);
  RUN_TEST(test_scan_assembler_clamps_config);
  RUN_TEST(test_scan_packet_header_layout);
  RUN_TEST(test_scan_packet_round_trip);
  RUN_TEST(test_scan_packet_size_limits);
  RUN_TEST(test_scan_packet_fits_robot_config);
  RUN_TEST(test_scan_packet_decodes_python_encoder_output);
  RUN_TEST(test_scan_packet_rejects_malformed);
  return UNITY_END();
}
