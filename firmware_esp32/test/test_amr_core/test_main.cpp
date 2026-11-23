// Unit test cho lib/amr_core — chạy trên PC: pio test -e native
#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "amr_diff_drive.h"
#include "amr_pid.h"
#include "amr_quadrature.h"
#include "amr_scan_packet.h"

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Encoder quadrature
// ---------------------------------------------------------------------------
namespace {
// Mô phỏng ISR: trả về số đếm sau khi đi qua chuỗi trạng thái AB.
int32_t runQuadrature(const std::vector<uint8_t>& states, uint32_t* invalid = nullptr) {
  int32_t count = 0;
  uint8_t prev = states.front();
  for (size_t i = 1; i < states.size(); ++i) {
    const uint8_t code = amr::quad::transition(prev, states[i]);
    if (code == amr::quad::kInvalid && invalid) ++(*invalid);
    count += amr::quad::delta(code);
    prev = states[i];
  }
  return count;
}
}  // namespace

void test_quadrature_forward_and_reverse() {
  // 10 xung đủ chu kỳ theo chiều thuận = 40 tick
  std::vector<uint8_t> fwd{0b00};
  for (int i = 0; i < 10; ++i) {
    for (uint8_t s : {0b01, 0b11, 0b10, 0b00}) fwd.push_back(s);
  }
  TEST_ASSERT_EQUAL_INT32(40, runQuadrature(fwd));

  std::vector<uint8_t> rev{0b00};
  for (int i = 0; i < 10; ++i) {
    for (uint8_t s : {0b10, 0b11, 0b01, 0b00}) rev.push_back(s);
  }
  TEST_ASSERT_EQUAL_INT32(-40, runQuadrature(rev));
}

void test_quadrature_bounce_cancels_and_invalid_flagged() {
  // Nảy cạnh trên kênh B: 00 -> 01 -> 00 -> 01 -> 11 : thực chất tiến 2 bước
  TEST_ASSERT_EQUAL_INT32(2, runQuadrature({0b00, 0b01, 0b00, 0b01, 0b11}));

  uint32_t invalid = 0;
  // 00 -> 11 và 01 -> 10 là nhảy 2 bước: không đếm, chỉ báo lỗi
  TEST_ASSERT_EQUAL_INT32(0, runQuadrature({0b00, 0b11, 0b11}, &invalid));
  TEST_ASSERT_EQUAL_UINT32(1, invalid);
  TEST_ASSERT_EQUAL_UINT8(amr::quad::kInvalid, amr::quad::transition(0b01, 0b10));
  TEST_ASSERT_EQUAL_UINT8(amr::quad::kNone, amr::quad::transition(0b10, 0b10));
}

// ---------------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------------
void test_pid_proportional_and_clamp() {
  amr::PidController pid({2.0f, 0.0f, 0.0f}, {1.0f, 1.0f}, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, pid.update(0.2f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, pid.update(5.0f, 0.0f, 0.01f));   // kẹp trên
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, pid.update(-5.0f, 0.0f, 0.01f)); // kẹp dưới
  TEST_ASSERT_TRUE(pid.saturated());
}

void test_pid_integral_and_limit() {
  amr::PidController pid({0.0f, 1.0f, 0.0f}, {10.0f, 0.5f}, 0.0f);
  for (int i = 0; i < 10; ++i) pid.update(1.0f, 0.0f, 0.01f);  // I = 1*1*0.1
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, pid.integralTerm());
  for (int i = 0; i < 1000; ++i) pid.update(1.0f, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, pid.integralTerm());  // bị kẹp ở integral limit
  pid.reset();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integralTerm());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.output());
}

void test_pid_anti_windup_conditional_integration() {
  // Không có anti-windup, 2 s bão hoà với sai số 10 sẽ đẩy I lên 20 (kẹp 50).
  amr::PidController pid({0.5f, 1.0f, 0.0f}, {1.0f, 50.0f}, 0.0f);
  for (int i = 0; i < 200; ++i) pid.update(10.0f, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(pid.saturated());
  TEST_ASSERT_TRUE(pid.integralTerm() < 1.0f);  // I đã ngừng tích luỹ
  // Đổi dấu sai số: đầu ra phải rời bão hoà ngay, không bị "kẹt" vì windup.
  const float out = pid.update(-1.0f, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(out < 1.0f);
}

void test_pid_derivative_on_measurement_no_kick() {
  amr::PidController pid({0.0f, 0.0f, 1.0f}, {100.0f, 1.0f}, 0.0f);
  pid.update(0.0f, 1.0f, 0.01f);
  // Nhảy setpoint, đo không đổi -> D = 0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.update(50.0f, 1.0f, 0.01f));
  // Giá trị đo tăng 0.1 trong 0.01 s -> D = -10
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -10.0f, pid.update(50.0f, 1.1f, 0.01f));
}

void test_pid_rejects_invalid_input() {
  amr::PidController pid({1.0f, 0.0f, 0.0f}, {1.0f, 1.0f}, 0.0f);
  const float out = pid.update(0.5f, 0.0f, 0.01f);
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(0.9f, 0.0f, 0.0f));
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(NAN, 0.0f, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(0.9f, INFINITY, 0.01f));
}

// Vòng kín với mô hình motor bậc 1 (K = 20 rad/s @ duty 1, tau = 80 ms) có ma sát
// tĩnh (|duty| < 0.08 không quay) và gain mặc định trong robot_config.h.
void test_pid_closed_loop_default_gains() {
  amr::PidController pid({0.05f, 0.60f, 0.0f}, {1.0f, 0.8f}, 0.02f);
  const float dt = 0.02f;  // 50 Hz
  const float K = 20.0f, tau = 0.08f, target = 8.0f;
  float w = 0.0f, peak = 0.0f;
  for (int i = 0; i < 100; ++i) {  // 2 s
    const float u = pid.update(target, w, dt);
    const float drive = std::fabs(u) < 0.08f ? 0.0f : u;
    w += dt / tau * (K * drive - w);
    if (w > peak) peak = w;
  }
  TEST_ASSERT_FLOAT_WITHIN(0.2f, target, w);   // bám sai số < 2.5 %
  TEST_ASSERT_TRUE(peak < target * 1.25f);     // vọt lố < 25 %
}

// ---------------------------------------------------------------------------
// Động học & odometry
// ---------------------------------------------------------------------------
void test_kinematics_round_trip() {
  amr::DiffDriveKinematics kin(0.0325f, 0.18f);
  const amr::WheelPair w = kin.toWheelSpeeds(0.3f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, (0.3f - 0.09f) / 0.0325f, w.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, (0.3f + 0.09f) / 0.0325f, w.right);
  const amr::BodyTwist t = kin.toBodyTwist(w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, t.linear);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, t.angular);
}

void test_limit_wheel_speeds_keeps_curvature() {
  const amr::WheelPair out = amr::limitWheelSpeeds({10.0f, 30.0f}, 15.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, out.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 15.0f, out.right);
  const amr::WheelPair same = amr::limitWheelSpeeds({-3.0f, 4.0f}, 15.0f);
  TEST_ASSERT_EQUAL_FLOAT(-3.0f, same.left);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, same.right);
}

void test_normalize_angle() {
  TEST_ASSERT_FLOAT_WITHIN(1e-9, M_PI, amr::normalizeAngle(-M_PI));
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, amr::normalizeAngle(4.0 * M_PI));
  TEST_ASSERT_FLOAT_WITHIN(1e-9, -M_PI / 2.0, amr::normalizeAngle(3.0 * M_PI / 2.0));
}

void test_odometry_straight_and_spin() {
  const double L = 0.18;
  amr::Odometry2D odom;
  for (int i = 0; i < 100; ++i) odom.update(0.01, 0.01, L);  // 1 m thẳng
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 1.0, odom.x());
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, odom.y());

  odom.reset();
  const double arc = M_PI * L / 200.0;  // 200 bước quay tại chỗ đủ 1 vòng
  for (int i = 0; i < 200; ++i) odom.update(-arc, arc, L);
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, odom.x());
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, odom.y());
  TEST_ASSERT_FLOAT_WITHIN(1e-9, 0.0, std::sin(odom.yaw()));
}

void test_odometry_quarter_circle_left() {
  // Bán kính 0.5 m, rẽ trái 90°: kết thúc tại (R, R), yaw = +pi/2
  const double L = 0.18, R = 0.5, total = M_PI / 2.0;
  const int steps = 50;
  amr::Odometry2D odom;
  for (int i = 0; i < steps; ++i) {
    const double dth = total / steps;
    odom.update((R - L / 2.0) * dth, (R + L / 2.0) * dth, L);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-9, R, odom.x());
  TEST_ASSERT_FLOAT_WITHIN(1e-9, R, odom.y());
  TEST_ASSERT_FLOAT_WITHIN(1e-9, M_PI / 2.0, odom.yaw());
}

// ---------------------------------------------------------------------------
// LiDAR: gom tia & đóng gói
// ---------------------------------------------------------------------------
void test_scan_bin_mapping_clockwise_to_ros() {
  amr::ScanAssemblerConfig cfg;
  cfg.bins = 360;
  cfg.clockwise = true;
  amr::ScanAssembler scan(cfg);
  TEST_ASSERT_EQUAL_INT(0, scan.binIndex(0.0f));
  TEST_ASSERT_EQUAL_INT(270, scan.binIndex(90.0f));   // bên phải robot = -90° (ROS)
  TEST_ASSERT_EQUAL_INT(90, scan.binIndex(270.0f));   // bên trái = +90°
  TEST_ASSERT_EQUAL_INT(0, scan.binIndex(0.4f));      // làm tròn về tia gần nhất
  TEST_ASSERT_EQUAL_INT(1, scan.binIndex(359.0f));
  TEST_ASSERT_EQUAL_INT(-1, scan.binIndex(NAN));

  cfg.clockwise = false;
  cfg.yaw_offset_deg = 10.0f;
  amr::ScanAssembler ccw(cfg);
  TEST_ASSERT_EQUAL_INT(10, ccw.binIndex(0.0f));
  TEST_ASSERT_EQUAL_INT(0, ccw.binIndex(350.0f));
}

void test_scan_keeps_closest_point_and_filters_range() {
  amr::ScanAssemblerConfig cfg;
  cfg.bins = 360;
  cfg.range_min_m = 0.1f;
  cfg.range_max_m = 8.0f;
  amr::ScanAssembler scan(cfg);
  TEST_ASSERT_TRUE(scan.addPoint(0.0f, 1500.0f, 10.0f));
  TEST_ASSERT_TRUE(scan.addPoint(0.2f, 1200.0f, 99.0f));   // cùng tia, gần hơn
  TEST_ASSERT_FALSE(scan.addPoint(-0.2f, 1300.0f, 5.0f));  // cùng tia, xa hơn
  TEST_ASSERT_FALSE(scan.addPoint(10.0f, 0.0f, 10.0f));    // 0 = không hợp lệ
  TEST_ASSERT_FALSE(scan.addPoint(10.0f, 50.0f, 10.0f));   // < range_min
  TEST_ASSERT_FALSE(scan.addPoint(10.0f, 9000.0f, 10.0f)); // > range_max
  TEST_ASSERT_TRUE(scan.addPoint(180.0f, 800.4f, 300.0f)); // quality kẹp về 255
  TEST_ASSERT_EQUAL_UINT16(1200, scan.rangeMm(0));
  TEST_ASSERT_EQUAL_UINT8(99, scan.intensity(0));
  TEST_ASSERT_EQUAL_UINT16(800, scan.rangeMm(180));
  TEST_ASSERT_EQUAL_UINT8(255, scan.intensity(180));
  TEST_ASSERT_EQUAL_UINT16(2, scan.validCount());
  TEST_ASSERT_EQUAL_UINT32(7, scan.pointCount());
  scan.clear();
  TEST_ASSERT_EQUAL_UINT16(0, scan.validCount());
  TEST_ASSERT_EQUAL_UINT16(0, scan.rangeMm(0));
}

void test_scan_packet_round_trip() {
  amr::ScanAssemblerConfig cfg;
  cfg.bins = 360;
  cfg.range_min_m = 0.1f;
  cfg.range_max_m = 8.0f;
  amr::ScanAssembler scan(cfg);
  for (int deg = 0; deg < 360; deg += 2) scan.addPoint(static_cast<float>(deg), 1000.0f + deg, 7.0f);

  amr::ScanMeta meta;
  meta.model = amr::LidarModel::kYdlidarX2;
  meta.seq = 42;
  meta.stamp_ns = 1700000000123456789ULL;
  meta.ros_time = true;
  meta.scan_time_s = 0.142f;

  uint8_t buf[amr::kMaxUdpPayload];
  const size_t len = amr::encodeScanPacket(scan, meta, true, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT32(amr::scanPacketSize(360, true), len);
  TEST_ASSERT_EQUAL_UINT32(44 + 360 * 3, len);
  TEST_ASSERT_EQUAL_MEMORY("AMRL", buf, 4);

  amr::ScanPacketView v;
  TEST_ASSERT_TRUE(amr::decodeScanPacket(buf, len, v));
  TEST_ASSERT_EQUAL_UINT8(1, v.version);
  TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(v.model));
  TEST_ASSERT_EQUAL_UINT16(amr::kScanFlagIntensity | amr::kScanFlagRosTime, v.flags);
  TEST_ASSERT_EQUAL_UINT32(42, v.seq);
  TEST_ASSERT_TRUE(v.stamp_ns == meta.stamp_ns);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.142f, v.scan_time_s);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f * static_cast<float>(M_PI) / 360.0f, v.angle_increment_rad);
  TEST_ASSERT_EQUAL_UINT16(360, v.num_ranges);
  TEST_ASSERT_EQUAL_UINT16(180, v.num_valid);
  // Điểm gốc 2° theo chiều kim đồng hồ -> tia 358 theo ROS
  TEST_ASSERT_EQUAL_UINT16(1002, v.rangeMm(358));
  TEST_ASSERT_EQUAL_UINT16(0, v.rangeMm(359));
  TEST_ASSERT_NOT_NULL(v.intensities);
  TEST_ASSERT_EQUAL_UINT8(7, v.intensities[358]);

  // Gói hỏng: sai độ dài, sai magic
  TEST_ASSERT_FALSE(amr::decodeScanPacket(buf, len - 1, v));
  buf[0] = 'X';
  TEST_ASSERT_FALSE(amr::decodeScanPacket(buf, len, v));
}

void test_scan_packet_too_large_is_rejected() {
  amr::ScanAssemblerConfig cfg;
  cfg.bins = 500;  // 44 + 500*3 = 1544 > 1472
  amr::ScanAssembler scan(cfg);
  static uint8_t buf[4096];
  amr::ScanMeta meta;
  TEST_ASSERT_EQUAL_UINT32(0, amr::encodeScanPacket(scan, meta, true, buf, sizeof(buf)));
  TEST_ASSERT_EQUAL_UINT32(44 + 500 * 2, amr::encodeScanPacket(scan, meta, false, buf, sizeof(buf)));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_quadrature_forward_and_reverse);
  RUN_TEST(test_quadrature_bounce_cancels_and_invalid_flagged);
  RUN_TEST(test_pid_proportional_and_clamp);
  RUN_TEST(test_pid_integral_and_limit);
  RUN_TEST(test_pid_anti_windup_conditional_integration);
  RUN_TEST(test_pid_derivative_on_measurement_no_kick);
  RUN_TEST(test_pid_rejects_invalid_input);
  RUN_TEST(test_pid_closed_loop_default_gains);
  RUN_TEST(test_kinematics_round_trip);
  RUN_TEST(test_limit_wheel_speeds_keeps_curvature);
  RUN_TEST(test_normalize_angle);
  RUN_TEST(test_odometry_straight_and_spin);
  RUN_TEST(test_odometry_quarter_circle_left);
  RUN_TEST(test_scan_bin_mapping_clockwise_to_ros);
  RUN_TEST(test_scan_keeps_closest_point_and_filters_range);
  RUN_TEST(test_scan_packet_round_trip);
  RUN_TEST(test_scan_packet_too_large_is_rejected);
  return UNITY_END();
}
