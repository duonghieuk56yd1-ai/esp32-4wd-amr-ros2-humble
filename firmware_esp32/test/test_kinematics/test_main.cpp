// Unit test: động học robot vi sai 2 bánh + tích phân odometry 2D.
// Chạy trên PC: pio test -e native -f test_kinematics
//
// Quy ước (REP-103): x tiến, y sang trái, yaw dương ngược chiều kim đồng hồ.
#include <unity.h>

#include <climits>
#include <cmath>
#include <cstdint>

#include "amr_diff_drive.h"
#include "amr_quadrature.h"
#include "robot_config.h"

void setUp() {}
void tearDown() {}

namespace {

constexpr float kR = 0.0325f;  // bán kính bánh (m)
constexpr float kL = 0.18f;    // khoảng cách 2 bánh (m)

}  // namespace

// ---------------------------------------------------------------------------
// Động học thuận / nghịch
// ---------------------------------------------------------------------------
void test_kinematics_known_values() {
  const amr::DiffDriveKinematics kin(kR, kL);
  // Đi thẳng 0.2 m/s: 2 bánh cùng tốc độ v / r
  amr::WheelPair w = kin.toWheelSpeeds(0.2f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.15385f, w.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 6.15385f, w.right);
  // Quay tại chỗ 1 rad/s ngược chiều kim đồng hồ: bánh PHẢI tiến, bánh TRÁI lùi
  w = kin.toWheelSpeeds(0.0f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.76923f, w.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.76923f, w.right);
  // Cung tròn: v = 0.3, w = 1 -> v_trái = (0.3 - 0.09)/r, v_phải = (0.3 + 0.09)/r
  w = kin.toWheelSpeeds(0.3f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, (0.3f - 0.09f) / kR, w.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, (0.3f + 0.09f) / kR, w.right);
  // Lùi: cả 2 bánh âm
  w = kin.toWheelSpeeds(-0.1f, 0.0f);
  TEST_ASSERT_TRUE(w.left < 0.0f && w.right < 0.0f);
}

void test_kinematics_round_trip_grid() {
  const amr::DiffDriveKinematics kin(kR, kL);
  for (int i = -4; i <= 4; ++i) {
    for (int j = -6; j <= 6; ++j) {
      const float v = 0.1f * static_cast<float>(i);  // -0.4 .. 0.4 m/s
      const float wz = 0.5f * static_cast<float>(j); // -3 .. 3 rad/s
      const amr::BodyTwist t = kin.toBodyTwist(kin.toWheelSpeeds(v, wz));
      TEST_ASSERT_FLOAT_WITHIN(1e-5f, v, t.linear);
      TEST_ASSERT_FLOAT_WITHIN(1e-4f, wz, t.angular);
    }
  }
}

void test_kinematics_wheels_to_twist() {
  const amr::DiffDriveKinematics kin(kR, kL);
  amr::WheelPair w;
  w.left = 10.0f;
  w.right = 10.0f;
  amr::BodyTwist t = kin.toBodyTwist(w);  // thẳng: v = r * w, không quay
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.325f, t.linear);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, t.angular);
  w.left = -5.0f;
  w.right = 5.0f;
  t = kin.toBodyTwist(w);  // quay tại chỗ CCW: w = r * (wR - wL) / L
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, t.linear);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, kR * 10.0f / kL, t.angular);
  w.left = 4.0f;
  w.right = 2.0f;
  t = kin.toBodyTwist(w);  // bánh trái nhanh hơn -> rẽ phải (w < 0)
  TEST_ASSERT_TRUE(t.linear > 0.0f && t.angular < 0.0f);
}

// ---------------------------------------------------------------------------
// Giới hạn tốc độ bánh (giữ nguyên độ cong quỹ đạo)
// ---------------------------------------------------------------------------
void test_limit_wheel_speeds_keeps_curvature() {
  amr::WheelPair in;
  in.left = 10.0f;
  in.right = 30.0f;
  const amr::WheelPair out = amr::limitWheelSpeeds(in, 15.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, out.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 15.0f, out.right);

  // Bán kính cong R = v / w không đổi sau khi co
  const amr::DiffDriveKinematics kin(kR, kL);
  const amr::BodyTwist before = kin.toBodyTwist(in);
  const amr::BodyTwist after = kin.toBodyTwist(out);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, before.linear / before.angular, after.linear / after.angular);

  in.left = -3.0f;
  in.right = 4.0f;
  const amr::WheelPair same = amr::limitWheelSpeeds(in, 15.0f);  // dưới ngưỡng: giữ nguyên
  TEST_ASSERT_EQUAL_FLOAT(-3.0f, same.left);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, same.right);
}

void test_limit_wheel_speeds_edge_cases() {
  amr::WheelPair in;
  in.left = -40.0f;
  in.right = 20.0f;
  amr::WheelPair out = amr::limitWheelSpeeds(in, -10.0f);  // ngưỡng âm = trị tuyệt đối
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -10.0f, out.left);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, out.right);

  in.left = 15.0f;
  in.right = -15.0f;
  out = amr::limitWheelSpeeds(in, 15.0f);  // đúng bằng ngưỡng
  TEST_ASSERT_EQUAL_FLOAT(15.0f, out.left);
  TEST_ASSERT_EQUAL_FLOAT(-15.0f, out.right);

  in.left = 0.0f;
  in.right = 0.0f;
  out = amr::limitWheelSpeeds(in, 0.0f);  // không chia cho 0
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right);
}

// Giới hạn trong robot_config.h phải tự nhất quán: /cmd_vel lớn nhất cho phép
// (đi thẳng hoặc quay tại chỗ) không vượt tốc độ bánh tối đa.
void test_robot_config_limits_are_consistent() {
  const amr::DiffDriveKinematics kin(WHEEL_RADIUS_M, WHEEL_SEPARATION_M);
  const amr::WheelPair straight = kin.toWheelSpeeds(MAX_LINEAR_VEL_MPS, 0.0f);
  const amr::WheelPair spin = kin.toWheelSpeeds(0.0f, MAX_ANGULAR_VEL_RAD_S);
  TEST_ASSERT_TRUE(straight.right <= MAX_WHEEL_SPEED_RAD_S);
  TEST_ASSERT_TRUE(spin.right <= MAX_WHEEL_SPEED_RAD_S);
  TEST_ASSERT_TRUE(ODOM_PUBLISH_HZ <= CONTROL_LOOP_HZ);
  TEST_ASSERT_TRUE(FAILSAFE_TIMEOUT_MS > 1000 / CONTROL_LOOP_HZ);
}

// ---------------------------------------------------------------------------
// Chuẩn hoá góc
// ---------------------------------------------------------------------------
void test_normalize_angle() {
  struct Case {
    double in;
    double out;
  };
  const Case cases[] = {
      {0.0, 0.0},
      {M_PI, M_PI},
      {-M_PI, M_PI},  // khoảng (-pi, pi]
      {M_PI / 2.0, M_PI / 2.0},
      {3.0 * M_PI / 2.0, -M_PI / 2.0},
      {-3.0 * M_PI / 2.0, M_PI / 2.0},
      {4.0 * M_PI, 0.0},
      {2.0 * M_PI + 0.25, 0.25},
      {-2.0 * M_PI - 0.25, -0.25},
      {100.0 * M_PI + 0.1, 0.1},
      {-1e-12, -1e-12},
  };
  for (const Case& c : cases) {
    const double out = amr::normalizeAngle(c.in);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, c.out, out);
    TEST_ASSERT_TRUE(out > -M_PI && out <= M_PI);
  }
}

// ---------------------------------------------------------------------------
// Odometry
// ---------------------------------------------------------------------------
void test_odometry_straight_forward_and_backward() {
  const double L = kL;
  amr::Odometry2D odom;
  for (int i = 0; i < 100; ++i) odom.update(0.01, 0.01, L);  // 1 m thẳng
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, odom.x());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, odom.y());
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, odom.yaw());
  for (int i = 0; i < 150; ++i) odom.update(-0.01, -0.01, L);  // lùi 1.5 m
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -0.5, odom.x());

  // Đi thẳng theo hướng 90°: tiến dọc trục y
  odom.reset(1.0, 2.0, M_PI / 2.0);
  odom.update(0.5, 0.5, L);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, odom.x());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.5, odom.y());
}

void test_odometry_spin_in_place() {
  const double L = kL;
  amr::Odometry2D odom;
  const double arc = M_PI * L / 200.0;  // 200 bước quay tại chỗ đủ 1 vòng
  for (int i = 0; i < 50; ++i) odom.update(-arc, arc, L);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, M_PI / 2.0, odom.yaw());  // 1/4 vòng CCW
  for (int i = 0; i < 150; ++i) odom.update(-arc, arc, L);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, odom.x());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, odom.y());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, std::sin(odom.yaw()));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, std::cos(odom.yaw()));
  // Quay theo chiều kim đồng hồ: yaw âm
  odom.reset();
  odom.update(arc, -arc, L);
  TEST_ASSERT_TRUE(odom.yaw() < 0.0);
}

void test_odometry_quarter_circle_left() {
  // Bán kính 0.5 m, rẽ trái 90°: kết thúc tại (R, R), yaw = +pi/2
  const double L = kL, R = 0.5, total = M_PI / 2.0;
  const int steps = 50;
  amr::Odometry2D odom;
  for (int i = 0; i < steps; ++i) {
    const double dth = total / steps;
    odom.update((R - L / 2.0) * dth, (R + L / 2.0) * dth, L);
  }
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, R, odom.x());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, R, odom.y());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, M_PI / 2.0, odom.yaw());
}

void test_odometry_full_circle_right_closes() {
  // Vòng tròn bán kính 0.3 m theo chiều kim đồng hồ: quay về đúng điểm xuất phát
  const double L = kL, R = 0.3;
  const int steps = 360;
  const double dth = 2.0 * M_PI / steps;
  amr::Odometry2D odom;
  double min_y = 0.0;
  for (int i = 0; i < steps; ++i) {
    odom.update((R + L / 2.0) * dth, (R - L / 2.0) * dth, L);  // trái nhanh hơn -> rẽ phải
    if (odom.y() < min_y) min_y = odom.y();
  }
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, odom.x());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, odom.y());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, std::sin(odom.yaw()));
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, -2.0 * R, min_y);  // tâm vòng tròn ở (0, -R)
}

// Tích phân cung tròn chính xác: 1 bước lớn = 1000 bước nhỏ cùng độ cong.
void test_odometry_independent_of_step_size() {
  const double L = kL, dl = 0.30, dr = 0.45;  // cung ~0.375 m, quay ~0.83 rad
  amr::Odometry2D big, small;
  big.reset(0.2, -0.1, 0.3);
  small.reset(0.2, -0.1, 0.3);
  big.update(dl, dr, L);
  for (int i = 0; i < 1000; ++i) small.update(dl / 1000.0, dr / 1000.0, L);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, big.x(), small.x());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, big.y(), small.y());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, big.yaw(), small.yaw());
}

// Hai nhánh tính (RK2 khi gần thẳng, cung tròn khi rẽ) đều khớp nghiệm chính xác
// (dây cung) quanh ngưỡng chuyển nhánh |dtheta| = 1e-6 -> không có bước nhảy.
void test_odometry_accurate_near_branch_threshold() {
  const double L = kL, ds = 0.05, yaw0 = 1.0;
  const double dthetas[] = {0.99e-6, 1.01e-6, -0.99e-6, -1.01e-6, 1e-4, 0.0};
  for (double dth : dthetas) {
    amr::Odometry2D odom;
    odom.reset(0.0, 0.0, yaw0);
    const double half = 0.5 * dth * L;
    odom.update(ds - half, ds + half, L);
    const double sinc = dth == 0.0 ? 1.0 : std::sin(0.5 * dth) / (0.5 * dth);
    const double chord = ds * sinc;
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, chord * std::cos(yaw0 + 0.5 * dth), odom.x());
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, chord * std::sin(yaw0 + 0.5 * dth), odom.y());
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, yaw0 + dth, odom.yaw());
  }
}

void test_odometry_reset_normalizes_yaw() {
  amr::Odometry2D odom;
  odom.reset(1.0, 2.0, 5.0 * M_PI / 2.0);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, odom.x());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, odom.y());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, M_PI / 2.0, odom.yaw());
}

// Toàn bộ chuỗi như firmware: tick encoder (bộ đếm 32 bit tràn giữa chừng) ->
// tickDelta -> rad -> quãng đường bánh -> odometry 50 Hz, so với nghiệm giải tích.
void test_odometry_from_encoder_ticks_matches_analytic_arc() {
  const amr::DiffDriveKinematics kin(kR, kL);
  const double v = 0.2, wz = 0.5;  // cung tròn bán kính 0.4 m
  const amr::WheelPair wheel =
      kin.toWheelSpeeds(static_cast<float>(v), static_cast<float>(wz));
  const double rate[2] = {wheel.left, wheel.right};  // rad/s
  const int ticks_per_rev = 1496;
  const double dt = 1.0 / 50.0;
  const int steps = 250;  // 5 s

  // Bộ đếm "ISR" bánh trái bắt đầu sát INT32_MAX để tràn trong lúc chạy
  uint32_t isr_count[2] = {static_cast<uint32_t>(INT32_MAX) - 2000u, 7u};
  int32_t prev[2] = {static_cast<int32_t>(isr_count[0]), static_cast<int32_t>(isr_count[1])};
  int64_t counted[2] = {0, 0};
  amr::Odometry2D odom;
  for (int s = 1; s <= steps; ++s) {
    double d_wheel[2];
    for (int i = 0; i < 2; ++i) {
      // Số tick nguyên mà encoder đã phát tới thời điểm s*dt (lượng tử hoá)
      const int64_t total = static_cast<int64_t>(
          std::floor(rate[i] * s * dt / (2.0 * M_PI) * ticks_per_rev));
      isr_count[i] += static_cast<uint32_t>(total - counted[i]);
      counted[i] = total;
      const int32_t now = static_cast<int32_t>(isr_count[i]);
      const int32_t d = amr::quad::tickDelta(now, prev[i]);
      prev[i] = now;
      d_wheel[i] = amr::quad::ticksToRadians(d, ticks_per_rev) * kR;
    }
    odom.update(d_wheel[0], d_wheel[1], kL);
  }
  TEST_ASSERT_TRUE(static_cast<int32_t>(isr_count[0]) < 0);  // đã thực sự tràn

  const double t = steps * dt;
  const double radius = v / wz;
  const double yaw = wz * t;
  // Sai số lượng tử hoá < 1 tick (~0.14 mm) mỗi bánh, không tích luỹ
  TEST_ASSERT_DOUBLE_WITHIN(2e-3, radius * std::sin(yaw), odom.x());
  TEST_ASSERT_DOUBLE_WITHIN(2e-3, radius * (1.0 - std::cos(yaw)), odom.y());
  TEST_ASSERT_DOUBLE_WITHIN(2e-3, yaw, odom.yaw());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_kinematics_known_values);
  RUN_TEST(test_kinematics_round_trip_grid);
  RUN_TEST(test_kinematics_wheels_to_twist);
  RUN_TEST(test_limit_wheel_speeds_keeps_curvature);
  RUN_TEST(test_limit_wheel_speeds_edge_cases);
  RUN_TEST(test_robot_config_limits_are_consistent);
  RUN_TEST(test_normalize_angle);
  RUN_TEST(test_odometry_straight_forward_and_backward);
  RUN_TEST(test_odometry_spin_in_place);
  RUN_TEST(test_odometry_quarter_circle_left);
  RUN_TEST(test_odometry_full_circle_right_closes);
  RUN_TEST(test_odometry_independent_of_step_size);
  RUN_TEST(test_odometry_accurate_near_branch_threshold);
  RUN_TEST(test_odometry_reset_normalizes_yaw);
  RUN_TEST(test_odometry_from_encoder_ticks_matches_analytic_arc);
  return UNITY_END();
}
