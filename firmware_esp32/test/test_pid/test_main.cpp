// Unit test: bộ PID vận tốc bánh (anti-windup, D trên giá trị đo + lọc thông thấp)
// và vòng kín với mô hình motor JGA25-370 + encoder thật (lượng tử hoá, lọc EMA).
// Chạy trên PC: pio test -e native -f test_pid
#include <unity.h>

#include <cmath>
#include <cstdint>

#include "amr_pid.h"
#include "robot_config.h"

void setUp() {}
void tearDown() {}

namespace {

// Bộ sinh số ngẫu nhiên cố định hạt giống (kết quả test lặp lại được).
struct Lcg {
  uint32_t state;
  explicit Lcg(uint32_t seed) : state(seed) {}
  float uniform(float lo, float hi) {
    state = state * 1664525u + 1013904223u;
    return lo + (hi - lo) * static_cast<float>(state >> 8) / 16777216.0f;
  }
};

// Motor + encoder như firmware: JGA25-370 1:34 @ 12 V (ước lượng) ~20 rad/s ở
// duty 1, hằng số thời gian 80 ms, ma sát tĩnh (|duty| < 0.08 không quay).
// Vận tốc "đo" = số tick encoder nguyên trong chu kỳ -> rad/s -> lọc EMA,
// đúng như control_task.cpp.
struct WheelSim {
  double gain = 20.0;      // rad/s ở duty 1
  double tau = 0.08;       // s
  double deadband = 0.08;  // duty
  double load = 0.0;       // tải (rad/s bị mất ở trạng thái xác lập)
  double w = 0.0;          // vận tốc thật (rad/s)
  double angle = 0.0;      // góc thật (rad)
  int64_t ticks_read = 0;
  float measured = 0.0f;   // vận tốc firmware nhìn thấy

  void step(float duty, float dt) {
    const int kSub = 20;
    const double h = static_cast<double>(dt) / kSub;
    const double drive = std::fabs(duty) < deadband ? 0.0 : static_cast<double>(duty);
    for (int k = 0; k < kSub; ++k) {
      double accel = (gain * drive - w) / tau;
      if (w > 0.0) accel -= load / tau;
      w += h * accel;
      angle += w * h;
    }
    const int64_t ticks =
        static_cast<int64_t>(std::floor(angle / (2.0 * M_PI) * ENCODER_TICKS_PER_REV));
    const double raw = static_cast<double>(ticks - ticks_read) * 2.0 * M_PI /
                       ENCODER_TICKS_PER_REV / static_cast<double>(dt);
    ticks_read = ticks;
    measured += WHEEL_VEL_FILTER_ALPHA * (static_cast<float>(raw) - measured);
  }
};

amr::PidController makeDefaultPid() {
  return amr::PidController({PID_KP, PID_KI, PID_KD}, {MOTOR_MAX_DUTY, PID_INTEGRAL_LIMIT},
                            PID_D_FILTER_TAU_S);
}

constexpr float kDt = 1.0f / CONTROL_LOOP_HZ;

}  // namespace

// ---------------------------------------------------------------------------
// Các thành phần P, I, D
// ---------------------------------------------------------------------------
void test_pid_proportional_and_clamp() {
  amr::PidController pid({2.0f, 0.0f, 0.0f}, {1.0f, 1.0f}, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, pid.update(0.2f, 0.0f, 0.01f));
  TEST_ASSERT_FALSE(pid.saturated());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, pid.update(5.0f, 0.0f, 0.01f));    // kẹp trên
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, pid.update(-5.0f, 0.0f, 0.01f));  // kẹp dưới
  TEST_ASSERT_TRUE(pid.saturated());
}

void test_pid_integral_and_limit() {
  amr::PidController pid({0.0f, 1.0f, 0.0f}, {10.0f, 0.5f}, 0.0f);
  for (int i = 0; i < 10; ++i) pid.update(1.0f, 0.0f, 0.01f);  // I = ki * e * t = 0.1
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, pid.integralTerm());
  for (int i = 0; i < 1000; ++i) pid.update(1.0f, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, pid.integralTerm());  // kẹp ở integral limit
  for (int i = 0; i < 1000; ++i) pid.update(-1.0f, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.5f, pid.integralTerm());
}

void test_pid_derivative_on_measurement_no_kick() {
  amr::PidController pid({0.0f, 0.0f, 1.0f}, {100.0f, 1.0f}, 0.0f);
  pid.update(0.0f, 1.0f, 0.01f);
  // Nhảy setpoint, giá trị đo không đổi -> D = 0 (không "đá" motor)
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.update(50.0f, 1.0f, 0.01f));
  // Giá trị đo tăng 0.1 trong 0.01 s -> D = -10
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -10.0f, pid.update(50.0f, 1.1f, 0.01f));
}

void test_pid_derivative_low_pass_filter() {
  // tau = 0.02 s, dt = 0.01 s -> alpha = dt / (tau + dt) = 1/3
  amr::PidController pid({0.0f, 0.0f, 1.0f}, {100.0f, 1.0f}, 0.02f);
  pid.update(0.0f, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -10.0f / 3.0f, pid.update(0.0f, 0.1f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -20.0f / 9.0f, pid.update(0.0f, 0.1f, 0.01f));  // tắt dần
  for (int i = 0; i < 200; ++i) pid.update(0.0f, 0.1f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, pid.output());
}

// ---------------------------------------------------------------------------
// Anti-windup
// ---------------------------------------------------------------------------
void test_pid_anti_windup_conditional_integration() {
  // Không có anti-windup, 2 s bão hoà với sai số 10 sẽ đẩy I lên 20 (kẹp 50).
  amr::PidController pid({0.5f, 1.0f, 0.0f}, {1.0f, 50.0f}, 0.0f);
  for (int i = 0; i < 200; ++i) pid.update(10.0f, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(pid.saturated());
  TEST_ASSERT_TRUE(pid.integralTerm() < 1.0f);  // I đã ngừng tích luỹ
  // Đổi dấu sai số: đầu ra phải rời bão hoà ngay, không bị "kẹt" vì windup.
  TEST_ASSERT_TRUE(pid.update(-1.0f, 0.0f, 0.01f) < 1.0f);
}

void test_pid_output_and_integral_always_bounded() {
  amr::PidController pid({0.3f, 2.0f, 0.01f}, {1.0f, 0.8f}, 0.02f);
  Lcg rng(12345u);
  for (int i = 0; i < 20000; ++i) {
    const float out = pid.update(rng.uniform(-30.0f, 30.0f), rng.uniform(-30.0f, 30.0f),
                                 rng.uniform(0.001f, 0.05f));
    TEST_ASSERT_TRUE(std::isfinite(out));
    TEST_ASSERT_TRUE(std::fabs(out) <= 1.0f);
    TEST_ASSERT_TRUE(std::fabs(pid.integralTerm()) <= 0.8f);
  }
}

// ---------------------------------------------------------------------------
// Đối xứng, reset, đổi tham số lúc chạy, đầu vào lỗi
// ---------------------------------------------------------------------------
void test_pid_is_odd_symmetric() {
  // Tiến và lùi phải được điều khiển y hệt nhau (chỉ đổi dấu)
  amr::PidController fwd({0.05f, 0.6f, 0.002f}, {1.0f, 0.8f}, 0.02f);
  amr::PidController rev({0.05f, 0.6f, 0.002f}, {1.0f, 0.8f}, 0.02f);
  Lcg rng(777u);
  for (int i = 0; i < 2000; ++i) {
    const float sp = rng.uniform(-20.0f, 20.0f);
    const float meas = rng.uniform(-20.0f, 20.0f);
    const float dt = rng.uniform(0.005f, 0.03f);
    // (gọi update() ngoài macro: TEST_ASSERT_EQUAL_FLOAT tính tham số expected 2 lần)
    const float out_fwd = fwd.update(sp, meas, dt);
    const float out_rev = rev.update(-sp, -meas, dt);
    TEST_ASSERT_EQUAL_FLOAT(-out_fwd, out_rev);
  }
}

void test_pid_reset_clears_state() {
  amr::PidController pid({1.0f, 1.0f, 1.0f}, {1.0f, 0.5f}, 0.0f);
  for (int i = 0; i < 100; ++i) pid.update(10.0f, 0.0f, 0.01f);
  pid.reset();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integralTerm());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.output());
  TEST_ASSERT_FALSE(pid.saturated());
  // Lần cập nhật đầu sau reset không còn giá trị đo cũ (0) -> không có xung D
  // (nếu còn: D = -kd * (5 - 0) / 0.01 = -500 -> kẹp -1)
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.update(5.0f, 5.0f, 0.01f));
}

void test_pid_set_limits_clamps_integral() {
  amr::PidController pid({0.0f, 1.0f, 0.0f}, {1.0f, 0.8f}, 0.0f);
  for (int i = 0; i < 1000; ++i) pid.update(1.0f, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, pid.integralTerm());
  pid.setLimits({1.0f, 0.3f});
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, pid.integralTerm());
  pid.setLimits({-2.0f, -0.5f});  // giới hạn âm được hiểu là trị tuyệt đối
  for (int i = 0; i < 1000; ++i) pid.update(1.0f, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, pid.integralTerm());
  pid.setGains({1.0f, 1.0f, 0.0f});
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, pid.update(10.0f, 0.0f, 0.01f));
}

void test_pid_change_gains_is_bumpless() {
  // I lưu theo đơn vị đầu ra -> đổi KI lúc chạy không làm nhảy đầu ra
  amr::PidController pid({0.1f, 0.6f, 0.0f}, {1.0f, 0.8f}, 0.0f);
  for (int i = 0; i < 50; ++i) pid.update(2.0f, 1.0f, 0.02f);
  const float before = pid.update(1.0f, 1.0f, 0.02f);  // sai số 0: đầu ra = I
  pid.setGains({0.1f, 1.2f, 0.0f});
  TEST_ASSERT_EQUAL_FLOAT(before, pid.update(1.0f, 1.0f, 0.02f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.2f, pid.gains().ki);
}

void test_pid_rejects_invalid_input() {
  amr::PidController pid({1.0f, 1.0f, 0.0f}, {1.0f, 1.0f}, 0.0f);
  const float out = pid.update(0.5f, 0.0f, 0.01f);
  const float integral = pid.integralTerm();
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(0.9f, 0.0f, 0.0f));    // dt = 0
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(0.9f, 0.0f, -0.01f));  // dt âm
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(NAN, 0.0f, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(0.9f, INFINITY, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(0.9f, 0.0f, NAN));
  TEST_ASSERT_EQUAL_FLOAT(integral, pid.integralTerm());  // I không bị nhiễm NaN
}

// ---------------------------------------------------------------------------
// Vòng kín với gain mặc định trong robot_config.h
// ---------------------------------------------------------------------------
void test_pid_closed_loop_step_response() {
  amr::PidController pid = makeDefaultPid();
  WheelSim wheel;
  const float target = 8.0f;  // rad/s ~ 0.26 m/s
  double peak = 0.0, avg = 0.0;
  const int steps = 3 * CONTROL_LOOP_HZ;  // 3 s
  for (int i = 0; i < steps; ++i) {
    wheel.step(pid.update(target, wheel.measured, kDt), kDt);
    if (wheel.w > peak) peak = wheel.w;
    if (i >= steps - CONTROL_LOOP_HZ) avg += wheel.w / CONTROL_LOOP_HZ;  // trung bình giây cuối
  }
  TEST_ASSERT_DOUBLE_WITHIN(0.2, target, avg);  // sai số xác lập < 2.5 %
  TEST_ASSERT_TRUE(peak < target * 1.25);       // vọt lố < 25 %
}

void test_pid_closed_loop_reverse_and_stop() {
  amr::PidController pid = makeDefaultPid();
  WheelSim wheel;
  for (int i = 0; i < 2 * CONTROL_LOOP_HZ; ++i) {
    wheel.step(pid.update(-6.0f, wheel.measured, kDt), kDt);
  }
  TEST_ASSERT_DOUBLE_WITHIN(0.3, -6.0, wheel.w);
  for (int i = 0; i < 2 * CONTROL_LOOP_HZ; ++i) {
    wheel.step(pid.update(0.0f, wheel.measured, kDt), kDt);
  }
  TEST_ASSERT_DOUBLE_WITHIN(0.3, 0.0, wheel.w);
}

void test_pid_closed_loop_rejects_load() {
  // Tải nặng (mất 5 rad/s ở trạng thái xác lập): thành phần I phải bù hết
  amr::PidController pid = makeDefaultPid();
  WheelSim wheel;
  wheel.load = 5.0;
  double avg = 0.0;
  const int steps = 4 * CONTROL_LOOP_HZ;
  for (int i = 0; i < steps; ++i) {
    wheel.step(pid.update(8.0f, wheel.measured, kDt), kDt);
    if (i >= steps - CONTROL_LOOP_HZ) avg += wheel.w / CONTROL_LOOP_HZ;
  }
  TEST_ASSERT_DOUBLE_WITHIN(0.3, 8.0, avg);
}

void test_pid_closed_loop_recovers_after_saturation() {
  // Đặt tốc độ không thể đạt (30 > 20 rad/s) trong 3 s rồi hạ về 5 rad/s:
  // nhờ anti-windup bánh phải về gần 5 rad/s trong vòng 0.6 s.
  amr::PidController pid = makeDefaultPid();
  WheelSim wheel;
  for (int i = 0; i < 3 * CONTROL_LOOP_HZ; ++i) {
    wheel.step(pid.update(30.0f, wheel.measured, kDt), kDt);
  }
  TEST_ASSERT_TRUE(pid.output() > 0.95f * MOTOR_MAX_DUTY);  // duty kịch trần
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 20.0, wheel.w);              // tốc độ tối đa của motor
  TEST_ASSERT_TRUE(pid.integralTerm() <= PID_INTEGRAL_LIMIT);
  int settle_steps = -1;
  for (int i = 0; i < 2 * CONTROL_LOOP_HZ; ++i) {
    wheel.step(pid.update(5.0f, wheel.measured, kDt), kDt);
    const bool inside = std::fabs(wheel.w - 5.0) < 0.5;
    if (inside && settle_steps < 0) settle_steps = i;
    if (!inside) settle_steps = -1;  // phải ở yên trong dải
  }
  TEST_ASSERT_TRUE(settle_steps >= 0);
  TEST_ASSERT_TRUE(settle_steps * kDt < 0.6f);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_pid_proportional_and_clamp);
  RUN_TEST(test_pid_integral_and_limit);
  RUN_TEST(test_pid_derivative_on_measurement_no_kick);
  RUN_TEST(test_pid_derivative_low_pass_filter);
  RUN_TEST(test_pid_anti_windup_conditional_integration);
  RUN_TEST(test_pid_output_and_integral_always_bounded);
  RUN_TEST(test_pid_is_odd_symmetric);
  RUN_TEST(test_pid_reset_clears_state);
  RUN_TEST(test_pid_set_limits_clamps_integral);
  RUN_TEST(test_pid_change_gains_is_bumpless);
  RUN_TEST(test_pid_rejects_invalid_input);
  RUN_TEST(test_pid_closed_loop_step_response);
  RUN_TEST(test_pid_closed_loop_reverse_and_stop);
  RUN_TEST(test_pid_closed_loop_rejects_load);
  RUN_TEST(test_pid_closed_loop_recovers_after_saturation);
  return UNITY_END();
}
