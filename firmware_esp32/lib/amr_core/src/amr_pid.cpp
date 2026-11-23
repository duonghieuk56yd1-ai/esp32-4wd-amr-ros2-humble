#include "amr_pid.h"

#include <cmath>

namespace amr {

namespace {
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

PidController::PidController(const PidGains& gains, const PidLimits& limits,
                             float d_filter_tau_s)
    : gains_(gains), d_tau_(d_filter_tau_s > 0.0f ? d_filter_tau_s : 0.0f) {
  setLimits(limits);
}

void PidController::setLimits(const PidLimits& limits) {
  limits_.output = std::fabs(limits.output);
  limits_.integral = std::fabs(limits.integral);
  i_term_ = clampf(i_term_, -limits_.integral, limits_.integral);
}

void PidController::reset() {
  i_term_ = 0.0f;
  d_filtered_ = 0.0f;
  prev_measurement_ = 0.0f;
  output_ = 0.0f;
  has_prev_ = false;
  saturated_ = false;
}

float PidController::update(float setpoint, float measurement, float dt) {
  if (!(dt > 0.0f) || !std::isfinite(dt) || !std::isfinite(setpoint) ||
      !std::isfinite(measurement)) {
    return output_;
  }

  const float error = setpoint - measurement;
  const float p_term = gains_.kp * error;

  // D trên giá trị đo, lọc thông thấp: d = -d(measurement)/dt
  float d_term = 0.0f;
  if (has_prev_) {
    const float d_raw = -(measurement - prev_measurement_) / dt;
    const float alpha = dt / (d_tau_ + dt);  // d_tau_ = 0 -> alpha = 1 (không lọc)
    d_filtered_ += alpha * (d_raw - d_filtered_);
    d_term = gains_.kd * d_filtered_;
  }
  prev_measurement_ = measurement;
  has_prev_ = true;

  // I với anti-windup: khi sai số còn đẩy đầu ra vào bão hoà, chỉ tích phân tới
  // đúng mức đầu ra vừa chạm giới hạn (đầu ra đạt 100 % duty) rồi dừng; không
  // tích luỹ vượt mức đó và cũng không tự xả phần I đã có.
  const float pd = p_term + d_term;
  float i_next = clampf(i_term_ + gains_.ki * error * dt, -limits_.integral, limits_.integral);
  if (pd + i_next > limits_.output && error > 0.0f) {
    i_next = std::fmax(i_term_, std::fmin(i_next, limits_.output - pd));
  } else if (pd + i_next < -limits_.output && error < 0.0f) {
    i_next = std::fmin(i_term_, std::fmax(i_next, -limits_.output - pd));
  }
  i_term_ = i_next;

  const float unsaturated = pd + i_term_;
  saturated_ = std::fabs(unsaturated) >= limits_.output;
  output_ = clampf(unsaturated, -limits_.output, limits_.output);
  return output_;
}

}  // namespace amr
