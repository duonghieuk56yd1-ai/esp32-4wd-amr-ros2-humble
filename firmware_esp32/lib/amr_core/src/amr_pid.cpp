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

  // I với anti-windup
  const float i_candidate =
      clampf(i_term_ + gains_.ki * error * dt, -limits_.integral, limits_.integral);

  float unsaturated = p_term + i_candidate + d_term;
  const bool pushing_high = unsaturated > limits_.output && error > 0.0f;
  const bool pushing_low = unsaturated < -limits_.output && error < 0.0f;
  if (pushing_high || pushing_low) {
    // Đã bão hoà và sai số còn đẩy thêm: giữ nguyên I (không tích luỹ).
    unsaturated = p_term + i_term_ + d_term;
  } else {
    i_term_ = i_candidate;
  }

  saturated_ = unsaturated > limits_.output || unsaturated < -limits_.output;
  output_ = clampf(unsaturated, -limits_.output, limits_.output);
  return output_;
}

}  // namespace amr
