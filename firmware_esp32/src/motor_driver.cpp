#include "motor_driver.h"

#include <Arduino.h>
#include <cmath>

#include "pinout.h"
#include "robot_config.h"

static_assert(MOTOR_MAX_DUTY > 0.0f && MOTOR_MAX_DUTY <= 1.0f, "MOTOR_MAX_DUTY phải trong (0, 1]");
static_assert(MOTOR_PWM_FREQ_HZ * (1UL << MOTOR_PWM_RES_BITS) <= 80000000UL,
              "LEDC: tần số x 2^bit vượt 80 MHz");

void MotorDriverTB6612::begin() {
  // STBY xuống LOW trước tiên để motor chắc chắn đứng yên
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, LOW);
  standby_ = true;

  channels_[kLeft] = {PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM, MOTOR_PWM_CH_LEFT,
                      MOTOR_LEFT_INVERTED != 0, 0.0f};
  channels_[kRight] = {PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM, MOTOR_PWM_CH_RIGHT,
                       MOTOR_RIGHT_INVERTED != 0, 0.0f};
  pwm_max_ = (1UL << MOTOR_PWM_RES_BITS) - 1UL;

  for (Channel& ch : channels_) {
    pinMode(ch.in1, OUTPUT);
    pinMode(ch.in2, OUTPUT);
    digitalWrite(ch.in1, LOW);
    digitalWrite(ch.in2, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(ch.pwm_pin, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS, ch.ledc_channel);
#else
    ledcSetup(ch.ledc_channel, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(ch.pwm_pin, ch.ledc_channel);
#endif
    writePwm(ch, 0);
  }
}

void MotorDriverTB6612::writePwm(const Channel& ch, uint32_t value) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(ch.ledc_channel, value);
#else
  ledcWrite(ch.ledc_channel, value);
#endif
}

void MotorDriverTB6612::setStandby(bool standby) {
  if (standby_ != standby) {
    digitalWrite(PIN_MOTOR_STBY, standby ? LOW : HIGH);
    standby_ = standby;
  }
}

void MotorDriverTB6612::setDuty(Side side, float duty) {
  Channel& ch = channels_[side];
  if (!std::isfinite(duty)) duty = 0.0f;
  if (duty > MOTOR_MAX_DUTY) duty = MOTOR_MAX_DUTY;
  if (duty < -MOTOR_MAX_DUTY) duty = -MOTOR_MAX_DUTY;
  ch.duty = duty;

  const float hw = ch.inverted ? -duty : duty;
  const uint32_t counts = static_cast<uint32_t>(lroundf(fabsf(hw) * static_cast<float>(pwm_max_)));
  // duty = 0 -> PWM = 0 -> TB6612 phanh ngắn mạch (giữ bánh đứng yên)
  digitalWrite(ch.in1, hw >= 0.0f ? HIGH : LOW);
  digitalWrite(ch.in2, hw >= 0.0f ? LOW : HIGH);
  writePwm(ch, counts > pwm_max_ ? pwm_max_ : counts);
  setStandby(false);
}

void MotorDriverTB6612::brake() {
  for (Channel& ch : channels_) {
    writePwm(ch, 0);
    digitalWrite(ch.in1, HIGH);
    digitalWrite(ch.in2, HIGH);
    ch.duty = 0.0f;
  }
  setStandby(false);  // phanh cần STBY = HIGH
}

void MotorDriverTB6612::standby() {
  for (Channel& ch : channels_) {
    writePwm(ch, 0);
    digitalWrite(ch.in1, LOW);
    digitalWrite(ch.in2, LOW);
    ch.duty = 0.0f;
  }
  setStandby(true);
}
