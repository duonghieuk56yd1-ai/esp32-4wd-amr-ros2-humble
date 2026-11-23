// =============================================================================
//  motor_driver.h — Driver TB6612FNG (2 kênh) với PWM phần cứng LEDC.
//
//  Bảng chân lý TB6612FNG:
//    IN1 IN2 PWM  STBY | Kết quả
//     H   L   H    H   | quay thuận          (PWM = L -> phanh ngắn mạch)
//     L   H   H    H   | quay nghịch         (PWM = L -> phanh ngắn mạch)
//     H   H   x    H   | phanh ngắn mạch
//     x   x   x    L   | standby: ngõ ra hở, motor nhả trôi
// =============================================================================
#pragma once

#include <stdint.h>

class MotorDriverTB6612 {
 public:
  enum Side : uint8_t { kLeft = 0, kRight = 1 };

  // Đưa mọi chân về trạng thái an toàn (STBY = LOW, PWM = 0). Gọi sớm nhất có thể.
  void begin();

  // duty -1..+1 (dương = bánh lăn tới sau khi áp dụng đảo chiều). Tự bật STBY.
  void setDuty(Side side, float duty);

  // Phanh ngắn mạch cả 2 motor (dừng nhanh, không tốn điện).
  void brake();

  // STBY = LOW: ngắt hoàn toàn, motor nhả trôi.
  void standby();

  float duty(Side side) const { return channels_[side].duty; }

 private:
  struct Channel {
    uint8_t in1;
    uint8_t in2;
    uint8_t pwm_pin;
    uint8_t ledc_channel;
    bool inverted;
    float duty;
  };

  void writePwm(const Channel& ch, uint32_t value);
  void setStandby(bool standby);

  Channel channels_[2] = {};
  uint32_t pwm_max_ = 0;
  bool standby_ = true;
};
