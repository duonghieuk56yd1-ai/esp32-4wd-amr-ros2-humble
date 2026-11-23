#include "encoders.h"

#include <Arduino.h>
#include <soc/gpio_reg.h>
#include <soc/soc.h>

#include "amr_quadrature.h"

namespace {

// Đọc mức chân trực tiếp từ thanh ghi (nhanh, an toàn trong ISR).
static inline __attribute__((always_inline)) uint32_t readPinFast(uint8_t pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1u
                  : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u;
}

}  // namespace

void IRAM_ATTR QuadratureEncoder::isr(void* arg) {
  QuadratureEncoder* self = static_cast<QuadratureEncoder*>(arg);
  const uint8_t curr =
      static_cast<uint8_t>((readPinFast(self->pin_a_) << 1) | readPinFast(self->pin_b_));
  const uint8_t prev = self->state_;
  // Tra bảng nén 32 bit (hằng số nằm trong lệnh, không đọc flash)
  const uint32_t code = (amr::quad::kLut >> ((((prev & 3u) << 2) | curr) << 1)) & 3u;
  self->state_ = curr;
  if (code == amr::quad::kForward) {
    self->count_ = self->count_ + 1;
  } else if (code == amr::quad::kReverse) {
    self->count_ = self->count_ - 1;
  } else if (code == amr::quad::kInvalid) {
    self->errors_ = self->errors_ + 1;
  }
}

void QuadratureEncoder::begin(uint8_t pin_a, uint8_t pin_b, bool inverted) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  inverted_ = inverted;

  // GPIO 34–39 không có pull-up nội: cần điện trở kéo lên ngoài nếu dùng.
  pinMode(pin_a, pin_a >= 34 ? INPUT : INPUT_PULLUP);
  pinMode(pin_b, pin_b >= 34 ? INPUT : INPUT_PULLUP);
  delayMicroseconds(50);

  count_ = 0;
  errors_ = 0;
  state_ = static_cast<uint8_t>((readPinFast(pin_a) << 1) | readPinFast(pin_b));

  attachInterruptArg(digitalPinToInterrupt(pin_a), isr, this, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(pin_b), isr, this, CHANGE);
}

int32_t QuadratureEncoder::count() const {
  const int32_t c = count_;  // đọc 32 bit căn lề là nguyên tử trên Xtensa
  return inverted_ ? -c : c;
}
