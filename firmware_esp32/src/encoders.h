// =============================================================================
//  encoders.h — Đọc encoder quadrature bằng ngắt ngoài GPIO (CHANGE trên cả
//  kênh A và B, giải mã x4 bằng bảng trạng thái -> lọc được nảy cạnh).
// =============================================================================
#pragma once

#include <stdint.h>

class QuadratureEncoder {
 public:
  // Cấu hình chân + gắn ngắt. Gọi từ task chạy trên core muốn xử lý ngắt.
  void begin(uint8_t pin_a, uint8_t pin_b, bool inverted);

  // Số tick tích luỹ (đã áp dụng đảo chiều). Tràn sau 2^31 tick — dùng hiệu
  // số không dấu khi tính delta.
  int32_t count() const;

  // Số lần nhảy 2 trạng thái (mất cạnh) — nên luôn bằng 0.
  uint32_t errorCount() const { return errors_; }

 private:
  static void isr(void* arg);

  volatile int32_t count_ = 0;
  volatile uint32_t errors_ = 0;
  volatile uint8_t state_ = 0;
  uint8_t pin_a_ = 0;
  uint8_t pin_b_ = 0;
  bool inverted_ = false;
};
