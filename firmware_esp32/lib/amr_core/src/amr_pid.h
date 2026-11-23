// =============================================================================
//  amr_pid.h — Bộ điều khiển PID rời rạc cho vận tốc bánh xe.
//
//   * Thành phần I lưu ở đơn vị đầu ra (đã nhân KI) -> đổi KI lúc chạy không giật.
//   * Anti-windup kép: (1) kẹp |I| <= integral_limit, (2) tích phân có điều kiện:
//     khi sai số còn đẩy đầu ra vào bão hoà, I chỉ tăng tới mức đầu ra vừa chạm
//     giới hạn (motor nhận đủ 100 % duty) rồi dừng -> không windup, không hụt lực.
//   * Thành phần D tính trên GIÁ TRỊ ĐO (không giật khi đổi setpoint) và đi qua
//     bộ lọc thông thấp bậc 1 (hằng số thời gian d_filter_tau_s).
// =============================================================================
#pragma once

namespace amr {

struct PidGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

struct PidLimits {
  float output = 1.0f;    // đầu ra bị kẹp trong [-output, +output]
  float integral = 1.0f;  // thành phần I bị kẹp trong [-integral, +integral]
};

class PidController {
 public:
  PidController() = default;
  PidController(const PidGains& gains, const PidLimits& limits, float d_filter_tau_s);

  void setGains(const PidGains& gains) { gains_ = gains; }
  void setLimits(const PidLimits& limits);
  const PidGains& gains() const { return gains_; }

  // Xoá trạng thái (thành phần I, bộ lọc D, giá trị đo trước đó).
  void reset();

  // Một bước điều khiển. dt tính bằng giây. Trả về đầu ra đã kẹp.
  // Đầu vào không hợp lệ (dt <= 0, NaN/Inf) -> giữ nguyên đầu ra trước đó.
  float update(float setpoint, float measurement, float dt);

  float output() const { return output_; }
  float integralTerm() const { return i_term_; }
  bool saturated() const { return saturated_; }  // đầu ra đang ở (hoặc vượt) giới hạn

 private:
  PidGains gains_{};
  PidLimits limits_{};
  float d_tau_ = 0.0f;

  float i_term_ = 0.0f;
  float d_filtered_ = 0.0f;
  float prev_measurement_ = 0.0f;
  float output_ = 0.0f;
  bool has_prev_ = false;
  bool saturated_ = false;
};

}  // namespace amr
