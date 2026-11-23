// =============================================================================
//  amr_diff_drive.h — Động học robot vi sai 2 bánh và tích phân odometry 2D.
//
//  Quy ước (REP-103): x tiến về phía trước, y sang trái, yaw dương ngược chiều
//  kim đồng hồ. Vận tốc bánh (rad/s) dương = bánh lăn về phía trước.
//
//    v_trái  = (v - w*L/2) / r        v = r*(w_phải + w_trái)/2
//    v_phải  = (v + w*L/2) / r        w = r*(w_phải - w_trái)/L
// =============================================================================
#pragma once

namespace amr {

struct WheelPair {
  float left = 0.0f;
  float right = 0.0f;
};

struct BodyTwist {
  float linear = 0.0f;   // m/s
  float angular = 0.0f;  // rad/s
};

class DiffDriveKinematics {
 public:
  DiffDriveKinematics(float wheel_radius_m, float wheel_separation_m)
      : r_(wheel_radius_m), l_(wheel_separation_m) {}

  // (v, w) thân xe -> vận tốc góc 2 bánh (rad/s)
  WheelPair toWheelSpeeds(float linear, float angular) const;
  // vận tốc góc 2 bánh (rad/s) -> (v, w) thân xe
  BodyTwist toBodyTwist(const WheelPair& wheel_rad_s) const;

  float wheelRadius() const { return r_; }
  float wheelSeparation() const { return l_; }

 private:
  float r_;
  float l_;
};

// Co cả 2 bánh theo CÙNG một tỉ lệ để không bánh nào vượt |max_abs|
// (giữ nguyên bán kính cong của quỹ đạo).
WheelPair limitWheelSpeeds(const WheelPair& wheels, float max_abs);

// Đưa góc về khoảng (-pi, pi].
double normalizeAngle(double angle);

class Odometry2D {
 public:
  void reset(double x = 0.0, double y = 0.0, double yaw = 0.0);

  // d_left / d_right: quãng đường mỗi bánh đã lăn kể từ lần gọi trước (m).
  // Tích phân cung tròn chính xác; khi gần đi thẳng dùng Runge-Kutta bậc 2.
  void update(double d_left, double d_right, double wheel_separation);

  double x() const { return x_; }
  double y() const { return y_; }
  double yaw() const { return yaw_; }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
};

}  // namespace amr
