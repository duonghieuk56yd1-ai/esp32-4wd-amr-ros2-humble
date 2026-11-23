#include "amr_diff_drive.h"

#include <cmath>

namespace amr {

WheelPair DiffDriveKinematics::toWheelSpeeds(float linear, float angular) const {
  const float half_l = 0.5f * l_;
  WheelPair w;
  w.left = (linear - angular * half_l) / r_;
  w.right = (linear + angular * half_l) / r_;
  return w;
}

BodyTwist DiffDriveKinematics::toBodyTwist(const WheelPair& wheel_rad_s) const {
  BodyTwist t;
  t.linear = 0.5f * r_ * (wheel_rad_s.right + wheel_rad_s.left);
  t.angular = r_ * (wheel_rad_s.right - wheel_rad_s.left) / l_;
  return t;
}

WheelPair limitWheelSpeeds(const WheelPair& wheels, float max_abs) {
  max_abs = std::fabs(max_abs);
  const float peak = std::fmax(std::fabs(wheels.left), std::fabs(wheels.right));
  if (!(peak > max_abs) || peak <= 0.0f) {
    return wheels;
  }
  const float scale = max_abs / peak;
  WheelPair out;
  out.left = wheels.left * scale;
  out.right = wheels.right * scale;
  return out;
}

double normalizeAngle(double angle) {
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle <= 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

void Odometry2D::reset(double x, double y, double yaw) {
  x_ = x;
  y_ = y;
  yaw_ = normalizeAngle(yaw);
}

void Odometry2D::update(double d_left, double d_right, double wheel_separation) {
  const double ds = 0.5 * (d_right + d_left);
  const double dtheta = (d_right - d_left) / wheel_separation;

  if (std::fabs(dtheta) < 1e-6) {
    // Gần như đi thẳng: Runge-Kutta bậc 2 (hướng giữa bước)
    const double heading = yaw_ + 0.5 * dtheta;
    x_ += ds * std::cos(heading);
    y_ += ds * std::sin(heading);
  } else {
    // Cung tròn chính xác bán kính R = ds / dtheta
    const double radius = ds / dtheta;
    const double yaw_next = yaw_ + dtheta;
    x_ += radius * (std::sin(yaw_next) - std::sin(yaw_));
    y_ -= radius * (std::cos(yaw_next) - std::cos(yaw_));
  }
  yaw_ = normalizeAngle(yaw_ + dtheta);
}

}  // namespace amr
