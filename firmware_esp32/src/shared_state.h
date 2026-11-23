// =============================================================================
//  shared_state.h — Dữ liệu trao đổi giữa các task FreeRTOS.
//
//   * control task (Core 1)  : ghi RobotStateSample, đọc CmdVelSample + link watchdog
//   * micro-ROS task (Core 1): ghi CmdVelSample, link watchdog, độ lệch ROS time;
//                              đọc RobotStateSample để publish
//   * LiDAR task (Core 0)    : đọc độ lệch ROS time, ghi thống kê LiDAR
//
//  Mọi truy cập đi qua spinlock (portMUX) và chỉ sao chép struct nhỏ -> vùng
//  găng rất ngắn, an toàn giữa 2 core.
// =============================================================================
#pragma once

#include <stdint.h>

enum class DriveState : uint8_t {
  kIdle = 0,  // đứng yên / chờ lệnh
  kRunning,   // PID đang bám vận tốc đặt
  kFailsafe,  // mất kết nối Wi-Fi/agent -> motor bị ngắt
  kFault,     // lỗi động cơ/encoder -> motor bị ngắt
};

const char* driveStateName(DriveState state);

struct CmdVelSample {
  float linear = 0.0f;   // m/s
  float angular = 0.0f;  // rad/s
  uint32_t rx_ms = 0;    // millis() lúc nhận
  uint32_t seq = 0;      // tăng mỗi lần nhận; 0 = chưa từng nhận
};

struct RobotStateSample {
  int64_t stamp_us = 0;  // esp_timer_get_time() lúc lấy mẫu encoder
  uint32_t seq = 0;
  double x = 0.0, y = 0.0, yaw = 0.0;  // odometry (m, rad)
  float linear = 0.0f, angular = 0.0f; // vận tốc thân xe đo được
  double wheel_pos_rad[2] = {0.0, 0.0};       // [trái, phải] góc quay tích luỹ
  float wheel_vel_rad_s[2] = {0.0f, 0.0f};    // vận tốc bánh đã lọc
  float wheel_target_rad_s[2] = {0.0f, 0.0f}; // vận tốc đặt cho PID
  float duty[2] = {0.0f, 0.0f};               // duty PWM -1..1
  int32_t ticks[2] = {0, 0};
  uint32_t encoder_errors[2] = {0, 0};
  DriveState state = DriveState::kIdle;
};

struct LidarStats {
  bool receiving = false;    // đang nhận dữ liệu từ LiDAR
  float scan_hz = 0.0f;      // tần số quét đo được
  uint16_t last_valid = 0;   // số tia có phản hồi ở vòng gần nhất
  uint32_t scans_sent = 0;
  uint32_t send_errors = 0;
  uint32_t decode_errors = 0;
};

namespace shared {

void init();

// /cmd_vel mới nhất
void setCmdVel(float linear, float angular);
CmdVelSample cmdVel();

// Trạng thái robot (odometry, bánh xe, PID)
void setRobotState(const RobotStateSample& state);
RobotStateSample robotState();

// Link watchdog: "bằng chứng" Laptop còn liên lạc được = MỌI gói UDP nhận từ
// micro-ROS agent (ACK của gói reliable, trả lời ping/đồng bộ giờ, /cmd_vel).
void markLinkAlive();
void markLinkDown();             // chắc chắn mất (Wi-Fi rớt, mất agent)
uint32_t msSinceLinkAlive();     // UINT32_MAX nếu đang down / chưa từng lên

void setAgentConnected(bool connected);
bool agentConnected();

// Ánh xạ đồng hồ ESP32 -> ROS time: ros_ns = esp_us * 1000 + offset_ns
void setRosTimeOffsetNs(int64_t offset_ns);
bool rosTimeValid();
// Chưa đồng bộ: trả về uptime (ns). `synced` (nếu khác nullptr) cho biết kết quả
// có phải ROS time hay không — đọc cùng lúc với offset.
int64_t toRosTimeNs(int64_t esp_time_us, bool* synced = nullptr);

void setLidarStats(const LidarStats& stats);
LidarStats lidarStats();

}  // namespace shared
