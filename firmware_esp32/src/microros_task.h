// =============================================================================
//  microros_task.h — Node micro-ROS qua Wi-Fi UDP (Core 1).
//
//   Subscribe : /cmd_vel       geometry_msgs/msg/Twist       (best effort)
//   Publish   : /odom          nav_msgs/msg/Odometry         (reliable, 30 Hz)
//               /joint_states  sensor_msgs/msg/JointState    (best effort, 30 Hz)
//               /tf            tf2_msgs/msg/TFMessage        (reliable, 30 Hz)
//
//  Tự kết nối lại khi mất Wi-Fi / agent, đồng bộ đồng hồ với agent để đóng dấu
//  thời gian ROS, và cấp "nhịp tim" cho failsafe của control task.
// =============================================================================
#pragma once

#include <IPAddress.h>

namespace microros {

void start(const IPAddress& agent_ip, uint16_t agent_port);

unsigned stackHighWaterMark();

}  // namespace microros
