// =============================================================================
//  secrets.h — Thông tin Wi-Fi và địa chỉ micro-ROS agent (Laptop)
// -----------------------------------------------------------------------------
//  Cách 1: sửa trực tiếp 4 giá trị bên dưới.
//  Cách 2 (khuyến nghị, tránh lộ mật khẩu khi push lên Git): tạo file
//  include/secrets_local.h (đã có trong .gitignore) với nội dung ví dụ:
//
//      #define WIFI_SSID     "TenWiFiThat"
//      #define WIFI_PASSWORD "MatKhauThat"
//      #define AGENT_IP      "192.168.1.50"
//
//  Nếu secrets_local.h tồn tại, các macro trong đó được ưu tiên.
// =============================================================================
#pragma once

#if __has_include("secrets_local.h")
#include "secrets_local.h"
#endif

// Tên và mật khẩu mạng Wi-Fi 2.4 GHz (ESP32 không hỗ trợ 5 GHz).
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

// IPv4 của Laptop chạy micro_ros_agent (xem bằng `ip -4 addr` hoặc `ipconfig`).
// Gói tin LiDAR (UDP LIDAR_UDP_PORT trong robot_config.h) cũng gửi về IP này.
#ifndef AGENT_IP
#define AGENT_IP "192.168.1.100"
#endif

// Cổng UDP của micro-ROS agent:
//   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
#ifndef AGENT_PORT
#define AGENT_PORT 8888
#endif
