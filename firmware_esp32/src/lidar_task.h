// =============================================================================
//  lidar_task.h — Cầu nối LiDAR -> Wi-Fi (Core 0).
//  Đọc UART (Serial2) bằng thư viện kaiaai/LDS, gom đủ 1 vòng 360° thành
//  LIDAR_SCAN_BINS tia rồi gửi 1 gói UDP nhị phân (xem amr_scan_packet.h) tới
//  Laptop cổng LIDAR_UDP_PORT.
// =============================================================================
#pragma once

#include <IPAddress.h>

namespace lidar {

void start(const IPAddress& dest_ip, uint16_t dest_port);

unsigned stackHighWaterMark();

}  // namespace lidar
