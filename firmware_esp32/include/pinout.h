// =============================================================================
//  pinout.h — Sơ đồ chân ESP32-WROOM-32 (board esp32dev / DevKit V1)
// -----------------------------------------------------------------------------
//  Nguyên tắc chọn chân:
//   * Không dùng GPIO 6–11 (flash SPI), 1/3 (UART0 nạp code/USB), 0 và 12
//     (strapping: chế độ boot / điện áp flash).
//   * Encoder chỉ dùng chân có pull-up nội và KHÔNG dùng GPIO 36/39 (lỗi phần
//     cứng ESP32: ngắt giả khi Wi-Fi/ADC hoạt động).
//   * GPIO 21/22 để dành cho I2C (ví dụ IMU sau này).
//   * Toàn bộ 7 chân TB6612FNG nằm cùng một hàng header (32,33,25,26,27,14,13).
//
//  Cấp nguồn encoder bằng 3.3 V (ESP32 KHÔNG chịu được tín hiệu 5 V).
//  Nối chung GND giữa ESP32, TB6612FNG, nguồn motor và LiDAR.
// =============================================================================
#pragma once

// ---------------------------------------------------------------------------
//  Driver động cơ TB6612FNG (kênh A = bánh TRÁI, kênh B = bánh PHẢI)
// ---------------------------------------------------------------------------
#define PIN_MOTOR_STBY      13   // STBY: LOW = standby (cả 2 motor nhả trôi)

#define PIN_MOTOR_L_IN1     32   // AIN1
#define PIN_MOTOR_L_IN2     33   // AIN2
#define PIN_MOTOR_L_PWM     25   // PWMA

#define PIN_MOTOR_R_IN1     26   // BIN1
#define PIN_MOTOR_R_IN2     27   // BIN2
#define PIN_MOTOR_R_PWM     14   // PWMB

// PWM phần cứng LEDC: 20 kHz (ngoài ngưỡng nghe, TB6612 hỗ trợ tới 100 kHz)
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_RES_BITS  10   // 0..1023
#define MOTOR_PWM_CH_LEFT   0    // kênh 0,1 dùng chung timer 0
#define MOTOR_PWM_CH_RIGHT  1

// ---------------------------------------------------------------------------
//  Encoder quadrature (ngắt ngoài CHANGE trên cả 2 kênh A/B -> giải mã x4)
// ---------------------------------------------------------------------------
#define PIN_ENC_L_A         18
#define PIN_ENC_L_B         19
#define PIN_ENC_R_A         23
#define PIN_ENC_R_B         4

// ---------------------------------------------------------------------------
//  LiDAR trên Serial2 (UART2)
// ---------------------------------------------------------------------------
#define LIDAR_SERIAL        Serial2
#define PIN_LIDAR_RX        16   // RX2 của ESP32  <- chân TX của LiDAR
#define PIN_LIDAR_TX        17   // TX2 của ESP32  -> chân RX của LiDAR (nếu có)

// Chân điều khiển motor quét của LiDAR (tuỳ chọn, -1 = không nối):
//  * YDLIDAR X2/X2L: chân M_SCTR. Thư viện LDS để chân này ở chế độ INPUT
//    (thả nổi) khi chạy -> LiDAR quay tốc độ mặc định; không nối cũng được.
//  * Camsense X1: không dùng.
#define PIN_LIDAR_MOTOR_PWM 15
#define PIN_LIDAR_MOTOR_EN  -1
#define LIDAR_PWM_CH        2    // kênh 2 -> timer 1 (khác timer của motor)
#define LIDAR_PWM_FREQ_HZ   10000
#define LIDAR_PWM_RES_BITS  11

// ---------------------------------------------------------------------------
//  LED trạng thái (LED xanh trên board DevKit), -1 = tắt
//    nháy nhanh : đang kết nối Wi-Fi
//    nháy chậm  : có Wi-Fi, đang chờ micro-ROS agent
//    sáng liên tục: đã kết nối agent
//    nháy kép   : failsafe / lỗi động cơ
// ---------------------------------------------------------------------------
#define PIN_STATUS_LED      2
