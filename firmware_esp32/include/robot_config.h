// =============================================================================
//  robot_config.h — Thông số robot 2WD vi sai (2 bánh chủ động sau + 1 bánh
//  mắt trâu trước): cơ khí, PID, failsafe, ROS 2 và LiDAR.
// =============================================================================
#pragma once

// ===========================================================================
//  1. CHỌN LIDAR — chỉ để lại ĐÚNG 1 dòng #define, comment dòng còn lại
// ===========================================================================
#define LIDAR_YDLIDAR_X2        // YDLIDAR X2 / X2L / S2  (UART 115200, ~7 Hz)
// #define LIDAR_CAMSENSE_X1    // Camsense X1 / X2       (UART 115200, ~5.2 Hz)

// ===========================================================================
//  2. CƠ KHÍ & ENCODER
// ===========================================================================
#define WHEEL_RADIUS_M          0.0325f   // bán kính bánh chủ động (m)
#define WHEEL_SEPARATION_M      0.18f     // khoảng cách tâm 2 bánh chủ động (m)

// Số xung/vòng của MỘT kênh encoder, đo tại trục bánh xe
// (JGA25-370: 11 xung/vòng motor x hộp số 1:34 = 374).
#define ENCODER_PPR             374
// Firmware giải mã x4 (ngắt CHANGE trên cả kênh A và B): 374 x 4 = 1496 tick/vòng.
#define ENCODER_TICKS_PER_REV   (ENCODER_PPR * 4)

// Đảo chiều (0/1). Quy ước sau khi đảo: gửi lệnh tiến thì CẢ HAI bánh lăn về
// phía trước và encoder của cả hai đếm DƯƠNG (/joint_states position tăng).
// Hai motor lắp đối xứng gương nên thường phải đảo một bên.
#define MOTOR_LEFT_INVERTED     0
#define MOTOR_RIGHT_INVERTED    1
#define ENCODER_LEFT_INVERTED   0
#define ENCODER_RIGHT_INVERTED  1

// ===========================================================================
//  3. GIỚI HẠN VẬN TỐC
// ===========================================================================
// Tốc độ bánh tối đa khi có tải (rad/s). JGA25-370 1:34 @12V không tải ~190 rpm
// (~20 rad/s); để dư địa cho PID nên đặt ~75–80 % giá trị đo được.
#define MAX_WHEEL_SPEED_RAD_S   15.0f
#define MAX_LINEAR_VEL_MPS      0.40f     // kẹp /cmd_vel linear.x (m/s)
#define MAX_ANGULAR_VEL_RAD_S   3.0f      // kẹp /cmd_vel angular.z (rad/s)

// ===========================================================================
//  4. PID VẬN TỐC BÁNH (mỗi bánh một bộ, chạy ở CONTROL_LOOP_HZ)
//     Sai số tính bằng rad/s, đầu ra là duty PWM chuẩn hoá -1.0 … +1.0.
//     Kinh nghiệm chỉnh: tăng KP tới khi bám nhanh nhưng chưa rung, rồi tăng KI
//     để triệt sai số xác lập; KD thường để 0 vì vận tốc đo từ encoder nhiễu.
// ===========================================================================
#define PID_KP                  0.05f
#define PID_KI                  0.60f
#define PID_KD                  0.0f
#define PID_INTEGRAL_LIMIT      0.80f     // anti-windup: |thành phần I| <= giá trị này (duty)
#define PID_D_FILTER_TAU_S      0.02f     // hằng số thời gian lọc thông thấp thành phần D
#define MOTOR_MAX_DUTY          1.0f      // giảm (vd 0.6) nếu điện áp nguồn > định mức motor
#define WHEEL_VEL_FILTER_ALPHA  0.5f      // lọc EMA vận tốc đo, 1.0 = không lọc

// ===========================================================================
//  5. VÒNG ĐIỀU KHIỂN & FAILSAFE
// ===========================================================================
#define CONTROL_LOOP_HZ         50        // PID + odometry (task Core 1)
#define ODOM_PUBLISH_HZ         30        // /odom, /joint_states, /tf

// Mất kết nối Wi-Fi / micro-ROS agent quá thời gian này -> NGẮT động cơ ngay,
// chỉ chạy lại khi có kết nối và nhận được /cmd_vel MỚI.
#define FAILSAFE_TIMEOUT_MS     500
// Không nhận /cmd_vel mới quá thời gian này -> vận tốc đặt về 0 (dừng mềm).
// Giống cmd_vel_timeout của diff_drive_controller. Đặt 0 để tắt.
#define CMD_VEL_TIMEOUT_MS      500
// Khi failsafe: 1 = phanh ngắn mạch (dừng nhanh), 0 = nhả trôi (standby).
#define FAILSAFE_BRAKE          1
// Bảo vệ động cơ: ngắt khi duty bão hoà mà bánh quay NGƯỢC chiều (sai chiều
// motor/encoder) hoặc gần như KHÔNG quay (kẹt bánh / tuột dây encoder).
// Lỗi được xoá khi nhận /cmd_vel = 0.
#define DRIVE_FAULT_DETECTION   1

// ===========================================================================
//  6. ROS 2 / micro-ROS
// ===========================================================================
#define ROS_NODE_NAME           "esp32_base"
#define ROS_NAMESPACE           ""
#define ROS_DOMAIN_ID           0         // phải trùng ROS_DOMAIN_ID trên Laptop

#define TOPIC_CMD_VEL           "cmd_vel"
#define TOPIC_ODOM              "odom"
#define TOPIC_JOINT_STATES      "joint_states"
#define TOPIC_TF                "tf"

#define ODOM_FRAME_ID           "odom"
#define BASE_FRAME_ID           "base_footprint"
#define LEFT_WHEEL_JOINT_NAME   "left_wheel_joint"   // phải trùng tên joint trong URDF
#define RIGHT_WHEEL_JOINT_NAME  "right_wheel_joint"

// 1 = firmware broadcast TF odom -> base_footprint (30 Hz).
// Đặt 0 nếu dùng robot_localization (EKF) để phát TF này.
#define PUBLISH_ODOM_TF         1

// Phương sai (đường chéo ma trận covariance) gửi kèm /odom.
#define ODOM_COV_POSE_XY        1e-3
#define ODOM_COV_POSE_YAW       1e-2
#define ODOM_COV_TWIST_VX       1e-3      // dùng cho cả vy (robot vi sai: vy ~ 0)
#define ODOM_COV_TWIST_WZ       1e-2
#define ODOM_COV_UNUSED         1e3       // z, roll, pitch, vz, wx, wy (không đo)

// Đồng bộ đồng hồ với agent để đóng dấu thời gian ROS cho /odom, /tf và LiDAR.
#define TIME_SYNC_PERIOD_MS     10000

// ===========================================================================
//  7. LIDAR -> UDP (Laptop lắng nghe cổng này, IP = AGENT_IP trong secrets.h)
// ===========================================================================
#define LIDAR_UDP_PORT          8889
#define LIDAR_SCAN_BINS         360       // số tia/vòng (360 = 1°/tia). Tối đa 476
#define LIDAR_SEND_INTENSITY    1         // 1 = gửi kèm cường độ (uint8) mỗi tia
#define LIDAR_YAW_OFFSET_DEG    0.0f      // bù góc lắp LiDAR (độ, ngược chiều kim đồng hồ là dương)

#if defined(LIDAR_YDLIDAR_X2)
#define LIDAR_RANGE_MIN_M       0.10f
#define LIDAR_RANGE_MAX_M       8.0f
// Góc gốc của LiDAR tăng theo chiều kim đồng hồ (nhìn từ trên xuống) -> firmware
// đổi sang ngược chiều kim đồng hồ theo chuẩn ROS (REP-103).
// Nếu scan trong RViz bị lật gương, đổi giá trị này.
#define LIDAR_ANGLE_CLOCKWISE   1
#elif defined(LIDAR_CAMSENSE_X1)
#define LIDAR_RANGE_MIN_M       0.08f
#define LIDAR_RANGE_MAX_M       8.0f
#define LIDAR_ANGLE_CLOCKWISE   1
#endif

// ===========================================================================
//  8. FreeRTOS & gỡ lỗi (nâng cao)
// ===========================================================================
#define CONTROL_TASK_CORE       1         // PID, odometry, failsafe
#define CONTROL_TASK_PRIORITY   10
#define MICROROS_TASK_CORE      1         // micro-ROS: /cmd_vel, /odom, /joint_states, /tf
#define MICROROS_TASK_PRIORITY  5
#define LIDAR_TASK_CORE         0         // LiDAR -> UDP (cùng core với Wi-Fi stack)
#define LIDAR_TASK_PRIORITY     4

#define WIFI_HOSTNAME           "esp32-amr"
#define SERIAL_BAUD             115200
#define STATUS_PRINT_PERIOD_MS  1000      // in trạng thái ra Serial, 0 = tắt
