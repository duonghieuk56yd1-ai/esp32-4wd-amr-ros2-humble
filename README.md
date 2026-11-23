# esp32-4wd-amr-ros2-humble

Robot tự hành vi sai (2 bánh chủ động + 1 bánh mắt trâu, LiDAR) điều khiển bằng ESP32 và ROS 2 Humble,
kết nối không dây hoàn toàn qua Wi-Fi UDP với Laptop.

| Thư mục | Nội dung |
|---|---|
| [`firmware_esp32/`](firmware_esp32/) | Firmware PlatformIO + micro-ROS (Humble) cho ESP32-WROOM-32: PID 2 bánh, odometry, `/odom`, `/joint_states`, `/tf`, cầu nối LiDAR UDP |
| [`ros2_ws/`](ros2_ws/) | Workspace ROS 2 Humble trên Laptop: `amr_description` (URDF/Xacro + RViz2), `amr_lidar_bridge` (UDP 8889 → `/scan`), `amr_bringup` (micro-ROS agent, slam_toolbox, Nav2) |
