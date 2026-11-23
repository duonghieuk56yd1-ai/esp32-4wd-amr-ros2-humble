# firmware_esp32 — Robot 2WD vi sai + LiDAR, micro-ROS (ROS 2 Humble) qua Wi-Fi

Firmware PlatformIO cho **ESP32-WROOM-32** (`esp32dev`, framework Arduino) điều khiển robot tự hành
3 bánh: 2 bánh chủ động phía sau có encoder (driver **TB6612FNG**) + 1 bánh mắt trâu phía trước.
Robot liên lạc với Laptop **hoàn toàn không dây qua Wi-Fi UDP**:

| Kênh | Cổng UDP | Nội dung |
|---|---|---|
| micro-ROS agent | `8888` | `/cmd_vel`, `/odom`, `/joint_states`, `/tf` |
| Cầu nối LiDAR | `8889` | 1 gói nhị phân / 1 vòng quét 360° (YDLIDAR X2 hoặc Camsense X1) |

## Kiến trúc

```
                       ESP32-WROOM-32 (2 core)
 ┌────────────────────────── Core 1 ──────────────────────────┐ ┌──────── Core 0 ────────┐
 │ ISR encoder (CHANGE A/B, giải mã x4)                        │ │ Wi-Fi + lwIP stack      │
 │        │ tick                                               │ │                         │
 │ control task (50 Hz, ưu tiên 10)                            │ │ LiDAR task (ưu tiên 4)  │
 │   encoder -> rad, rad/s -> PID (anti-windup) -> PWM TB6612  │ │  Serial2 -> kaiaai/LDS  │
 │   odometry vi sai (x, y, yaw) · failsafe · bảo vệ động cơ   │ │  gom 360 tia (1°/tia)   │
 │        ▲ cmd_vel          │ trạng thái robot                │ │  -> UDP 8889 (1124 B)   │
 │ micro-ROS task (ưu tiên 5)▼                                 │ └─────────────────────────┘
 │   sub /cmd_vel · pub /odom /joint_states /tf (30 Hz)        │
 │   ping agent · đồng bộ đồng hồ · tự kết nối lại             │
 │ loopTask: LED trạng thái, in chẩn đoán qua USB Serial       │
 └─────────────────────────────────────────────────────────────┘
```

* Vòng điều khiển **không bao giờ chờ mạng**: micro-ROS chạy ở task riêng (ưu tiên thấp hơn), hai
  bên trao đổi dữ liệu qua `shared_state` (spinlock, chỉ sao chép struct nhỏ).
* Dấu thời gian của `/odom`, `/tf` và gói LiDAR cùng lấy từ đồng hồ ESP32 đã đồng bộ với agent
  (`rmw_uros_sync_session`) → scan và odometry nhất quán thời gian cho SLAM.

## Cấu trúc thư mục

```
firmware_esp32/
├── platformio.ini            # esp32dev + micro_ros_platformio (humble, wifi) + kaiaai/LDS; env native để test
├── include/
│   ├── secrets.h             # WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT (8888)
│   ├── pinout.h              # chân TB6612FNG, encoder, Serial2 LiDAR, LED
│   └── robot_config.h        # cơ khí, PID, failsafe, ROS, chọn LiDAR (1 dòng)
├── src/
│   ├── main.cpp              # khởi tạo an toàn, Wi-Fi, tạo task, LED, log trạng thái
│   ├── control_task.*        # Core 1: PID + odometry + failsafe
│   ├── microros_task.*       # Core 1: node micro-ROS
│   ├── lidar_task.*          # Core 0: LiDAR -> UDP
│   ├── encoders.*            # ngắt ngoài quadrature x4
│   ├── motor_driver.*        # TB6612FNG + LEDC PWM 20 kHz
│   └── shared_state.*        # dữ liệu dùng chung giữa các task
├── lib/amr_core/             # logic thuần (test được trên PC): PID, động học, encoder, gói LiDAR
├── test/test_amr_core/       # unit test Unity: pio test -e native
├── extra_packages/tf2_msgs/  # gói tf2_msgs (ROS 2 Humble) để micro-ROS build kèm -> publish /tf
└── tools/lidar_udp_monitor.py# công cụ Laptop: nhận & giải mã gói LiDAR (không cần ROS)
```

## Đấu nối phần cứng

Chân được chọn để tránh strapping pin quan trọng (0, 12), chân flash (6–11), USB-UART (1, 3) và
GPIO 36/39 (lỗi phần cứng gây ngắt giả khi bật Wi-Fi); GPIO 21/22 để dành cho I2C (IMU sau này).
Sửa trong `include/pinout.h` nếu đấu khác.

### TB6612FNG

| TB6612FNG | ESP32 | Ghi chú |
|---|---|---|
| PWMA | GPIO 25 | motor **trái** (kênh A) |
| AIN1 / AIN2 | GPIO 32 / GPIO 33 | |
| PWMB | GPIO 14 | motor **phải** (kênh B) |
| BIN1 / BIN2 | GPIO 26 / GPIO 27 | |
| STBY | GPIO 13 | LOW = standby |
| VCC | 3.3 V | logic |
| VM | nguồn motor (vd 7.4–12 V) | thêm tụ 100–470 µF sát chân VM |
| AO1/AO2, BO1/BO2 | motor trái, motor phải | |
| GND | GND chung | |

### Encoder (JGA25-370: 11 xung × hộp số 1:34 = 374 xung/vòng)

| Encoder | ESP32 |
|---|---|
| Trái kênh A / B | GPIO 18 / GPIO 19 |
| Phải kênh A / B | GPIO 23 / GPIO 4 |
| VCC | **3.3 V** (ESP32 không chịu tín hiệu 5 V) |
| GND | GND |

Firmware bật pull-up nội và giải mã **x4** (ngắt `CHANGE` trên cả A và B) → `374 × 4 = 1496`
tick/vòng ≈ 0.14 mm quãng đường bánh mỗi tick.

### LiDAR (Serial2)

| LiDAR | ESP32 | Ghi chú |
|---|---|---|
| TX | GPIO 16 (RX2) | 115200 baud |
| RX (nếu có) | GPIO 17 (TX2) | |
| M_SCTR (YDLIDAR X2) | GPIO 15 | tuỳ chọn; không nối thì LiDAR quay tốc độ mặc định |
| VCC | 5 V | dòng khởi động motor LiDAR khá lớn: nên cấp từ nguồn 5 V riêng (buck), không lấy từ chân 5V của DevKit khi cắm USB |
| GND | GND | |

LED trạng thái: GPIO 2 (LED trên board DevKit).

## Cấu hình

1. **Wi-Fi & agent** — `include/secrets.h`. Khuyến nghị tạo `include/secrets_local.h` (đã bị
   `.gitignore`) để không lộ mật khẩu khi push:

   ```c
   #define WIFI_SSID     "TenWiFi"        // Wi-Fi 2.4 GHz
   #define WIFI_PASSWORD "MatKhau"
   #define AGENT_IP      "192.168.1.50"   // IP Laptop chạy micro_ros_agent
   ```

   Nên đặt IP tĩnh / DHCP reservation cho Laptop vì firmware gửi cả micro-ROS lẫn LiDAR tới IP này.

2. **Chọn LiDAR** — `include/robot_config.h`, chỉ để lại đúng một dòng:

   ```c
   #define LIDAR_YDLIDAR_X2        // YDLIDAR X2 / X2L / S2
   // #define LIDAR_CAMSENSE_X1    // Camsense X1 / X2
   ```

3. **Thông số robot** — cũng trong `robot_config.h`: `WHEEL_RADIUS_M 0.0325`,
   `WHEEL_SEPARATION_M 0.18`, `ENCODER_PPR 374`, `PID_KP/KI/KD`, giới hạn vận tốc, thời gian
   failsafe, tên frame/joint (phải khớp URDF), góc lắp LiDAR, số tia/vòng…

## Build & nạp

Cần PlatformIO Core ≥ 6.1 (hoặc VS Code + PlatformIO IDE) và `git`, `cmake`, `python3-pip`
(micro_ros_platformio tự build thư viện micro-ROS ở lần build đầu, mất vài phút):

```bash
cd firmware_esp32
```

```bash
pio run
```

```bash
pio run -t upload
```

```bash
pio device monitor
```

* Đổi `board_microros_distro`, colcon meta hoặc `extra_packages/` thì phải build lại micro-ROS:
  `pio run -t clean_microros`.
* Kích thước hiện tại: Flash ~856 KB (65 % phân vùng app 1.25 MB), RAM tĩnh ~74 KB.

## Chạy trên Laptop (ROS 2 Humble)

Laptop và robot phải cùng mạng Wi-Fi. Mở firewall cho UDP 8888 và 8889
(Ubuntu: `sudo ufw allow 8888/udp` và `sudo ufw allow 8889/udp`).

micro-ROS agent (chọn một cách):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v4
```

Khi robot kết nối, Serial in `Da ket noi micro-ROS agent` và LED sáng liên tục. Kiểm tra:

```bash
ros2 topic list
```

```bash
ros2 topic hz /odom
```

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Kiểm tra luồng LiDAR (không cần ROS):

```bash
python3 tools/lidar_udp_monitor.py
```

> `ROS_DOMAIN_ID` trên Laptop phải trùng `ROS_DOMAIN_ID` trong `robot_config.h` (mặc định 0).
> Agent chạy trong WSL2 cần networking mode "mirrored" (hoặc port-forward UDP) thì ESP32 mới tới được.

### Topic

| Topic | Kiểu | QoS | Tần số |
|---|---|---|---|
| `/cmd_vel` (sub) | `geometry_msgs/msg/Twist` | best effort (nhận được từ cả publisher reliable) | — |
| `/odom` | `nav_msgs/msg/Odometry` (`odom` → `base_footprint`) | reliable | 30 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` (`left_wheel_joint`, `right_wheel_joint`: position rad, velocity rad/s) | best effort | 30 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` (`odom` → `base_footprint`) | reliable | 30 Hz |

`/odom` và `/tf` dùng reliable để tương thích subscriber mặc định của Nav2, robot_localization và
`tf2_ros::TransformListener`; `/joint_states` best effort khớp `robot_state_publisher`
(SensorDataQoS). Quy ước joint: giá trị **dương = bánh lăn về phía trước** (URDF trục `0 1 0`).
Dùng EKF (robot_localization) để phát TF `odom → base_footprint` thì đặt `PUBLISH_ODOM_TF 0`.

## Gói LiDAR UDP (cổng 8889)

Mỗi vòng quét là **một** datagram (little-endian), 360 tia + cường độ = **1124 byte**
(< 1472 byte nên không bị phân mảnh IP). Định nghĩa chuẩn ở
`lib/amr_core/src/amr_scan_packet.h`:

| Offset | Kiểu | Trường | Ghi chú |
|---|---|---|---|
| 0 | `char[4]` | magic | `"AMRL"` |
| 4 | `uint8` | version | `1` |
| 5 | `uint8` | lidar_model | 1 = YDLIDAR X2, 2 = Camsense X1 |
| 6 | `uint16` | flags | bit0: có intensities · bit1: `stamp_ns` là ROS time |
| 8 | `uint32` | seq | số thứ tự vòng quét (phát hiện mất gói) |
| 12 | `uint64` | stamp_ns | thời điểm bắt đầu vòng quét |
| 20 | `float32` | scan_time_s | thời gian 1 vòng |
| 24 | `float32` | angle_min_rad | `0` |
| 28 | `float32` | angle_increment_rad | `2π / N` |
| 32 | `float32` | range_min_m | |
| 36 | `float32` | range_max_m | |
| 40 | `uint16` | num_ranges | N (mặc định 360) |
| 42 | `uint16` | num_valid | số tia có phản hồi |
| 44 | `uint16[N]` | ranges_mm | `0` = không có phản hồi → `+inf` trong LaserScan |
| 44+2N | `uint8[N]` | intensities | chỉ khi flags bit0 = 1 |

* Tia `i` ở góc `angle_min + i·angle_increment` trong frame LiDAR theo **quy ước ROS** (x phía
  trước, góc dương ngược chiều kim đồng hồ). Firmware đã đổi chiều quay gốc của LiDAR
  (`LIDAR_ANGLE_CLOCKWISE`) và bù góc lắp (`LIDAR_YAW_OFFSET_DEG`), nên phía Laptop chỉ việc chép
  sang `sensor_msgs/LaserScan` (`angle_max = angle_min + (N-1)·angle_increment`, `time_increment = 0`).
* Nhiều điểm rơi vào cùng một tia → giữ điểm **gần nhất** (an toàn cho tránh vật cản).
* Python: `struct.unpack_from('<4sBBHIQfffffHH', buf)`; xem `tools/lidar_udp_monitor.py`.

## An toàn & failsafe

| Tình huống | Phản ứng |
|---|---|
| Khởi động | Driver về standby ngay đầu `setup()`, sau đó giữ phanh; motor chỉ chạy khi đã kết nối agent **và** nhận `/cmd_vel` đầu tiên |
| Mất Wi-Fi / agent > `FAILSAFE_TIMEOUT_MS` (500 ms) | **Ngắt động cơ ngay** (phanh ngắn mạch, hoặc standby nếu `FAILSAFE_BRAKE 0`), reset PID. Chỉ chạy lại khi có kết nối **và** một `/cmd_vel` mới (không chạy tiếp lệnh cũ) |
| Không có `/cmd_vel` mới > `CMD_VEL_TIMEOUT_MS` (500 ms) | Vận tốc đặt = 0 → PID hãm về 0 rồi nhả (giống `cmd_vel_timeout` của diff_drive_controller; đặt 0 để tắt) |
| Duty bão hoà mà bánh quay **ngược** chiều > 300 ms | `FAULT`: ngắt động cơ — gần như chắc chắn sai `MOTOR_*_INVERTED`/`ENCODER_*_INVERTED` |
| Duty bão hoà mà bánh **không quay** > 1 s | `FAULT`: kẹt bánh hoặc mất tín hiệu encoder |
| Xoá `FAULT` | Gửi `/cmd_vel` bằng 0 |

"Liên lạc tốt" được xác định bằng: ping agent thành công, `/odom` hoặc `/tf` (reliable) được agent
xác nhận, hoặc nhận `/cmd_vel`. Wi-Fi tắt modem-sleep để giảm trễ; firmware tự kết nối lại Wi-Fi và
tạo lại phiên micro-ROS (kể cả khi agent khởi động lại).

## LED trạng thái (GPIO 2)

| LED | Ý nghĩa |
|---|---|
| Nháy nhanh (5 Hz) | Đang kết nối Wi-Fi (hoặc `AGENT_IP` sai) |
| Nháy chậm (1 Hz) | Có Wi-Fi, đang chờ micro-ROS agent |
| Sáng liên tục | Đã kết nối agent |
| Nháy kép | `FAULT` động cơ |

## Chạy thử lần đầu (bring-up)

1. **Kê robot cho bánh không chạm đất.** Nạp firmware, mở `pio device monitor`. Mỗi giây có 2
   dòng `[STAT]`: trạng thái Wi-Fi/agent, `cmd`, vận tốc đo/đặt `wL`, `wR`, `duty`, `tick`,
   odometry, LiDAR, stack còn trống của các task.
2. **Chiều encoder:** xoay tay từng bánh về phía trước → `tick` của bánh đó phải **tăng**. Nếu
   giảm, đổi `ENCODER_LEFT_INVERTED` / `ENCODER_RIGHT_INVERTED`. `encErr` phải đứng yên ở 0.
3. **Chiều motor:** chạy agent rồi gửi lệnh tiến chậm (10 Hz để không dính `CMD_VEL_TIMEOUT_MS`):

   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```

   Cả hai bánh phải lăn tới. Bánh nào quay ngược thì đổi `MOTOR_*_INVERTED` của bánh đó (nếu
   motor và encoder lệch chiều nhau, firmware báo `FAULT ... quay NGUOC chieu lenh` và tự ngắt).
4. **Hiệu chỉnh odometry:** cho robot đi thẳng 1 m đo thực tế → chỉnh `WHEEL_RADIUS_M` cho
   `/odom` x = 1.0; quay tại chỗ 10 vòng → chỉnh `WHEEL_SEPARATION_M` cho yaw đúng.
5. **Chỉnh PID:** quan sát `wL=đo/đặt` trên Serial hoặc `ros2 topic echo /joint_states`. Tăng
   `PID_KP` tới khi bám nhanh nhưng chưa dao động, tăng `PID_KI` để hết sai số xác lập; thường
   `PID_KD = 0`. Đo tốc độ bánh tối đa thực tế (có tải) và đặt `MAX_WHEEL_SPEED_RAD_S` ~80 % giá trị đó.

## Unit test (không cần ESP32)

```bash
pio test -e native
```

17 test: bảng giải mã quadrature (kể cả nảy cạnh/mất cạnh), PID (kẹp, anti-windup, D trên giá trị
đo, vòng kín với mô hình motor), động học & odometry (đi thẳng, quay tại chỗ, cung tròn), gom tia
LiDAR và đóng gói/giải mã UDP.

## Xử lý sự cố

| Hiện tượng | Kiểm tra |
|---|---|
| LED nháy nhanh mãi | SSID/mật khẩu, Wi-Fi phải là 2.4 GHz, `AGENT_IP` đúng định dạng |
| LED nháy chậm mãi | Agent đã chạy `udp4 --port 8888`? Firewall Laptop? `AGENT_IP` đúng IP Laptop? |
| Có node nhưng không thấy topic | `ROS_DOMAIN_ID` hai bên, chạy `ros2 daemon stop` rồi thử lại |
| TF cảnh báo thời gian / extrapolation | Đồng hồ ESP32 đồng bộ với agent; kiểm tra Serial có `Dong bo thoi gian voi agent OK` |
| Robot chạy giật khi dùng teleop bàn phím | Lệnh chỉ gửi khi bấm phím; tăng/tắt `CMD_VEL_TIMEOUT_MS` hoặc giữ phím |
| `lidar=--` | Dây TX LiDAR → GPIO 16, nguồn 5 V đủ dòng, đúng macro model trong `robot_config.h` |
| Scan bị lật gương / lệch góc | Đổi `LIDAR_ANGLE_CLOCKWISE`, chỉnh `LIDAR_YAW_OFFSET_DEG` |

## Ghi chú kỹ thuật

* `platform = espressif32 @ 6.13.0` (Arduino-ESP32 2.0.17 / ESP-IDF 4.4) và
  `micro_ros_platformio` được ghim commit để build lặp lại được.
* `kaiaai/LDS` được ghim commit `6e17803` vì có bản sửa kiểm tra checksum gói Camsense X1 (chưa có
  trong bản 0.6.3 trên registry).
* `tf2_msgs` không nằm trong bản build mặc định của micro_ros_platformio nên được thêm qua
  `extra_packages/tf2_msgs` (nguyên bản từ `ros2/geometry2` nhánh humble, giấy phép BSD).
* micro-ROS mặc định: MTU 512 B, lịch sử stream 4 → message reliable tối đa ~2 KB
  (`/odom` ~0.7 KB được phân mảnh tự động). Thời gian chờ xác nhận publish reliable được giảm từ
  1000 ms xuống 50 ms để vòng publish không bị treo khi Wi-Fi kém.
