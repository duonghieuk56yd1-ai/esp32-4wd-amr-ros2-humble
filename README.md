# Robot tự hành ESP32 + LiDAR · micro-ROS · ROS 2 Humble qua Wi-Fi — Cẩm nang A–Z

Robot vi sai 3 bánh: **2 bánh chủ động phía sau có encoder** (JGA25-370 + TB6612FNG) và **1 bánh
mắt trâu phía trước**, LiDAR 2D (**YDLIDAR X2** hoặc **Camsense X1**). ESP32 nói chuyện với Laptop
**hoàn toàn không dây qua Wi-Fi UDP**; Laptop chạy ROS 2 Humble để hiển thị 3D, quét bản đồ (SLAM) và
tự dẫn đường tránh vật cản (Nav2).

Tài liệu này đi theo đúng thứ tự làm việc: **mua linh kiện → đấu nguồn → đấu dây → cài phần mềm →
nạp firmware → Bước 1 → Bước 2 → Bước 3 → Bước 4**. Làm lần lượt, mỗi bước đều có cách kiểm tra
"đã đúng chưa" trước khi sang bước sau.

## Mục lục

0. [Tổng quan hệ thống](#0-tổng-quan-hệ-thống)
1. [Linh kiện & dụng cụ](#1-linh-kiện--dụng-cụ)
2. [Sơ đồ cấp nguồn an toàn](#2-sơ-đồ-cấp-nguồn-an-toàn-lipo-3s--2-mạch-buck)
3. [Bảng nối dây chi tiết](#3-bảng-nối-dây-chi-tiết)
4. [Cài phần mềm trên Laptop Ubuntu 22.04](#4-cài-phần-mềm-trên-laptop-ubuntu-2204)
5. [Cấu hình & nạp firmware ESP32](#5-cấu-hình--nạp-firmware-esp32)
6. [Bước 1 — Kết nối Wi-Fi & xem xe 3D trên RViz2](#6-bước-1--kết-nối-wi-fi--xem-xe-3d-trên-rviz2)
7. [Bước 2 — Chạy thử động cơ bằng bàn phím & hiệu chỉnh Odometry](#7-bước-2--chạy-thử-động-cơ-bằng-bàn-phím--hiệu-chỉnh-odometry)
8. [Bước 3 — Quét bản đồ bằng SLAM](#8-bước-3--quét-bản-đồ-bằng-slam)
9. [Bước 4 — Nav2 tự động dẫn đường, tránh vật cản](#9-bước-4--nav2-tự-động-dẫn-đường-tránh-vật-cản)
10. [Xử lý sự cố](#10-xử-lý-sự-cố)
11. [Kiểm thử tự động & CI](#11-kiểm-thử-tự-động--ci)
12. [Cấu trúc repo & tài liệu chi tiết](#12-cấu-trúc-repo--tài-liệu-chi-tiết)

---

## 0. Tổng quan hệ thống

```
        ROBOT (ESP32-WROOM-32)                            LAPTOP (Ubuntu 22.04 + ROS 2 Humble)
 ┌──────────────────────────────────────┐   Wi-Fi    ┌───────────────────────────────────────────┐
 │ Core 1: encoder → PID 50 Hz → TB6612 │  2.4 GHz   │ micro-ROS agent  UDP 8888                 │
 │         odometry · failsafe          │◄──────────►│   ⇄ /cmd_vel  /odom  /joint_states  /tf    │
 │ Core 1: micro-ROS node "esp32_base"  │            │ amr_lidar_bridge UDP 8889 → /scan          │
 │ Core 0: LiDAR (Serial2) → gói UDP    │            │ robot_state_publisher · RViz2             │
 └──────────────────────────────────────┘            │ slam_toolbox (bản đồ) · Nav2 (dẫn đường)  │
                                                     └───────────────────────────────────────────┘
```

| Topic / TF | Kiểu | Ai phát | Tần số |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | bàn phím (teleop) hoặc Nav2 → ESP32 | khi có lệnh |
| `/odom` | `nav_msgs/Odometry` | ESP32 | 30 Hz |
| `/joint_states` | `sensor_msgs/JointState` (`left_wheel_joint`, `right_wheel_joint`) | ESP32 | 30 Hz |
| `/tf` `odom → base_footprint` | `tf2_msgs/TFMessage` | ESP32 | 30 Hz |
| `/scan` (`frame_id = laser_frame`) | `sensor_msgs/LaserScan` | `amr_lidar_bridge` | ~7 Hz (X2), ~5 Hz (X1) |
| `/tf` `base_footprint → base_link → bánh xe, laser_frame` | URDF | `robot_state_publisher` | 30 Hz |
| `/tf` `map → odom` | | slam_toolbox (Bước 3) hoặc AMCL (Bước 4) | |

---

## 1. Linh kiện & dụng cụ

| Linh kiện | SL | Ghi chú |
|---|---|---|
| ESP32 DevKit V1 (ESP32-WROOM-32) | 1 | board PlatformIO `esp32dev` |
| Module driver **TB6612FNG** | 1 | 2 kênh, 1.2 A liên tục / 3.2 A đỉnh mỗi kênh, VM tối đa 15 V |
| Động cơ **JGA25-370 12 V có encoder Hall**, hộp số 1:34 | 2 | 11 xung/vòng motor × 34 = **374 xung/vòng bánh** |
| Bánh xe Ø65 mm (bán kính 32.5 mm) + khớp nối trục | 2 | |
| Bánh mắt trâu (ball caster) | 1 | gắn phía trước |
| LiDAR **YDLIDAR X2/X2L** *hoặc* **Camsense X1** | 1 | UART 115200, nguồn 5 V |
| Pin **LiPo 3S** 11.1 V (1500–2200 mAh, ≥ 20C) + sạc cân bằng | 1 | đầy 12.6 V, cạn 9.9 V |
| Mạch hạ áp **Buck #1: 5 V, ≥ 3 A** (LM2596, XL4015, MP1584…) | 1 | cho ESP32 + LiDAR |
| Mạch hạ áp **Buck #2: 3.3 V** (Mini-360, LM2596 chỉnh 3.3 V…) | 1 | cho 2 encoder + logic TB6612 |
| Cầu chì 5 A + đế, công tắc nguồn ≥ 5 A, giắc XT60 | 1 | |
| Tụ hoá 470 µF/25 V, 1000 µF/10 V, 100 µF/10 V; tụ gốm 100 nF × 2 | | chống sụt áp, lọc nhiễu motor |
| Còi báo pin yếu LiPo (cắm vào dây cân bằng) | 1 | rất nên có |
| Dây AWG20 (đường nguồn), dây dupont/AWG26 (tín hiệu), cáp USB **có truyền dữ liệu** | | |
| Đồng hồ vạn năng, mỏ hàn, thước dây, băng dính đánh dấu sàn | | |

---

## 2. Sơ đồ cấp nguồn an toàn (LiPo 3S → 2 mạch Buck)

**Vì sao ESP32 hay bị reset (brownout)?** Lúc motor khởi động/đảo chiều, dòng vọt lên vài ampe làm
điện áp pin sụt và đất (GND) "nảy"; cùng lúc Wi-Fi của ESP32 phát cần ~0.5 A đỉnh. Nếu ESP32 lấy
điện chung đường với motor (hoặc từ chân 5V của driver, hoặc chỉ từ cổng USB), điện áp 3.3 V trên
chip tụt dưới ~2.5 V → **brownout reset**. Cách chữa: mỗi khối có **đường nguồn riêng đi thẳng từ
pin**, có tụ dự trữ, và GND nối **hình sao**.

```
                  Cầu chì 5 A     Công tắc
 LiPo 3S (+) ───────[ ~ ]────────[ / ]──────●── (A) ──────────────────► TB6612FNG  VM
 11.1–12.6 V                                │                    tụ 470 µF/25 V sát chân VM–GND
                                            │
                                            ├── (B) ► Buck #1 IN+    OUT+ 5.0 V ──●──► ESP32  VIN
                                            │                                     ├──► LiDAR  5V
                                            │                    tụ 1000 µF/10 V ─┘
                                            │
                                            └── (C) ► Buck #2 IN+    OUT+ 3.3 V ──●──► Encoder TRÁI  VCC
                                                                                  ├──► Encoder PHẢI  VCC
                                                                                  ├──► TB6612FNG  VCC (logic)
                                                                 tụ 100 µF/10 V ──┘
 LiPo 3S (−) ────────────────────────── ● ĐIỂM GND CHUNG (hình sao) ──────────────────────────────
                                        ├─► TB6612 GND (gần VM)      ├─► Buck #1 IN− / OUT−
                                        ├─► Buck #2 IN− / OUT−       ├─► ESP32 GND
                                        └─► LiDAR GND                └─► Encoder GND (×2)

 Motor: hàn 1 tụ gốm 100 nF ngay giữa 2 cực của MỖI motor (giảm nhiễu chổi than).
```

**Nguyên tắc bắt buộc**

1. Ba nhánh (A) motor, (B) Buck 5 V, (C) Buck 3.3 V **tách ra ngay sau công tắc**, mỗi nhánh một dây
   riêng — không nối tiếp kiểu "pin → driver → buck".
2. **Chỉnh điện áp buck TRƯỚC khi nối tải**: cấp pin cho buck, vặn biến trở đo bằng đồng hồ:
   Buck #1 = **5.0–5.1 V**, Buck #2 = **3.30 V** (không vượt 3.4 V). Sau đó mới nối ESP32/LiDAR/encoder.
3. Chỉ cấp 5 V vào chân **VIN** (hoặc 5V) của ESP32. **Tuyệt đối không cấp 5 V vào chân 3V3.**
4. Encoder dùng **3.3 V** (từ Buck #2): tín hiệu A/B của encoder bằng điện áp nguồn của nó, mà chân
   GPIO ESP32 **không chịu 5 V**.
5. Mọi GND phải chung: thiếu GND chung thì encoder/LiDAR/driver đọc sai hoặc cháy chân.
6. Buck #1 và Buck #2 cùng bật/tắt bằng một công tắc (encoder không được có điện khi ESP32 tắt).

**Trình tự cấp điện lần đầu**

1. Chưa cắm ESP32/LiDAR/encoder: bật công tắc, chỉnh và đo 2 buck như trên. Tắt công tắc.
2. Đo điện trở giữa (+) và GND của từng đường 12 V / 5 V / 3.3 V: phải lớn (không chập).
3. Nối tải, **kê robot cho 2 bánh không chạm đất**, bật công tắc, đo lại: VM ≈ 11–12.6 V, 5 V, 3.3 V.
   LED xanh trên ESP32 bắt đầu nháy.

**Pin LiPo**

* Dùng sạc cân bằng (balance), không sạc khi không có người trông, cất trong túi chống cháy.
* Không xả dưới **3.3 V/cell (9.9 V)**: gắn còi báo pin yếu vào dây cân bằng (đặt ngưỡng 3.5 V/cell).
* Pin phồng, móp → bỏ, không dùng tiếp.

**Cắm USB khi đang chạy pin?** ESP32 DevKit V1 thường có diode giữa cổng USB và chân VIN nên cắm USB
để xem log khi robot đang chạy pin là được. Nếu board của bạn không có diode này (một số bản clone),
chỉ cắm USB khi **tắt công tắc pin** (lúc đó motor không có điện, ESP32 chạy bằng USB).

---

## 3. Bảng nối dây chi tiết

Chân đã chọn để tránh chân flash (GPIO 6–11), UART nạp code (1, 3), chân strapping quan trọng (0, 12)
và GPIO 36/39 (lỗi phần cứng gây ngắt giả khi bật Wi-Fi). Đổi chân trong
[`firmware_esp32/include/pinout.h`](firmware_esp32/include/pinout.h).

### 3.1. ESP32 ↔ TB6612FNG (kênh A = bánh TRÁI, kênh B = bánh PHẢI)

| ESP32 | TB6612FNG | Chức năng |
|---|---|---|
| GPIO 25 | PWMA | PWM 20 kHz bánh trái |
| GPIO 32 | AIN1 | chiều bánh trái |
| GPIO 33 | AIN2 | chiều bánh trái |
| GPIO 14 | PWMB | PWM 20 kHz bánh phải |
| GPIO 26 | BIN1 | chiều bánh phải |
| GPIO 27 | BIN2 | chiều bánh phải |
| GPIO 13 | STBY | LOW = standby (2 motor nhả) |
| GND | GND | GND chung |

### 3.2. Nguồn & motor của TB6612FNG

| TB6612FNG | Nối tới | Ghi chú |
|---|---|---|
| VM | Pin 12 V (sau cầu chì + công tắc, nhánh A) | tụ 470 µF/25 V sát chân VM–GND |
| VCC | Buck #2 **3.3 V** | nguồn logic (2.7–5.5 V) |
| GND (cả 2–3 chân) | GND chung | |
| AO1 / AO2 | Motor **TRÁI**: M+ (đỏ) / M− (trắng) | |
| BO1 / BO2 | Motor **PHẢI**: M+ (đỏ) / M− (trắng) | |

### 3.3. Encoder JGA25-370 ↔ ESP32

Dây 6 màu thường gặp của JGA25-370 có encoder (**kiểm tra lại nhãn của người bán**):
đỏ = M+, trắng = M−, xanh dương = VCC encoder, đen = GND encoder, vàng = C1 (A), xanh lá = C2 (B).

| Encoder | Bánh TRÁI | Bánh PHẢI |
|---|---|---|
| C1 / A (vàng) | GPIO 18 | GPIO 23 |
| C2 / B (xanh lá) | GPIO 19 | GPIO 4 |
| VCC (xanh dương) | Buck #2 **3.3 V** | Buck #2 **3.3 V** |
| GND (đen) | GND chung | GND chung |
| M+ / M− (đỏ / trắng) | TB6612 AO1 / AO2 | TB6612 BO1 / BO2 |

Firmware bật điện trở kéo lên nội và giải mã **x4** (ngắt trên mọi cạnh của A và B):
374 × 4 = **1496 tick/vòng bánh** ≈ 0.14 mm quãng đường mỗi tick. Lắp ngược chiều không sao: chỉnh
bằng `*_INVERTED` ở Bước 2.

### 3.4. LiDAR ↔ ESP32 (Serial2, 115200 baud)

| LiDAR | ESP32 / nguồn | YDLIDAR X2/X2L | Camsense X1 |
|---|---|---|---|
| 5V / VCC | Buck #1 **5 V** | ✔ | ✔ |
| GND | GND chung | ✔ | ✔ |
| TX (dữ liệu ra) | **GPIO 16** (RX2) | ✔ | ✔ |
| RX | GPIO 17 (TX2) | không cần | không có |
| M_SCTR (tốc độ motor) | GPIO 15 | tuỳ chọn, không nối thì quay tốc độ mặc định | không có |

Tín hiệu TX của X2 và Camsense X1 là mức 3.3 V, nối thẳng vào ESP32. Chọn model LiDAR trong
`robot_config.h` (mục 5.3).

### 3.5. Tra cứu theo chân ESP32

| Chân ESP32 | Nối tới | | Chân ESP32 | Nối tới |
|---|---|---|---|---|
| VIN | Buck #1 5 V | | GPIO 18 | Encoder TRÁI A |
| GND | GND chung | | GPIO 19 | Encoder TRÁI B |
| GPIO 2 | LED trên board (trạng thái) | | GPIO 23 | Encoder PHẢI A |
| GPIO 4 | Encoder PHẢI B | | GPIO 25 | TB6612 PWMA |
| GPIO 13 | TB6612 STBY | | GPIO 26 | TB6612 BIN1 |
| GPIO 14 | TB6612 PWMB | | GPIO 27 | TB6612 BIN2 |
| GPIO 15 | LiDAR M_SCTR (tuỳ chọn) | | GPIO 32 | TB6612 AIN1 |
| GPIO 16 | LiDAR TX | | GPIO 33 | TB6612 AIN2 |
| GPIO 17 | LiDAR RX (tuỳ chọn) | | 3V3 | **không dùng** để cấp ngoài |

---

## 4. Cài phần mềm trên Laptop Ubuntu 22.04

### 4.1. ROS 2 Humble (theo hướng dẫn chính thức docs.ros.org)

Locale UTF-8:

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Thêm kho apt của ROS 2:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -fsSL -o /dev/null -w '%{url_effective}' https://github.com/ros-infrastructure/ros-apt-source/releases/latest | awk -F/ '{print $NF}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Cài ROS 2 Humble Desktop + công cụ phát triển. **Phải `apt upgrade` trước** (Ubuntu 22.04 mới cài
có thể bị gỡ nhầm gói hệ thống `systemd`/`udev` nếu bỏ qua bước này):

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4.2. Nav2, slam_toolbox, teleop và các gói cần thiết

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-tools git
```

### 4.3. Tải mã nguồn, build workspace và micro-ROS Agent

micro-ROS Agent cho Humble không có gói apt → build từ mã nguồn ngay trong workspace (file
[`ros2_ws/micro_ros_agent.repos`](ros2_ws/micro_ros_agent.repos); lần build đầu mất 3–5 phút và tải
thêm Micro-XRCE-DDS-Agent từ GitHub):

```bash
cd ~
git clone https://github.com/duonghieuk56yd1-ai/esp32-4wd-amr-ros2-humble.git
cd ~/esp32-4wd-amr-ros2-humble/ros2_ws
vcs import src < micro_ros_agent.repos
```

```bash
sudo rosdep init   # chỉ lần đầu trên máy; báo "already exists" thì bỏ qua
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

```bash
colcon build --symlink-install
echo "source ~/esp32-4wd-amr-ros2-humble/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Kiểm tra: `ros2 pkg list | grep -E "amr_|micro_ros_agent"` phải thấy `amr_bringup`,
`amr_description`, `amr_lidar_bridge`, `micro_ros_agent`.

> Không muốn build agent? Bỏ lệnh `vcs import` và cài Docker
> (`sudo apt install docker.io && sudo usermod -aG docker $USER`, đăng xuất/đăng nhập lại):
> `bringup.launch.py` tự chạy image `microros/micro-ros-agent:humble` khi không có gói agent.

### 4.4. Mở tường lửa `ufw` cho UDP 8888 & 8889

```bash
sudo ufw allow 8888/udp comment 'micro-ROS agent'
sudo ufw allow 8889/udp comment 'AMR LiDAR'
sudo ufw status
```

(ufw đang tắt thì lệnh chỉ lưu luật — vẫn nên chạy để khi bật ufw không bị chặn.)

### 4.5. Wi-Fi cho robot

* ESP32 **chỉ chạy Wi-Fi 2.4 GHz**. Không dùng mạng "Khách/Guest" (chặn thiết bị nói chuyện với nhau).
* **Tắt tiết kiệm điện Wi-Fi của Laptop** — nếu bật, gói tin tới Laptop có thể trễ 100–300 ms, robot
  dễ báo mất kết nối. Xem tên card Wi-Fi bằng `iw dev` (ví dụ `wlo1`), rồi:

  ```bash
  sudo iw dev wlo1 set power_save off
  ```

  Tắt vĩnh viễn:

  ```bash
  printf '[connection]\nwifi.powersave = 2\n' | sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf
  sudo systemctl restart NetworkManager
  ```

* Không có router? Cho Laptop phát Wi-Fi 2.4 GHz (`band bg`; Laptop sẽ có IP `10.42.0.1` →
  `AGENT_IP "10.42.0.1"`; card Wi-Fi thường không vừa phát vừa bắt mạng khác):

  ```bash
  nmcli device wifi hotspot ifname wlo1 band bg ssid AMR-NET password amr12345
  ```

### 4.6. PlatformIO (nạp firmware cho ESP32)

Cách 1 — VS Code + extension **PlatformIO IDE**. Cách 2 — dòng lệnh:

```bash
sudo apt install -y python3-venv python3-pip git cmake curl
curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
echo 'export PATH="$PATH:$HOME/.platformio/penv/bin"' >> ~/.bashrc
source ~/.bashrc
```

Quyền truy cập cổng USB-Serial (CP210x / CH340) — sau đó **đăng xuất rồi đăng nhập lại**:

```bash
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -a -G dialout,plugdev $USER
sudo apt remove -y brltty   # brltty của Ubuntu 22.04 hay "cướp" cổng CH340 → mất /dev/ttyUSB0
```

---

## 5. Cấu hình & nạp firmware ESP32

### 5.1. Lấy IP của Laptop

```bash
hostname -I
```

Ví dụ `192.168.1.100`. Nên đặt **IP cố định** cho Laptop (DHCP reservation trong router), vì ESP32
gửi cả micro-ROS lẫn LiDAR tới đúng IP này.

### 5.2. Khai báo Wi-Fi (không lộ mật khẩu khi push Git)

Tạo file `firmware_esp32/include/secrets_local.h` (đã nằm trong `.gitignore`):

```bash
cd ~/esp32-4wd-amr-ros2-humble/firmware_esp32
cat > include/secrets_local.h <<'EOF'
#define WIFI_SSID     "TenWiFi_2.4GHz"
#define WIFI_PASSWORD "MatKhauWiFi"
#define AGENT_IP      "192.168.1.100"
EOF
```

`AGENT_PORT` mặc định 8888 (trong [`secrets.h`](firmware_esp32/include/secrets.h)).

### 5.3. Chọn LiDAR — 1 dòng trong `include/robot_config.h`

```c
#define LIDAR_YDLIDAR_X2        // YDLIDAR X2 / X2L
// #define LIDAR_CAMSENSE_X1    // Camsense X1
```

Các thông số khác cùng file: `WHEEL_RADIUS_M 0.0325`, `WHEEL_SEPARATION_M 0.18`, `ENCODER_PPR 374`,
`PID_KP/KI/KD`, giới hạn vận tốc (`MAX_LINEAR_VEL_MPS 0.40`, `MAX_ANGULAR_VEL_RAD_S 3.0`),
`FAILSAFE_TIMEOUT_MS 500`, `CMD_VEL_TIMEOUT_MS 500`, đảo chiều motor/encoder…

### 5.4. Build & nạp

Cắm ESP32 vào Laptop bằng cáp USB (nên tắt công tắc pin khi nạp lần đầu):

```bash
cd ~/esp32-4wd-amr-ros2-humble/firmware_esp32
pio run
```

Lần đầu mất 5–10 phút (tải toolchain + build thư viện micro-ROS), các lần sau vài giây.

```bash
pio run -t upload
```

Nếu báo `Failed to connect to ESP32`: giữ nút **BOOT** trên board khi dòng `Connecting....` hiện ra.

```bash
pio device monitor
```

(Thoát bằng `Ctrl+C`.) Log mẫu khi mọi thứ đúng:

```
[     52][MAIN] ESP32 2WD AMR | micro-ROS Humble qua Wi-Fi UDP | LiDAR: YDLIDAR X2/X2L/S2
[   2310][UROS] Wi-Fi OK: IP 192.168.1.123, RSSI -52 dBm -> tim agent 192.168.1.100:8888
[   2890][UROS] Da ket noi micro-ROS agent (node 'esp32_base', client key 0x1A2B3C4D)
[   2960][UROS] Dong bo thoi gian voi agent OK
[   3000][STAT] wifi=OK rssi=-52 agent=ON link=8ms state=FAILSAFE cmd=(0.00,0.00) ...
[   3000][STAT] odom=(0.000 m, 0.000 m, 0.0 deg) v=0.00 w=0.00 | lidar=OK 7.0Hz valid=325 ...
```

`state=FAILSAFE` lúc mới bật là **bình thường**: motor chỉ được phép chạy sau khi nhận lệnh
`/cmd_vel` đầu tiên.

| LED xanh (GPIO 2) | Ý nghĩa |
|---|---|
| Nháy nhanh | Đang kết nối Wi-Fi (sai SSID/mật khẩu, Wi-Fi 5 GHz, `AGENT_IP` sai định dạng) |
| Nháy chậm | Có Wi-Fi, đang chờ micro-ROS agent trên Laptop |
| Sáng liên tục | Đã kết nối agent |
| Nháy kép | `FAULT` động cơ (xem Bước 2) |

---

## 6. Bước 1 — Kết nối Wi-Fi & xem xe 3D trên RViz2

**1.1. Xem mô hình 3D khi chưa có robot** (kiểm tra cài đặt ROS):

```bash
ros2 launch amr_description display.launch.py preview:=true
```

RViz2 hiện xe xanh dương, 2 bánh đen, bánh mắt trâu bạc, LiDAR đỏ/đen trên nóc; kéo thanh trượt
`left_wheel_joint` thấy bánh quay. Đóng lại bằng `Ctrl+C`.

**1.2. Bật robot** (kê 2 bánh khỏi mặt đất), chờ LED **nháy chậm** (đã có Wi-Fi).

**1.3. Chạy toàn bộ phần Laptop** (micro-ROS agent UDP 8888 + LiDAR bridge UDP 8889 + URDF + RViz2):

```bash
ros2 launch amr_bringup bringup.launch.py
```

Vài giây sau LED **sáng liên tục**, terminal agent in `session established`, `lidar_bridge` in
`Nhận LiDAR YDLIDAR X2 từ 192.168.1.123: 360 tia, 7.0 Hz…`.

**1.4. Kiểm tra** (mở terminal mới):

```bash
ros2 topic list
```

Phải có `/cmd_vel /joint_states /odom /scan /tf /tf_static /robot_description`.

```bash
ros2 topic hz /odom
```

≈ 30 Hz. Với LiDAR: `ros2 topic hz /scan` ≈ 7 Hz (X2) / 5 Hz (X1).

```bash
ros2 run tf2_tools view_frames
```

Tạo `frames_*.pdf`: cây `odom → base_footprint → base_link → left/right_wheel_link, caster_wheel_link,
laser_frame` phải liền mạch.

**1.5. Trên RViz2** (Fixed Frame `odom`): thấy xe 3D, các chấm đỏ của LiDAR bao quanh đúng hình dạng
phòng (phía trước xe = trục X đỏ). **Xoay tay một bánh về phía trước** → vạch vàng trên bánh 3D quay
theo cùng chiều. Đưa tay lại gần phía trước LiDAR → chấm đỏ xuất hiện phía trước xe 3D.

**1.6. Chất lượng Wi-Fi**: ping IP của ESP32 (in trong log `Wi-Fi OK: IP …`):

```bash
ping -c 20 192.168.1.123
```

Trễ trung bình nên < 20 ms, không mất gói; RSSI trong log nên > −70 dBm.

---

## 7. Bước 2 — Chạy thử động cơ bằng bàn phím & hiệu chỉnh Odometry

Giữ `bringup.launch.py` chạy ở terminal 1. Robot vẫn **kê bánh khỏi mặt đất** cho tới mục 2.3.

**2.1. Chiều encoder.** Mở log Serial (`pio device monitor`, hoặc xem `ros2 topic echo /joint_states`).
Xoay tay từng bánh **về phía trước** → `tick` (và `position`) của bánh đó phải **tăng**. Nếu giảm, đổi
`ENCODER_LEFT_INVERTED` / `ENCODER_RIGHT_INVERTED` (0 ↔ 1) trong `robot_config.h`, nạp lại
(`pio run -t upload`). `encErr` phải đứng yên ở 0.

**2.2. Chiều motor.** Gửi lệnh tiến chậm liên tục 10 Hz (`Ctrl+C` để dừng):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

Cả 2 bánh phải lăn **về phía trước**. Bánh nào quay ngược → đổi `MOTOR_LEFT_INVERTED` /
`MOTOR_RIGHT_INVERTED`. Nếu motor và encoder lệch chiều nhau, firmware tự ngắt và báo
`FAULT: banh ... quay NGUOC chieu lenh` (LED nháy kép); sửa xong gửi `/cmd_vel` = 0 để xoá lỗi.

**2.3. Lái bằng bàn phím** (đặt robot xuống sàn, khu vực trống):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.8
```

| Phím | Tác dụng |
|---|---|
| `i` / `,` | tiến / lùi |
| `j` / `l` | quay trái / phải tại chỗ |
| `u` `o` `m` `.` | đi cong |
| `k` (hoặc phím bất kỳ khác) | dừng |
| `q` / `z` | tăng / giảm 10 % mọi tốc độ |

**Nhấn GIỮ phím** để xe chạy liên tục: teleop chỉ gửi lệnh khi có phím, còn firmware tự dừng xe nếu
**0.5 s** không có lệnh mới (`CMD_VEL_TIMEOUT_MS`, an toàn khi Laptop treo). Xe có thể khựng nhẹ
~0.5 s đầu khi giữ phím (độ trễ lặp phím của bàn phím) — bình thường.

**Kiểm tra failsafe**: đang chạy, tắt Wi-Fi Laptop → motor phải ngắt trong 0.5 s (log `FAILSAFE`).
Bật lại Wi-Fi, firmware tự kết nối lại; xe chỉ chạy tiếp khi có lệnh **mới**.

**2.4. PID (thường không cần chỉnh).** Trong log Serial `wL=đo/đặt wR=đo/đặt` (rad/s): giá trị đo
phải bám giá trị đặt trong ~0.3 s. Bám chậm → tăng `PID_KP`; dao động → giảm `PID_KP`; còn sai lệch
lâu dài → tăng `PID_KI`.

**2.5. Hiệu chỉnh Odometry** — dùng công cụ `calibrate_odom` (in quãng đường và góc quay theo
`/odom`, rồi tính thông số mới):

*a) Bán kính bánh (đi thẳng).* Dán băng dính đánh dấu vị trí xuất phát (ví dụ mép trước xe).
Terminal 2:

```bash
ros2 run amr_bringup calibrate_odom
```

Terminal 3 — đi thẳng 0.1 m/s trong 10 s (≈ 1 m) rồi tự dừng:

```bash
ros2 topic pub -r 10 -t 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

Đo **quãng đường thật** bằng thước (cùng điểm mốc trên xe), về terminal 2 nhấn `Ctrl+C`, nhập số đo
(m). Công cụ in `WHEEL_RADIUS_M mới = r × thật / odom`.

*b) Khoảng cách 2 bánh (quay tại chỗ, làm SAU khi đã sửa bán kính).* Dán vạch mốc thẳng hướng mũi xe.
Chạy lại `calibrate_odom`, quay ≈ 5 vòng:

```bash
ros2 topic pub -r 10 -t 630 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

Dùng teleop nhấp `j`/`l` để mũi xe trùng đúng vạch mốc, đếm **số vòng thật** (ví dụ 5), `Ctrl+C` ở
terminal 2 và nhập 5. Công cụ in `WHEEL_SEPARATION_M mới = L × vòng_odom / vòng_thật`.

*c) Cập nhật* — sửa **cả hai** nơi cho khớp (test tự động sẽ báo nếu lệch):

| File | Dòng |
|---|---|
| `firmware_esp32/include/robot_config.h` | `#define WHEEL_RADIUS_M ...f` · `#define WHEEL_SEPARATION_M ...f` |
| `ros2_ws/src/amr_description/urdf/amr_3wd.urdf.xacro` | `<xacro:arg name="wheel_radius" default="..."/>` · `wheel_separation` |

Nạp lại firmware (`pio run -t upload`), khởi động lại `bringup.launch.py`, đo lại cho tới khi sai số
< 2 %. Tốt: đi thẳng 1 m sai < 2 cm, quay 5 vòng sai < 10°.

---

## 8. Bước 3 — Quét bản đồ bằng SLAM

Tắt `bringup.launch.py` ở Bước 1–2 (`Ctrl+C`), rồi chạy (lệnh này đã gồm agent + LiDAR + URDF):

```bash
ros2 launch amr_bringup slam.launch.py
```

RViz2 mở `slam.rviz` (Fixed Frame `map`). Terminal khác, lái chậm:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5
```

Mẹo để bản đồ đẹp:

* Đi chậm (≤ 0.2 m/s), **quay chậm** (LiDAR chỉ 5–7 vòng/s), dừng một chút ở các góc phòng.
* Đi dọc tường, đi hết các phòng, cuối cùng **quay về điểm xuất phát** để slam_toolbox "đóng vòng".
* Tránh kính, gương, vật đen bóng (LiDAR không thấy).

Lưu bản đồ khi đã ưng ý (tên khớp mặc định của Bước 4):

```bash
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/amr_map
```

Được `~/maps/amr_map.pgm` (ảnh) + `~/maps/amr_map.yaml`. **Ghi nhớ vị trí & hướng xuất phát** của xe:
đó là gốc bản đồ, Bước 4 sẽ đặt xe về đúng chỗ này.

---

## 9. Bước 4 — Nav2 tự động dẫn đường, tránh vật cản

Đặt xe về **đúng điểm và hướng xuất phát lúc quét bản đồ**, rồi:

```bash
ros2 launch amr_bringup navigation.launch.py
```

(Bản đồ khác: thêm `map:=/đường/dẫn/ban_do.yaml`.) Trong RViz2 (`navigation.rviz`):

1. Chấm đỏ LiDAR phải **trùng tường trên bản đồ**. Nếu lệch: bấm **2D Pose Estimate**, nhấn-kéo tại
   vị trí thật của xe theo hướng mũi xe.
2. Bấm **Nav2 Goal** (hoặc **2D Goal Pose**), nhấn-kéo tại điểm đích → Nav2 vẽ đường (xanh) và xe tự
   chạy tới (tối đa 0.25 m/s), lệch hướng > 45° thì quay tại chỗ trước.
3. **Thử tránh vật cản**: đặt thùng giấy chắn ngang đường xe đang đi → vật hiện trên costmap (vùng
   màu quanh vật), Nav2 tính lại đường vòng qua (khoảng 1 lần/giây); vật quá gần thì xe dừng chờ
   (collision detection) rồi đi tiếp khi đường thông.
4. Panel **Navigation 2**: xem trạng thái, **Cancel** để huỷ.

**Dừng khẩn cấp**: `Ctrl+C` terminal Nav2 (firmware dừng xe sau 0.5 s không có lệnh) hoặc tắt công tắc.

Vừa quét bản đồ vừa dẫn đường (không cần bản đồ có sẵn):

```bash
ros2 launch amr_bringup navigation.launch.py slam:=true
```

Các thông số Nav2 đã tối ưu cho xe vi sai nhỏ qua Wi-Fi (`transform_tolerance: 0.5` s, Regulated
Pure Pursuit 10 Hz, footprint tròn 0.21 m, inflation 0.35 m) —
xem [`ros2_ws/README.md`](ros2_ws/README.md#cấu-hình-chính).

### Chạy nhanh hằng ngày

| Việc | Lệnh |
|---|---|
| Xem robot | `ros2 launch amr_bringup bringup.launch.py` |
| Lái tay | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.8` |
| Quét bản đồ | `ros2 launch amr_bringup slam.launch.py` |
| Lưu bản đồ | `ros2 run nav2_map_server map_saver_cli -f ~/maps/amr_map` |
| Tự dẫn đường | `ros2 launch amr_bringup navigation.launch.py` |

---

## 10. Xử lý sự cố

| Hiện tượng | Nguyên nhân / cách xử lý |
|---|---|
| Log `Brownout detector was triggered`, ESP32 tự khởi động lại khi motor chạy | Sai sơ đồ nguồn (mục 2): ESP32 phải lấy 5 V từ Buck #1 đi thẳng từ pin, có tụ 1000 µF, GND hình sao; pin yếu (< 10.5 V) → sạc |
| LED nháy nhanh mãi | SSID/mật khẩu, Wi-Fi phải 2.4 GHz, `AGENT_IP` đúng định dạng |
| LED nháy chậm mãi | Chưa chạy `bringup.launch.py`; `AGENT_IP` ≠ IP Laptop (`hostname -I`); ufw chặn UDP 8888; mạng Guest/AP isolation |
| Có `/esp32_base` nhưng không có topic | `ROS_DOMAIN_ID` của Laptop phải bằng `ROS_DOMAIN_ID` trong `robot_config.h` (mặc định 0); chạy `ros2 daemon stop` rồi thử lại |
| Không có `/scan` | Log `lidar_bridge` cảnh báo không nhận gói → ufw 8889; log ESP32 `lidar=--` → dây TX LiDAR vào GPIO 16, LiDAR có quay không (nguồn 5 V), đúng model trong `robot_config.h` |
| LiDAR trong RViz bị lật gương / lệch góc | Đổi `LIDAR_ANGLE_CLOCKWISE`; xoay bằng `LIDAR_YAW_OFFSET_DEG` |
| Bánh 3D trong RViz quay ngược bánh thật | Đổi `ENCODER_*_INVERTED` (Bước 2.1) |
| Log `FAULT ... quay NGUOC chieu lenh` | Motor và encoder cùng bánh lệch chiều: đổi `MOTOR_*_INVERTED` hoặc `ENCODER_*_INVERTED` |
| Log `FAULT ... khong quay` | Kẹt bánh, tuột dây encoder, hoặc VM không có điện |
| Log `FAILSAFE: mat ket noi` liên tục | Wi-Fi yếu (RSSI < −75 dBm), chưa tắt power save Laptop (mục 4.5), router xa |
| Xe chạy khựng khi dùng teleop | Phải **giữ phím** (lệnh hết hạn sau 0.5 s); muốn khác thì sửa `CMD_VEL_TIMEOUT_MS` |
| Xe đi thẳng bị lệch | Hiệu chỉnh odometry (Bước 2.5); 2 motor khác nhau nhiều → kiểm tra PID trên log `wL`/`wR` |
| Bản đồ bị nhoè, tường đôi | Quay chậm hơn; hiệu chỉnh odometry; kiểm tra `ros2 topic hz /scan` |
| Nav2 không chạy, chờ TF `map` | Đặt **2D Pose Estimate**; `/scan` có dữ liệu không |
| Nav2: xe quay vòng / lắc mãi | Vị trí AMCL sai → 2D Pose Estimate; giảm `desired_linear_vel` |
| `pio run -t upload` không thấy cổng | Cáp USB chỉ sạc; chưa vào nhóm `dialout`; `brltty` chưa gỡ (mục 4.6) |
| `colcon test` báo `test_matches_firmware` lỗi sau khi hiệu chỉnh | Chưa sửa URDF xacro cho trùng `robot_config.h` (Bước 2.5c) |
| Dòng `1 package had stderr output: amr_lidar_bridge` khi build | Cảnh báo của setuptools, không phải lỗi |

Log Serial mỗi giây có 2 dòng `[STAT]` (trạng thái Wi-Fi/agent, `cmd`, vận tốc đo/đặt, duty PWM,
tick, odometry, LiDAR, stack các task) — dán 2 dòng này khi cần hỏi lỗi.

---

## 11. Kiểm thử tự động & CI

Firmware — 46 unit test chạy trên PC, không cần ESP32:

```bash
cd ~/esp32-4wd-amr-ros2-humble/firmware_esp32 && pio test -e native
```

| Bộ test | Nội dung |
|---|---|
| `test_kinematics` | động học vi sai 2 bánh (thuận/nghịch, giới hạn tốc độ giữ độ cong), chuẩn hoá góc, odometry (đi thẳng/lùi, quay tại chỗ, cung tròn, khép kín vòng tròn, không phụ thuộc bước tích phân), chuỗi đầy đủ tick encoder → odometry so với nghiệm giải tích |
| `test_pid` | P/I/D, kẹp đầu ra, anti-windup, D trên giá trị đo + lọc, đối xứng tiến/lùi, đổi gain không giật, vòng kín với mô hình motor JGA25 (bám đích, đảo chiều, chịu tải, hồi phục sau bão hoà) |
| `test_scan_packet` | gom tia LiDAR (đổi chiều quay, bù góc, giữ điểm gần nhất), đóng gói/giải mã UDP, gói hỏng, gói do bộ mã hoá Python tạo ra |
| `test_quadrature` | giải mã encoder x4, nảy cạnh, mất cạnh, bộ đếm tràn 32 bit |

ROS 2 — 76 test (giải mã gói LiDAR UDP, node bridge, URDF, launch, tham số Nav2/SLAM, công cụ hiệu
chỉnh odometry, **đối chiếu tên topic/frame và thông số giữa firmware ↔ URDF ↔ bridge ↔ SLAM ↔ Nav2 ↔
RViz**, flake8/pep257):

```bash
cd ~/esp32-4wd-amr-ros2-humble/ros2_ws && colcon test && colcon test-result --verbose
```

GitHub Actions ([`.github/workflows/ci.yml`](.github/workflows/ci.yml)) chạy cả hai mỗi lần push:
`pio test` + `pio run` cho firmware, và đúng các lệnh `vcs import` → `rosdep install` → `colcon build`
→ `colcon test` của mục 4.3 trong container `ros:humble-ros-base` (Ubuntu 22.04).

---

## 12. Cấu trúc repo & tài liệu chi tiết

```
esp32-4wd-amr-ros2-humble/
├── firmware_esp32/                 # PlatformIO + micro-ROS (Humble) cho ESP32
│   ├── platformio.ini
│   ├── include/                    # secrets.h · pinout.h · robot_config.h
│   ├── src/                        # control_task (Core 1) · microros_task (Core 1) · lidar_task (Core 0)
│   ├── lib/amr_core/               # PID, động học, encoder, gói LiDAR (logic thuần, test trên PC)
│   ├── test/                       # 4 bộ unit test Unity
│   └── README.md                   # chi tiết firmware, định dạng gói UDP, failsafe
├── ros2_ws/src/
│   ├── amr_description/            # URDF/Xacro + RViz2 (robot_view.rviz)
│   ├── amr_lidar_bridge/           # UDP 8889 → /scan
│   └── amr_bringup/                # bringup · slam · navigation · calibrate_odom
│   (ros2_ws/README.md: chi tiết launch, tham số Nav2/SLAM)
└── .github/workflows/ci.yml
```

* [`firmware_esp32/README.md`](firmware_esp32/README.md) — kiến trúc firmware, định dạng gói LiDAR
  UDP, bảng failsafe, ghi chú kỹ thuật.
* [`ros2_ws/README.md`](ros2_ws/README.md) — các launch file và tham số, cấu hình Nav2/slam_toolbox.
