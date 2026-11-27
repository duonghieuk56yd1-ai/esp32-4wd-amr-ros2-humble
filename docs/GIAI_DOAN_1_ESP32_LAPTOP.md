# Giai đoạn 1 — Chỉ có ESP32 và Laptop: làm gì, hiểu gì

Tài liệu này dành cho lúc bạn **mới có board ESP32, cáp USB và laptop Ubuntu 22.04 đã cài ROS 2
Humble**, chưa có motor, encoder, LiDAR hay pin.

Mỗi bước được viết theo cùng một khuôn:

* **Mục đích** — bước này để làm gì.
* **Hiểu trước** — kiến thức cần biết, giải thích ngắn gọn.
* **Làm** — gõ gì, gõ ở đâu.
* **Kết quả đúng** — bạn phải nhìn thấy gì.
* **Nếu sai** — nguyên nhân hay gặp và cách sửa.

Làm xong giai đoạn này, toàn bộ phần **phần mềm** coi như đã chạy đúng. Khi linh kiện về, bạn chỉ
còn việc lắp mạch rồi làm tiếp từ Bước 2 của [README](../README.md).

---

## Bức tranh tổng thể (đọc trước, 3 phút)

```
  ESP32 (firmware)                         Laptop (ROS 2 Humble)
 ┌─────────────────┐   Wi-Fi, UDP 8888   ┌──────────────────────┐
 │ đọc encoder     │ ──────────────────► │ micro-ROS Agent      │──► các topic ROS 2:
 │ tính odometry   │                     │ ("người phiên dịch") │    /odom /joint_states /tf
 │ PID điều motor  │ ◄────────────────── │                      │◄── /cmd_vel (lệnh lái)
 └─────────────────┘                     └──────────────────────┘
                                                    │
                                    robot_state_publisher + RViz2
                                    (vẽ xe 3D theo dữ liệu thật)
```

Một số khái niệm sẽ gặp suốt tài liệu:

| Từ | Nghĩa dễ hiểu |
|---|---|
| **Node** | Một chương trình nhỏ trong ROS 2. Ví dụ: ESP32 là node `esp32_base`, RViz là một node khác. |
| **Topic** | "Kênh tin nhắn" có tên, như `/odom`. Node này **publish** (gửi), node kia **subscribe** (nghe). |
| **micro-ROS** | Phiên bản ROS 2 thu nhỏ chạy được trên vi điều khiển như ESP32. |
| **micro-ROS Agent** | Chương trình chạy trên laptop. ESP32 quá yếu để nói chuyện trực tiếp với ROS 2, nên nó gửi gói tin gọn nhẹ qua Wi-Fi (UDP cổng 8888); Agent "phiên dịch" sang topic ROS 2 bình thường. **Không có Agent thì ESP32 không xuất hiện trong ROS.** |
| **`/odom`** | Vị trí và vận tốc xe do ESP32 **tự tính** từ số vòng quay bánh (odometry). |
| **`/joint_states`** | Góc quay hiện tại của từng bánh. |
| **TF** | Cây toạ độ: cho biết bánh, LiDAR, thân xe nằm ở đâu so với nhau và so với điểm xuất phát. |
| **URDF** | File mô tả hình dáng và các khớp của xe (bạn vẽ bằng FreeCAD). |
| **robot_state_publisher** | Đọc URDF + `/joint_states`, tính ra TF cho từng bộ phận. |
| **RViz2** | Cửa sổ 3D để **nhìn** mọi dữ liệu trên. |

---

## Chuẩn bị: mở 3 cửa sổ terminal

Suốt tài liệu, bạn dùng **3 terminal** (trong Ubuntu nhấn `Ctrl+Alt+T` để mở một terminal; nhấn
`Ctrl+Shift+T` để mở thêm tab). Nên đặt tên trong đầu như sau:

| Terminal | Dùng để | Chạy lâu dài? |
|---|---|---|
| **T1 — Serial** | Xem ESP32 "nói" gì qua cáp USB | Có |
| **T2 — Laptop** | Chạy Agent + RViz | Có |
| **T3 — Kiểm tra** | Gõ lệnh kiểm tra, lái bằng bàn phím | Gõ rồi xem |

Lệnh nào **chạy lâu dài** sẽ chiếm terminal đó cho tới khi bạn nhấn `Ctrl+C` để dừng. Đó là lý do
cần nhiều terminal.

---

## Bước 0 — Kiểm tra laptop đã sẵn sàng

**Mục đích:** chắc chắn các phần mềm cần thiết đã có, tránh mất thời gian dò lỗi ở bước sau.

**Hiểu trước:** mỗi terminal mới phải được "nạp" môi trường ROS (lệnh `source`). Nếu bạn đã làm
theo README mục 4.1 và 4.3, dòng `source` đã nằm trong file `~/.bashrc` nên mỗi terminal mới sẽ tự
nạp.

**Làm** (ở T3), gõ lần lượt từng lệnh:

```bash
printenv ROS_DISTRO
```

```bash
ros2 pkg list | grep -E "amr_|micro_ros_agent"
```

```bash
pio --version
```

**Kết quả đúng:**

* Lệnh 1 in ra `humble`.
* Lệnh 2 in ra 4 dòng: `amr_bringup`, `amr_description`, `amr_lidar_bridge`, `micro_ros_agent`.
* Lệnh 3 in ra số phiên bản PlatformIO, ví dụ `PlatformIO Core, version 6.x`.

**Nếu sai:**

| Hiện tượng | Làm gì |
|---|---|
| Lệnh 1 không in gì | Chưa nạp ROS: chạy `source /opt/ros/humble/setup.bash`. Nếu vẫn không được, làm lại README mục 4.1. |
| Lệnh 2 thiếu các gói `amr_` | Chưa build workspace: làm README mục 4.3. |
| Lệnh 2 thiếu `micro_ros_agent` | Chưa chạy `vcs import` ở mục 4.3. Có thể dùng Docker thay thế (xem ghi chú cuối mục 4.3). |
| Lệnh 3 báo `command not found` | Làm README mục 4.6. |

---

## Bước 1 — Xem xe FreeCAD của bạn trên RViz (chưa cần ESP32)

**Mục đích:** kiểm tra hình 3D bạn vẽ đã nằm đúng chỗ, **trước khi** dữ liệu thật đổ vào. Nếu
mô hình sai ngay từ đầu, mọi bước sau nhìn trên RViz đều khó hiểu.

**Hiểu trước:** ở chế độ `preview:=true`, chưa có ESP32 nên không ai gửi `/joint_states`. Launch
file sẽ mở thêm một cửa sổ có **thanh trượt** (joint_state_publisher_gui) để bạn tự giả lập góc
quay bánh. `use_meshes:=true` báo cho URDF dùng file STL từ FreeCAD thay vì hình hộp.

**Làm** (ở T2):

```bash
ros2 launch amr_description display.launch.py preview:=true use_meshes:=true
```

Hai cửa sổ sẽ hiện ra: RViz2 và một cửa sổ nhỏ có 2 thanh trượt `left_wheel_joint`,
`right_wheel_joint`.

**Kết quả đúng — kiểm tra 4 điều:**

1. Thấy xe của bạn đứng **trên** lưới (lưới là mặt sàn): bánh chạm lưới, thân không chìm.
2. Ở góc dưới trái khung RViz, **mục RobotModel có dấu tích xanh**, không có chữ đỏ.
3. Đầu xe hướng theo **trục X màu đỏ**. Bật hiển thị TF nếu cần: tích ô `TF` ở bảng *Displays*.
4. Kéo thanh trượt `left_wheel_joint`: bánh trái **quay tròn quanh tâm**, không đảo lắc. Kéo giá trị
   **dương**: mặt trên của bánh lăn **về phía trước** (về phía trục X đỏ).

**Cách xoay góc nhìn trong RViz:** giữ chuột trái và kéo để xoay, lăn chuột để zoom, giữ chuột giữa
(hoặc `Shift` + chuột trái) và kéo để dịch.

Xong thì nhấn `Ctrl+C` ở T2 để đóng.

**Nếu sai:** xem bảng lỗi thường gặp ở [ros2_ws/docs/FREECAD_URDF.md](../ros2_ws/docs/FREECAD_URDF.md)
mục 7 (xe quá to hoặc quá nhỏ, bánh lệch tâm, bánh nằm ngang, xe ngược hướng…). Chạy kiểm tra STL:

```bash
cd ~/esp32-4wd-amr-ros2-humble/ros2_ws/src/amr_description
```

```bash
python3 scripts/stl_tool.py check meshes/*.stl
```

---

## Bước 2 — Lấy IP laptop và khai báo Wi-Fi cho ESP32

**Mục đích:** cho ESP32 biết vào mạng Wi-Fi nào và gửi dữ liệu tới máy nào.

**Hiểu trước:**

* ESP32 **chỉ dùng được Wi-Fi 2.4 GHz**. Nếu router phát cả hai băng tần cùng tên, thường vẫn được;
  nếu không vào mạng được, hãy tạo một mạng 2.4 GHz riêng.
* ESP32 cần **IP của laptop** (gọi là `AGENT_IP`) vì Agent chạy trên laptop.
* Mật khẩu Wi-Fi để trong file `secrets_local.h`. File này đã được Git bỏ qua, nên không bị đẩy lên
  GitHub.

**Làm:**

1. Kết nối laptop vào **cùng mạng Wi-Fi** mà ESP32 sẽ dùng.
2. Ở T3, xem IP của laptop:

   ```bash
   hostname -I
   ```

   Lệnh in ra một hoặc vài địa chỉ. Lấy địa chỉ dạng `192.168.x.x` (hoặc `10.x.x.x`) **của mạng
   Wi-Fi đó**, ví dụ `192.168.1.100`. Bỏ qua các địa chỉ `172.17.x.x` (Docker).
3. Tạo file khai báo. Thay 3 giá trị trong ngoặc kép bằng thông tin **của bạn**:

   ```bash
   cd ~/esp32-4wd-amr-ros2-humble/firmware_esp32
   ```

   ```bash
   cat > include/secrets_local.h <<'EOF'
   #define WIFI_SSID     "TenWiFi_2.4GHz"
   #define WIFI_PASSWORD "MatKhauWiFi"
   #define AGENT_IP      "192.168.1.100"
   EOF
   ```

   Lệnh `cat > ... <<'EOF'` ghi mọi dòng phía sau vào file cho tới dòng `EOF`. Bạn cũng có thể mở file
   bằng trình soạn thảo: `gedit include/secrets_local.h`.
4. Mở cổng tường lửa để ESP32 gửi tới được laptop:

   ```bash
   sudo ufw allow 8888/udp
   ```

   ```bash
   sudo ufw allow 8889/udp
   ```

   8888 là cổng của micro-ROS Agent, 8889 là cổng LiDAR (dùng sau này).

**Kết quả đúng:** chạy `cat include/secrets_local.h` thấy đúng 3 dòng với thông tin của bạn.

**Nếu sai:** IP laptop có thể **đổi** mỗi khi kết nối lại Wi-Fi. Nếu một hôm ESP32 bỗng không kết nối
được, việc đầu tiên là chạy lại `hostname -I` và so với `AGENT_IP`. Tốt nhất hãy đặt IP cố định cho
laptop trong trang quản trị router (DHCP reservation).

---

## Bước 3 — Nạp firmware vào ESP32

**Mục đích:** đưa chương trình vào ESP32.

**Hiểu trước:**

* `pio run` **biên dịch** mã nguồn thành file chạy được. Lần đầu mất **5–10 phút** vì phải tải
  trình biên dịch và build thư viện micro-ROS. Các lần sau chỉ vài giây.
* `pio run -t upload` biên dịch rồi **nạp** qua cáp USB.
* Mỗi lần sửa `secrets_local.h` hoặc `robot_config.h`, bạn phải nạp lại thì ESP32 mới biết.

**Làm** (ở T1):

1. Cắm ESP32 vào laptop bằng cáp USB. Cáp phải là **cáp có dây dữ liệu**; cáp chỉ để sạc sẽ không
   nạp được.
2. Kiểm tra laptop đã nhận board:

   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```

   Phải thấy ví dụ `/dev/ttyUSB0`.
3. Biên dịch và nạp:

   ```bash
   cd ~/esp32-4wd-amr-ros2-humble/firmware_esp32
   ```

   ```bash
   pio run -t upload
   ```

**Kết quả đúng:** cuối cùng hiện `[SUCCESS]`. Trong quá trình nạp có các dòng
`Writing at 0x... (xx %)`.

**Nếu sai:**

| Hiện tượng | Nguyên nhân / cách sửa |
|---|---|
| Không có `/dev/ttyUSB0` | Cáp chỉ sạc; hoặc `brltty` chiếm cổng (`sudo apt remove brltty`, rút ra cắm lại). |
| `Permission denied: '/dev/ttyUSB0'` | Chưa vào nhóm `dialout` (README mục 4.6), **đăng xuất rồi đăng nhập lại**. |
| `Failed to connect to ESP32: ... Wrong boot mode` | Khi thấy `Connecting....`, **giữ nút BOOT** trên board cho tới khi bắt đầu `Writing`. |
| `could not open port ... busy` | Đang mở Serial Monitor ở terminal khác: nhấn `Ctrl+C` ở đó rồi nạp lại. |

---

## Bước 4 — Đọc ESP32 "nói" gì qua Serial Monitor

**Mục đích:** Serial Monitor là **cửa sổ quan trọng nhất** khi dò lỗi. ESP32 in ra trạng thái
Wi-Fi, kết nối, encoder… mỗi giây.

**Hiểu trước:** `pio device monitor` mở cổng USB và in mọi thứ ESP32 gửi lên. Khi đang mở monitor
thì **không nạp được** (cổng bận). Muốn nạp lại, nhấn `Ctrl+C` ở T1 trước.

**Làm** (ở T1):

```bash
pio device monitor
```

Nhấn nút **EN** (hoặc RST) trên board để ESP32 khởi động lại, và xem log từ đầu.

**Kết quả đúng:**

```
[     52][MAIN] ESP32 2WD AMR | micro-ROS Humble qua Wi-Fi UDP | LiDAR: YDLIDAR X2/X2L/S2
[   2310][UROS] Wi-Fi OK: IP 192.168.1.123, RSSI -52 dBm -> tim agent 192.168.1.100:8888
[   5400][LIDAR] Chua nhan duoc du lieu: kiem tra TX LiDAR -> GPIO16, nguon 5V, chon dung model
[   6000][STAT] wifi=OK rssi=-52 agent=-- link=-1ms state=FAILSAFE ...
```

Giải thích từng dòng:

* **`Wi-Fi OK: IP 192.168.1.123`** — ESP32 đã vào mạng. **Ghi lại IP này** (IP của ESP32, khác IP
  laptop). `RSSI -52 dBm` là độ mạnh sóng: càng gần 0 càng tốt, nên lớn hơn −70.
* **`Chua nhan duoc du lieu ... LiDAR`** — **bình thường**, vì bạn chưa gắn LiDAR. Bỏ qua.
* **`agent=--`** — chưa tìm thấy Agent. **Đúng**, vì bạn chưa chạy Agent (đó là Bước 5).
* **`state=FAILSAFE`** — motor đang bị khoá an toàn. **Đúng**: motor chỉ được mở khoá khi có kết
  nối **và** nhận lệnh lái mới.

**Đèn LED xanh trên board** (GPIO 2) cho biết trạng thái mà không cần nhìn log:

| LED | Ý nghĩa |
|---|---|
| Nháy **nhanh** (5 lần/giây) | Đang cố vào Wi-Fi |
| Nháy **chậm** (1 lần/giây) | Có Wi-Fi, đang chờ Agent ← **trạng thái bây giờ** |
| **Sáng liên tục** | Đã kết nối Agent |
| Nháy **kép** | Lỗi động cơ (FAULT), sẽ gặp ở Bước 8 |

Để T1 chạy nguyên như vậy.

**Nếu sai:** LED nháy nhanh mãi, không có dòng `Wi-Fi OK`:

* Sai tên Wi-Fi hoặc mật khẩu (phân biệt hoa thường), hoặc mạng 5 GHz. Sửa `secrets_local.h`,
  nhấn `Ctrl+C` ở T1, rồi chạy lại `pio run -t upload` và `pio device monitor`.
* Mạng "Khách/Guest" hoặc mạng có trang đăng nhập (quán cà phê, trường học) thường chặn thiết bị nói
  chuyện với nhau. Dùng router riêng, hoặc cho laptop phát Wi-Fi (README mục 4.5).

---

## Bước 5 — Bật Agent và RViz: ESP32 "bước vào" ROS 2

**Mục đích:** nối ESP32 vào ROS 2 và thấy xe 3D trên RViz được điều khiển bởi **dữ liệu thật** từ
ESP32.

**Hiểu trước:** lệnh `bringup.launch.py` khởi động cùng lúc 4 thứ:

1. **micro-ROS Agent** (UDP 8888): phiên dịch cho ESP32.
2. **lidar_bridge** (UDP 8889): nhận dữ liệu LiDAR. Chưa có LiDAR nên nó sẽ cảnh báo, bỏ qua.
3. **robot_state_publisher**: đọc URDF của bạn + `/joint_states` từ ESP32 để tính TF.
4. **RViz2**.

Khác với Bước 1: lần này **không có thanh trượt**. Góc quay bánh lấy từ encoder thật trên ESP32, vị
trí xe lấy từ `/odom` của ESP32.

**Làm** (ở T2):

```bash
ros2 launch amr_bringup bringup.launch.py use_meshes:=true
```

**Kết quả đúng:**

* Sau vài giây, T1 (Serial) in:
  ```
  [UROS] Da ket noi micro-ROS agent (node 'esp32_base', client key 0x...)
  [UROS] Dong bo thoi gian voi agent OK
  [STAT] wifi=OK rssi=-52 agent=ON link=8ms state=FAILSAFE ...
  ```
  `agent=ON` nghĩa là đã kết nối. `link=8ms` là thời gian kể từ gói tin gần nhất từ Agent, luôn
  dưới khoảng 100 ms là tốt.
* **LED sáng liên tục.**
* T2 in nhiều dòng, trong đó có dòng của Agent kiểu `session established` và cảnh báo
  `Chưa nhận được gói LiDAR nào trên UDP 8889`. Cảnh báo này **bình thường** khi chưa có LiDAR.
* RViz hiện xe của bạn, **RobotModel tích xanh**.

**Nếu sai:**

| Hiện tượng | Nguyên nhân / cách sửa |
|---|---|
| LED vẫn nháy chậm, `agent=--` mãi | `AGENT_IP` sai (so lại `hostname -I`); tường lửa chặn (Bước 2.4); laptop và ESP32 ở hai mạng khác nhau. |
| T2 báo `[LỖI] Không tìm thấy gói micro_ros_agent` | Chưa build Agent (README 4.3). |
| Kết nối được rồi lại mất liên tục | Tiết kiệm điện Wi-Fi trên laptop đang bật (README 4.5); ESP32 quá xa router. |
| RobotModel đỏ, báo `No transform from ...` | Chờ thêm vài giây. Nếu vẫn đỏ, ESP32 chưa gửi `/joint_states`; xem T1 đã `agent=ON` chưa. |

---

## Bước 6 — Dùng lệnh ROS 2 để "nghe" ESP32

**Mục đích:** biết dùng các lệnh kiểm tra cơ bản. Sau này mọi lỗi đều được dò bằng những lệnh này.

**Hiểu trước:** RViz chỉ là hình vẽ. Muốn xem **con số thật** trong topic, dùng các lệnh `ros2 topic`.

**Làm** (ở T3), từng lệnh một:

**6.1. Liệt kê các topic đang có**

```bash
ros2 topic list
```

Phải có `/odom`, `/joint_states`, `/tf`, `/cmd_vel`, `/robot_description`. Ba topic đầu do ESP32
gửi lên qua Agent; `/cmd_vel` là nơi ESP32 **nghe** lệnh lái.

**6.2. Đo tần số ESP32 gửi `/odom`**

```bash
ros2 topic hz /odom
```

Kết quả `average rate: 30.0` (khoảng 28–30 là đạt). Firmware gửi 30 lần/giây
(`ODOM_PUBLISH_HZ` trong `robot_config.h`). Nhấn `Ctrl+C` để dừng.

**6.3. Xem nội dung một tin `/joint_states`**

```bash
ros2 topic echo /joint_states --once
```

Thấy `name: [left_wheel_joint, right_wheel_joint]` và `position: [0.0, 0.0]`. Đây là góc quay
(radian) của hai bánh, hiện bằng 0 vì chưa có bánh nào quay. **Tên khớp ở đây phải trùng tên trong
URDF** thì RViz mới quay được bánh 3D. Repo đã đặt sẵn cho khớp.

**6.4. Xem cây toạ độ TF**

```bash
ros2 run tf2_tools view_frames
```

Lệnh chạy khoảng 5 giây rồi tạo file `frames_<ngày giờ>.pdf` trong thư mục hiện tại. Mở bằng
`xdg-open frames_*.pdf`. Phải thấy một cây liền mạch:

```
odom → base_footprint → base_link → left_wheel_link, right_wheel_link, caster_wheel_link, laser_frame
```

`odom → base_footprint` do **ESP32** gửi. Các nhánh còn lại do **robot_state_publisher** tính từ
URDF.

**6.5. Kiểm tra chất lượng Wi-Fi**

```bash
ping -c 20 192.168.1.123
```

Thay `192.168.1.123` bằng IP **của ESP32** (dòng `Wi-Fi OK` ở T1). Dòng cuối cho biết
`0% packet loss` và thời gian trung bình (`avg`): nên dưới 20 ms.

---

## Bước 7 — Giả lập encoder bằng 2 sợi dây: làm bánh 3D quay

**Mục đích:** chứng minh **cả đường đi** hoạt động: chân encoder → firmware → micro-ROS →
`/joint_states` → bánh 3D trong RViz. Khi lắp encoder thật, bạn sẽ biết chắc phần mềm không có lỗi.

**Hiểu trước — encoder hoạt động thế nào:**

Encoder của motor JGA25-370 có **2 dây tín hiệu A và B**. Khi bánh quay, A và B bật/tắt **lệch nhau
một phần tư chu kỳ**:

```
Quay tới:   A  ‾‾|__|‾‾|__        Firmware xem chân nào đổi TRƯỚC
            B  ‾|__|‾‾|__|‾       để biết chiều quay.
```

* ESP32 bật điện trở kéo lên bên trong, nên khi **không nối gì**, A = 1 và B = 1.
* Nối một chân xuống **GND** thì chân đó = 0.
* Firmware đếm **mọi** lần A hoặc B đổi mức (giải mã ×4). Mỗi lần đổi đúng thứ tự = **+1 tick**;
  ngược thứ tự = **−1 tick**.
* Một vòng bánh = 374 xung × 4 = **1496 tick**.

Vì vậy **chạm một dây** không làm bánh quay: chạm vào thì −1 tick, nhả ra thì +1 tick, tổng bằng 0.
Phải chạm **hai dây theo đúng thứ tự** như encoder thật.

**Chuẩn bị:** 2 dây cắm (jumper) đực–đực hoặc đực–cái. **Chỉ dùng chân GND**, **không bao giờ**
chạm vào chân 5V hoặc VIN.

| Bánh | Chân A | Chân B |
|---|---|---|
| Trái | **GPIO 18** (`D18`) | **GPIO 19** (`D19`) |
| Phải | **GPIO 23** (`D23`) | **GPIO 4** (`D4`) |

**Làm** (bánh trái):

1. Cắm đầu của **cả hai** dây vào hai chân **GND** của ESP32 (board DevKit có nhiều chân GND).
2. Ở T3, theo dõi góc bánh:

   ```bash
   ros2 topic echo /joint_states --field position
   ```

3. Dùng đầu còn lại của 2 dây, làm đúng **4 động tác theo thứ tự**:

   | Thứ tự | Động tác | Trạng thái (A,B) | Tick |
   |---|---|---|---|
   | 1 | Chạm dây 1 vào **D19 (B)** và giữ | (1,0) | +1 |
   | 2 | Chạm dây 2 vào **D18 (A)** và giữ | (0,0) | +2 |
   | 3 | **Nhả** dây ở D19 (B) | (0,1) | +3 |
   | 4 | **Nhả** dây ở D18 (A) | (1,1) | +4 |

   Dễ nhớ: **"B vào trước, B ra trước"**.

**Kết quả đúng:**

* Ở T1, dòng `[STAT] ... tick=(4,0)`: bánh trái tăng **4 tick** sau mỗi chu kỳ 4 động tác.
* Ở T3, số đầu tiên trong `position` tăng thêm khoảng **0.0168 rad** mỗi chu kỳ
  (= 4 × 2π / 1496, tức khoảng 1°).
* Làm **ngược thứ tự** (A vào trước, A ra trước) thì tick **giảm** 4.
* Làm nhiều chu kỳ (vài chục lần) sẽ thấy vạch vàng trên bánh trái trong RViz nhích dần. Mỗi chu kỳ
  chỉ khoảng 1°, nên phải làm nhiều mới nhìn thấy bằng mắt; con số ở T1 và T3 là bằng chứng chính
  xác hơn.

Làm tương tự với bánh phải (D4 là B, D23 là A). **Chiều đếm của bánh phải bị đảo** (tick giảm khi làm
"B vào trước"). Điều này là cố ý: hai motor lắp đối xứng gương nên firmware đảo chiều bên phải
(`ENCODER_RIGHT_INVERTED 1` trong `robot_config.h`). Chiều đúng cuối cùng sẽ được kiểm tra lại với
motor thật ở README Bước 2.

**Nếu thấy lạ:**

| Hiện tượng | Giải thích |
|---|---|
| Số tick nhảy lung tung vài đơn vị khi chạm | Tiếp điểm tay bị "dội" (nảy). Không sao: tổng cuối cùng sau khi nhả vẫn đúng. |
| `encErr=(1,0)` tăng | Bạn chạm/nhả **hai dây cùng lúc**, firmware không biết chiều nên bỏ qua và đếm lỗi. Làm chậm lại. Với encoder thật, `encErr` tăng nghĩa là nhiễu hoặc lỏng dây. |
| Không đổi gì | Kiểm tra lại đúng chân D18/D19 (đọc nhãn in trên board); đầu kia của dây phải ở GND. |

Ngoài ra, ở RViz xe cũng **trôi** rất nhẹ (vì `/odom` tính từ tick). Đây chính là odometry: ESP32 tin
rằng bánh trái lăn tới nên xe đã xoay nhẹ.

---

## Bước 8 — Gửi lệnh lái bằng bàn phím và xem cơ chế an toàn

**Mục đích:** kiểm tra ESP32 **nhận** được lệnh lái, và hiểu hai cơ chế an toàn **FAILSAFE** và
**FAULT** trước khi gắn motor thật.

**Hiểu trước:**

* `teleop_twist_keyboard` biến phím bấm thành tin nhắn `/cmd_vel` (vận tốc tới + vận tốc quay).
* ESP32 nhận `/cmd_vel`, tính tốc độ từng bánh, rồi dùng **PID** để tăng giảm công suất motor sao
  cho tốc độ **đo được từ encoder** bằng tốc độ mong muốn.
* Chưa có motor và encoder, tốc độ đo được luôn bằng 0. PID sẽ tăng công suất lên **tối đa**, mà
  bánh vẫn không quay. Sau **1 giây** như vậy, firmware kết luận "kẹt bánh hoặc mất encoder" và **cắt
  motor**: đó là trạng thái **FAULT**. Đây là tính năng bảo vệ: ngoài đời nó ngăn motor bị cháy khi
  kẹt.

**Làm:**

1. Ở T3 (nhấn `Ctrl+C` nếu lệnh echo ở Bước 7 còn chạy):

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.8
   ```

   Cửa sổ T3 phải **đang được chọn** (bấm chuột vào nó) thì phím mới có tác dụng.

2. **Nhấn `i` một lần.** Quan sát T1:
   ```
   [CTRL] Ket noi OK + co /cmd_vel moi -> cho phep chay
   [STAT] ... state=RUN cmd=(0.15,0.00) wL=0.00/4.62 ...
   ```
   * `cmd=(0.15,0.00)`: ESP32 đã nhận lệnh tiến 0,15 m/s.
   * `wL=0.00/4.62`: bánh trái **đo** 0 rad/s, **cần** 4,62 rad/s (= 0,15 m/s ÷ 0,0325 m).
   * Sau 0,5 giây không có phím mới, ESP32 tự dừng (`CMD_VEL_TIMEOUT_MS`), rồi về `state=IDLE`.

3. **Nhấn và GIỮ `i` khoảng 3 giây.** (Bàn phím chờ khoảng nửa giây rồi mới tự lặp phím; sau đó
   PID cần chút thời gian để đẩy công suất lên tối đa, rồi firmware chờ thêm 1 giây.) T1 in:
   ```
   [CTRL] FAULT: banh TRAI khong quay du duty=0.xx -> ket banh / mat encoder
   ```
   **LED nháy kép.** Đúng như dự đoán: không có motor nên bánh không quay.

4. **Nhấn `k`** (lệnh dừng = vận tốc 0). T1 in `Nhan /cmd_vel = 0 -> xoa loi dong co`, LED sáng
   liên tục trở lại.

5. **Thử FAILSAFE (mất kết nối):** ở T2 nhấn `Ctrl+C` để tắt Agent. Trong vòng khoảng 1,5 giây T1
   báo mất liên lạc và `state=FAILSAFE`, LED chuyển sang nháy chậm. Chạy lại lệnh ở Bước 5: ESP32
   **tự kết nối lại** sau vài giây mà không cần bấm reset.

   Sau khi kết nối lại, motor **vẫn bị khoá** cho tới khi có phím lái **mới**. Đây là cố ý: tránh xe
   tự lao đi bằng lệnh cũ ngay khi có sóng trở lại.

**Kết quả đúng:** nhìn thấy đủ các trạng thái `FAILSAFE → RUN → IDLE → FAULT → IDLE` như mô tả.

> Khi đã có motor thật mà vẫn bị FAULT, xem README Bước 2: nguyên nhân thường là nối ngược dây
> motor/encoder, hoặc quên cấp nguồn motor.

---

## Tắt hệ thống đúng cách

1. T3: nhấn `Ctrl+C` (thoát teleop).
2. T2: nhấn `Ctrl+C` (tắt Agent + RViz). Chờ vài giây cho tắt hẳn.
3. T1: nhấn `Ctrl+C` (đóng Serial Monitor).
4. Rút cáp USB của ESP32.

## Lần sau chạy lại (không cần nạp lại firmware)

ESP32 nhớ firmware kể cả khi rút điện. Chỉ cần:

1. Cắm ESP32 (USB hoặc sau này là pin), chờ **LED nháy chậm**.
2. T2: `ros2 launch amr_bringup bringup.launch.py use_meshes:=true`.
3. Chờ **LED sáng liên tục**, rồi làm việc.

Nếu IP laptop đổi, sửa `AGENT_IP` trong `secrets_local.h` và nạp lại (Bước 3).

---

## Bảng tự kiểm tra cuối giai đoạn 1

Đánh dấu khi đã làm được. Đủ cả 9 ô là phần mềm của bạn sẵn sàng cho phần cứng.

- [ ] Xe FreeCAD hiện đúng trên lưới, đầu xe theo trục X, bánh quay tròn (Bước 1)
- [ ] Nạp firmware `[SUCCESS]` (Bước 3)
- [ ] Serial in `Wi-Fi OK`, RSSI lớn hơn −70 dBm (Bước 4)
- [ ] LED sáng liên tục, Serial `agent=ON` (Bước 5)
- [ ] `ros2 topic hz /odom` khoảng 30 Hz (Bước 6)
- [ ] Cây TF liền mạch từ `odom` tới các bánh (Bước 6)
- [ ] Ping ESP32 dưới 20 ms, không mất gói (Bước 6)
- [ ] Chu kỳ 2 dây làm `tick` tăng/giảm đúng 4 (Bước 7)
- [ ] Thấy đủ `RUN`, `FAULT`, `FAILSAFE` và tự kết nối lại (Bước 8)

## Khi có thêm linh kiện

| Có thêm | Làm tiếp |
|---|---|
| Mạch Buck, TB6612FNG, 2 motor có encoder, pin LiPo 3S | README mục 2–3 (đấu dây an toàn), rồi **Bước 2** (lái bằng bàn phím, hiệu chỉnh odometry) |
| LiDAR | README **Bước 3** (SLAM vẽ bản đồ), rồi **Bước 4** (Nav2 tự dẫn đường) |

Khi gặp lỗi, gửi kèm **log ở T1 (Serial)** và **log ở T2 (launch)**: hai nơi này cho biết gần như
mọi nguyên nhân.
