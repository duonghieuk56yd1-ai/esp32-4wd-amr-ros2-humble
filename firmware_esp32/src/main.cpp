// =============================================================================
//  main.cpp — Firmware ESP32 cho robot 2WD vi sai + LiDAR, ROS 2 Humble (micro-ROS)
//
//   Core 1 : control task  (CONTROL_LOOP_HZ: encoder -> PID -> PWM, odometry, failsafe)
//            micro-ROS task (/cmd_vel, /odom, /joint_states, /tf @ ODOM_PUBLISH_HZ)
//            loopTask       (LED trạng thái, in chẩn đoán ra Serial)
//   Core 0 : Wi-Fi/lwIP stack + LiDAR task (Serial2 -> gói UDP LIDAR_UDP_PORT)
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>

#include "app_log.h"
#include "control_task.h"
#include "lidar_task.h"
#include "microros_task.h"
#include "pinout.h"
#include "robot_config.h"
#include "secrets.h"
#include "shared_state.h"

namespace {

#if defined(LIDAR_YDLIDAR_X2)
constexpr const char* kLidarName = "YDLIDAR X2/X2L/S2";
#elif defined(LIDAR_CAMSENSE_X1)
constexpr const char* kLidarName = "Camsense X1/X2";
#else
constexpr const char* kLidarName = "?";
#endif

bool g_config_ok = false;

void ledWrite(bool on) {
#if PIN_STATUS_LED >= 0
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

// nháy nhanh: chờ Wi-Fi | nháy chậm: chờ agent | sáng: đã kết nối | nháy kép: lỗi
void updateStatusLed(uint32_t now) {
  bool on;
  if (!g_config_ok || WiFi.status() != WL_CONNECTED) {
    on = (now / 100) % 2 == 0;
  } else if (!shared::agentConnected()) {
    on = (now / 500) % 2 == 0;
  } else if (shared::robotState().state == DriveState::kFault) {
    const uint32_t phase = now % 1000;
    on = phase < 100 || (phase >= 200 && phase < 300);
  } else {
    on = true;
  }
  ledWrite(on);
}

[[maybe_unused]] void printStatus() {
  const RobotStateSample s = shared::robotState();
  const CmdVelSample c = shared::cmdVel();
  const LidarStats l = shared::lidarStats();
  const uint32_t link_ms = shared::msSinceLinkAlive();
  const bool wifi = WiFi.status() == WL_CONNECTED;

  Serial.printf(
      "[%8lu][STAT] wifi=%s rssi=%d agent=%s link=%ldms state=%s cmd=(%.2f,%.2f) "
      "wL=%.2f/%.2f wR=%.2f/%.2f rad/s duty=(%.2f,%.2f) tick=(%ld,%ld) encErr=(%lu,%lu)\r\n",
      static_cast<unsigned long>(millis()), wifi ? "OK" : "--", wifi ? static_cast<int>(WiFi.RSSI()) : 0,
      shared::agentConnected() ? "ON" : "--",
      link_ms == UINT32_MAX ? -1L : static_cast<long>(link_ms), driveStateName(s.state), c.linear,
      c.angular, s.wheel_vel_rad_s[0], s.wheel_target_rad_s[0], s.wheel_vel_rad_s[1],
      s.wheel_target_rad_s[1], s.duty[0], s.duty[1], static_cast<long>(s.ticks[0]),
      static_cast<long>(s.ticks[1]), static_cast<unsigned long>(s.encoder_errors[0]),
      static_cast<unsigned long>(s.encoder_errors[1]));
  Serial.printf(
      "[%8lu][STAT] odom=(%.3f m, %.3f m, %.1f deg) v=%.2f w=%.2f | lidar=%s %.1fHz valid=%u "
      "sent=%lu sendErr=%lu decErr=%lu | stack(c/u/l)=%u/%u/%u heap=%lu\r\n",
      static_cast<unsigned long>(millis()), s.x, s.y, s.yaw * 57.29578, s.linear, s.angular,
      l.receiving ? "OK" : "--", l.scan_hz, static_cast<unsigned>(l.last_valid),
      static_cast<unsigned long>(l.scans_sent), static_cast<unsigned long>(l.send_errors),
      static_cast<unsigned long>(l.decode_errors), control::stackHighWaterMark(),
      microros::stackHighWaterMark(), lidar::stackHighWaterMark(),
      static_cast<unsigned long>(ESP.getFreeHeap()));
}

}  // namespace

void setup() {
  // 1) Motor về trạng thái an toàn NGAY khi boot + gắn ngắt encoder trên Core 1
  control::init();

  // Bộ đệm TX: log không chặn các task thời gian thực (mặc định chỉ có FIFO 128 byte)
  Serial.setTxBufferSize(2048);
  Serial.begin(SERIAL_BAUD);
#if PIN_STATUS_LED >= 0
  pinMode(PIN_STATUS_LED, OUTPUT);
  ledWrite(false);
#endif
  shared::init();

  APP_LOG("MAIN", "ESP32 2WD AMR | micro-ROS Humble qua Wi-Fi UDP | LiDAR: %s", kLidarName);
  APP_LOG("MAIN", "r=%.4f m, L=%.3f m, %d tick/vong | PID kp=%.3f ki=%.3f kd=%.3f | %d Hz",
          WHEEL_RADIUS_M, WHEEL_SEPARATION_M, ENCODER_TICKS_PER_REV, PID_KP, PID_KI, PID_KD,
          CONTROL_LOOP_HZ);

  IPAddress agent_ip;
  g_config_ok = agent_ip.fromString(AGENT_IP);
  if (!g_config_ok) {
    APP_LOG("MAIN", "LOI: AGENT_IP \"%s\" khong hop le (include/secrets.h)", AGENT_IP);
  }
  if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0) {
    APP_LOG("MAIN", "CANH BAO: chua dat WIFI_SSID/WIFI_PASSWORD/AGENT_IP trong include/secrets.h");
  }

  // 2) Wi-Fi STA không chặn; tắt modem sleep (nếu bật, gói tới trễ tới ~100 ms)
  WiFi.persistent(false);  // không ghi flash mỗi lần kết nối
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  APP_LOG("MAIN", "Ket noi Wi-Fi \"%s\" | agent %s:%d | LiDAR UDP -> %s:%d", WIFI_SSID, AGENT_IP,
          AGENT_PORT, AGENT_IP, LIDAR_UDP_PORT);

  // 3) Các task FreeRTOS
  control::start();
  if (g_config_ok) {
    microros::start(agent_ip, AGENT_PORT);
    lidar::start(agent_ip, LIDAR_UDP_PORT);
  }
}

void loop() {
  const uint32_t now = millis();
  updateStatusLed(now);

#if STATUS_PRINT_PERIOD_MS > 0
  static uint32_t last_print_ms = 0;
  if (now - last_print_ms >= STATUS_PRINT_PERIOD_MS) {
    last_print_ms = now;
    printStatus();
  }
#endif

  // Dự phòng: một số lý do ngắt kết nối (AP khởi động lại, deauth...) không được
  // auto-reconnect của Arduino-ESP32 xử lý -> chủ động nối lại sau 15 s (dài hơn
  // thời gian một lần kết nối + DHCP bình thường để không cắt ngang nó).
  static uint32_t wifi_down_since_ms = 0;
  static bool wifi_down = false;
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifi_down) {
      wifi_down = true;
      wifi_down_since_ms = now;
    } else if (now - wifi_down_since_ms > 15000) {
      WiFi.reconnect();
      wifi_down_since_ms = now;
    }
  } else {
    wifi_down = false;
  }

  vTaskDelay(pdMS_TO_TICKS(20));
}
