#include "lidar_task.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_timer.h>
#include <fcntl.h>
#include <lwip/sockets.h>

#include <cstring>

#include "amr_scan_packet.h"
#include "app_log.h"
#include "pinout.h"
#include "robot_config.h"
#include "shared_state.h"

// ---- Chọn driver LiDAR theo robot_config.h -------------------------------------------
#if defined(LIDAR_YDLIDAR_X2) && defined(LIDAR_CAMSENSE_X1)
#error "robot_config.h: chi duoc chon MOT loai LiDAR (LIDAR_YDLIDAR_X2 hoac LIDAR_CAMSENSE_X1)"
#elif defined(LIDAR_YDLIDAR_X2)
#include <LDS_YDLIDAR_X2_X2L.h>
#elif defined(LIDAR_CAMSENSE_X1)
#include <LDS_CAMSENSE_X1.h>
#else
#error "robot_config.h: chua chon LiDAR (LIDAR_YDLIDAR_X2 hoac LIDAR_CAMSENSE_X1)"
#endif

static_assert(LIDAR_SCAN_BINS >= 90 && LIDAR_SCAN_BINS <= amr::ScanAssembler::kMaxBins,
              "LIDAR_SCAN_BINS ngoai [90, 720]");
static_assert(amr::scanPacketSize(LIDAR_SCAN_BINS, LIDAR_SEND_INTENSITY != 0) <= amr::kMaxUdpPayload,
              "Goi LiDAR vuot 1472 byte (1 goi UDP): giam LIDAR_SCAN_BINS hoac tat LIDAR_SEND_INTENSITY");
static_assert(LIDAR_RANGE_MIN_M > 0.0f && LIDAR_RANGE_MAX_M > LIDAR_RANGE_MIN_M &&
                  LIDAR_RANGE_MAX_M <= 65.0f,
              "Gioi han khoang cach LiDAR khong hop le");

namespace lidar {
namespace {

#if defined(LIDAR_YDLIDAR_X2)
LDS_YDLIDAR_X2_X2L g_lidar;  // YDLIDAR X2 / X2L / S2
constexpr amr::LidarModel kModel = amr::LidarModel::kYdlidarX2;
#else
LDS_CAMSENSE_X1 g_lidar;     // Camsense X1 / X2
constexpr amr::LidarModel kModel = amr::LidarModel::kCamsenseX1;
#endif

constexpr bool kSendIntensity = LIDAR_SEND_INTENSITY != 0;
constexpr size_t kPacketBytes = amr::scanPacketSize(LIDAR_SCAN_BINS, kSendIntensity);
constexpr size_t kUartRxBufferBytes = 2048;  // ~170 ms dữ liệu ở 115200 baud
constexpr uint32_t kTaskStackBytes = 6144;
constexpr float kMinScanTimeS = 0.05f;  // vòng quét ngắn bất thường (> 20 Hz) -> bỏ
constexpr float kMaxScanTimeS = 1.0f;   // vòng quét dài bất thường (mất dữ liệu) -> bỏ
constexpr uint32_t kNoDataMs = 2000;

amr::ScanAssemblerConfig makeConfig() {
  amr::ScanAssemblerConfig cfg;
  cfg.bins = LIDAR_SCAN_BINS;
  cfg.range_min_m = LIDAR_RANGE_MIN_M;
  cfg.range_max_m = LIDAR_RANGE_MAX_M;
  cfg.yaw_offset_deg = LIDAR_YAW_OFFSET_DEG;
  cfg.clockwise = LIDAR_ANGLE_CLOCKWISE != 0;
  return cfg;
}

amr::ScanAssembler g_scan(makeConfig());
uint8_t g_tx[kPacketBytes];

int g_sock = -1;
sockaddr_in g_dest{};
TaskHandle_t g_task = nullptr;

LidarStats g_stats;
uint32_t g_seq = 0;
int64_t g_scan_start_us = 0;
bool g_have_scan_start = false;
uint32_t g_points_total = 0;
uint32_t g_last_point_ms = 0;

// Đọc UART theo khối thay vì từng byte (giảm chi phí khoá/driver mỗi byte).
uint8_t g_rx[256];
size_t g_rx_len = 0;
size_t g_rx_pos = 0;

// ---- Callback cho thư viện LDS -----------------------------------------------------
int serialRead() {
  if (g_rx_pos >= g_rx_len) {
    const int avail = LIDAR_SERIAL.available();
    if (avail <= 0) return -1;
    const size_t want = static_cast<size_t>(avail) < sizeof(g_rx) ? static_cast<size_t>(avail) : sizeof(g_rx);
    g_rx_len = LIDAR_SERIAL.read(g_rx, want);
    g_rx_pos = 0;
    if (g_rx_len == 0) return -1;
  }
  return g_rx[g_rx_pos++];
}

size_t serialWrite(const uint8_t* buffer, size_t length) {
  return LIDAR_SERIAL.write(buffer, length);
}

void motorPin(float value, LDS::lds_pin_t lds_pin) {
  const int pin = (lds_pin == LDS::LDS_MOTOR_EN_PIN) ? PIN_LIDAR_MOTOR_EN : PIN_LIDAR_MOTOR_PWM;
  if (pin < 0) return;  // chân không được nối

  if (value <= static_cast<float>(LDS::DIR_INPUT)) {  // cấu hình hướng chân
    if (value == static_cast<float>(LDS::DIR_OUTPUT_PWM)) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcAttachChannel(pin, LIDAR_PWM_FREQ_HZ, LIDAR_PWM_RES_BITS, LIDAR_PWM_CH);
#else
      ledcSetup(LIDAR_PWM_CH, LIDAR_PWM_FREQ_HZ, LIDAR_PWM_RES_BITS);
      ledcAttachPin(pin, LIDAR_PWM_CH);
#endif
    } else {
      pinMode(pin, value == static_cast<float>(LDS::DIR_INPUT) ? INPUT : OUTPUT);
    }
    return;
  }
  if (value < static_cast<float>(LDS::VALUE_PWM)) {  // mức logic cố định
    digitalWrite(pin, value == static_cast<float>(LDS::VALUE_HIGH) ? HIGH : LOW);
  } else {  // duty PWM 0..1
    const uint32_t duty = static_cast<uint32_t>(static_cast<float>((1u << LIDAR_PWM_RES_BITS) - 1u) * value);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(LIDAR_PWM_CH, duty);
#else
    ledcWrite(LIDAR_PWM_CH, duty);
#endif
  }
}

void onInfo(LDS::info_t code, String info) {
  APP_LOG("LIDAR", "%s: %s", g_lidar.infoCodeToString(code).c_str(), info.c_str());
}

void onError(LDS::result_t, String) {
  ++g_stats.decode_errors;  // gói hỏng / sai checksum: chỉ đếm, không in tràn Serial
}

// Đóng gói & gửi vòng quét vừa xong; `now_us` là lúc bắt đầu vòng mới.
void finishScan(int64_t now_us) {
  const int64_t start_us = g_scan_start_us;
  const bool had_start = g_have_scan_start;
  g_scan_start_us = now_us;
  g_have_scan_start = true;

  if (!had_start) {  // vòng đầu tiên bắt đầu giữa chừng -> thiếu tia, bỏ
    g_scan.clear();
    return;
  }
  const float scan_time_s = static_cast<float>(now_us - start_us) * 1e-6f;
  if (scan_time_s < kMinScanTimeS || scan_time_s > kMaxScanTimeS) {
    g_scan.clear();
    return;
  }

  amr::ScanMeta meta;
  meta.model = kModel;
  meta.seq = g_seq++;
  meta.stamp_ns = static_cast<uint64_t>(shared::toRosTimeNs(start_us, &meta.ros_time));
  meta.scan_time_s = scan_time_s;

  g_stats.scan_hz = 1.0f / scan_time_s;
  g_stats.last_valid = g_scan.validCount();

  const size_t len = amr::encodeScanPacket(g_scan, meta, kSendIntensity, g_tx, sizeof(g_tx));
  if (len > 0 && g_sock >= 0 && WiFi.isConnected()) {
    const int sent = sendto(g_sock, g_tx, len, 0, reinterpret_cast<const sockaddr*>(&g_dest),
                            sizeof(g_dest));
    if (sent == static_cast<int>(len)) {
      ++g_stats.scans_sent;
    } else {
      ++g_stats.send_errors;
    }
  }
  g_scan.clear();
}

void onScanPoint(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  // Thư viện LDS gắn cờ scan_completed vào điểm ĐẦU TIÊN của vòng mới.
  if (scan_completed) finishScan(esp_timer_get_time());
  g_scan.addPoint(angle_deg, distance_mm, quality);
  ++g_points_total;
  g_last_point_ms = millis();
}

void lidarTask(void*) {
  LIDAR_SERIAL.setRxBufferSize(kUartRxBufferBytes);  // phải gọi trước begin()
  const uint32_t baud = g_lidar.getSerialBaudRate();
  LIDAR_SERIAL.begin(baud, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);

  g_lidar.setScanPointCallback(onScanPoint);
  g_lidar.setSerialReadCallback(serialRead);
  g_lidar.setSerialWriteCallback(serialWrite);
  g_lidar.setMotorPinCallback(motorPin);
  g_lidar.setInfoCallback(onInfo);
  g_lidar.setErrorCallback(onError);
  g_lidar.init();
  const LDS::result_t result = g_lidar.start();

  g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (g_sock >= 0) {
    fcntl(g_sock, F_SETFL, O_NONBLOCK);  // không bao giờ chặn task LiDAR
  }
  APP_LOG("LIDAR", "%s @ %lu baud, start(): %s, %d tia/vong -> UDP %s:%u (%u byte/goi)%s",
          g_lidar.getModelName(), static_cast<unsigned long>(baud),
          g_lidar.resultCodeToString(result).c_str(), LIDAR_SCAN_BINS,
          IPAddress(g_dest.sin_addr.s_addr).toString().c_str(),
          static_cast<unsigned>(ntohs(g_dest.sin_port)), static_cast<unsigned>(kPacketBytes),
          g_sock >= 0 ? "" : " [LOI tao socket]");

  const uint32_t started_ms = millis();
  uint32_t last_stats_ms = 0;
  bool warned_no_data = false;
  for (;;) {
    g_lidar.loop();  // đọc hết dữ liệu UART đang có, gọi onScanPoint cho từng điểm

    const uint32_t now = millis();
    if (now - last_stats_ms >= 200) {
      last_stats_ms = now;
      const bool receiving = g_points_total > 0 && (now - g_last_point_ms) < kNoDataMs;
      if (g_stats.receiving && !receiving) {
        APP_LOG("LIDAR", "Mat du lieu LiDAR > %lu ms", static_cast<unsigned long>(kNoDataMs));
      }
      if (!receiving && !warned_no_data && g_points_total == 0 && now - started_ms > 3000) {
        APP_LOG("LIDAR", "Chua nhan duoc du lieu: kiem tra TX LiDAR -> GPIO%d, nguon 5V, chon dung model",
                PIN_LIDAR_RX);
        warned_no_data = true;
      }
      g_stats.receiving = receiving;
      shared::setLidarStats(g_stats);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

}  // namespace

void start(const IPAddress& dest_ip, uint16_t dest_port) {
  memset(&g_dest, 0, sizeof(g_dest));
  g_dest.sin_family = AF_INET;
  g_dest.sin_port = htons(dest_port);
  g_dest.sin_addr.s_addr = static_cast<uint32_t>(dest_ip);

  xTaskCreatePinnedToCore(lidarTask, "lidar", kTaskStackBytes, nullptr, LIDAR_TASK_PRIORITY,
                          &g_task, LIDAR_TASK_CORE);
}

unsigned stackHighWaterMark() {
  return g_task != nullptr ? static_cast<unsigned>(uxTaskGetStackHighWaterMark(g_task)) : 0u;
}

}  // namespace lidar
