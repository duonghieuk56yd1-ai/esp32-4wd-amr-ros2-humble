// Log đơn giản qua Serial (USB). Mỗi lệnh in là một lần ghi -> an toàn khi
// nhiều task cùng in (HardwareSerial của Arduino-ESP32 có mutex nội bộ).
#pragma once

#include <Arduino.h>

#define APP_LOG(tag, fmt, ...) \
  Serial.printf("[%8lu][%s] " fmt "\r\n", static_cast<unsigned long>(millis()), tag, ##__VA_ARGS__)
