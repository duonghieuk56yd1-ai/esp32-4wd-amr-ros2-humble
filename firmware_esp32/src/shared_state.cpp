#include "shared_state.h"

#include <Arduino.h>

const char* driveStateName(DriveState state) {
  switch (state) {
    case DriveState::kIdle:
      return "IDLE";
    case DriveState::kRunning:
      return "RUN";
    case DriveState::kFailsafe:
      return "FAILSAFE";
    case DriveState::kFault:
      return "FAULT";
  }
  return "?";
}

namespace shared {
namespace {

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

CmdVelSample g_cmd;
RobotStateSample g_state;
LidarStats g_lidar;

bool g_link_up = false;
uint32_t g_link_alive_ms = 0;
bool g_agent_connected = false;

bool g_time_valid = false;
int64_t g_time_offset_ns = 0;

}  // namespace

void init() {
  portENTER_CRITICAL(&g_mux);
  g_cmd = CmdVelSample{};
  g_state = RobotStateSample{};
  g_lidar = LidarStats{};
  g_link_up = false;
  g_link_alive_ms = 0;
  g_agent_connected = false;
  g_time_valid = false;
  g_time_offset_ns = 0;
  portEXIT_CRITICAL(&g_mux);
}

void setCmdVel(float linear, float angular) {
  const uint32_t now = millis();
  portENTER_CRITICAL(&g_mux);
  g_cmd.linear = linear;
  g_cmd.angular = angular;
  g_cmd.rx_ms = now;
  g_cmd.seq = (g_cmd.seq == UINT32_MAX) ? 1 : g_cmd.seq + 1;  // 0 dành cho "chưa có"
  portEXIT_CRITICAL(&g_mux);
}

CmdVelSample cmdVel() {
  portENTER_CRITICAL(&g_mux);
  const CmdVelSample copy = g_cmd;
  portEXIT_CRITICAL(&g_mux);
  return copy;
}

void setRobotState(const RobotStateSample& state) {
  portENTER_CRITICAL(&g_mux);
  g_state = state;
  portEXIT_CRITICAL(&g_mux);
}

RobotStateSample robotState() {
  portENTER_CRITICAL(&g_mux);
  const RobotStateSample copy = g_state;
  portEXIT_CRITICAL(&g_mux);
  return copy;
}

void markLinkAlive() {
  const uint32_t now = millis();
  portENTER_CRITICAL(&g_mux);
  g_link_up = true;
  g_link_alive_ms = now;
  portEXIT_CRITICAL(&g_mux);
}

void markLinkDown() {
  portENTER_CRITICAL(&g_mux);
  g_link_up = false;
  portEXIT_CRITICAL(&g_mux);
}

uint32_t msSinceLinkAlive() {
  const uint32_t now = millis();
  portENTER_CRITICAL(&g_mux);
  const bool up = g_link_up;
  const uint32_t last = g_link_alive_ms;
  portEXIT_CRITICAL(&g_mux);
  if (!up) return UINT32_MAX;
  const int32_t elapsed = static_cast<int32_t>(now - last);  // có dấu: last có thể mới hơn now
  return elapsed > 0 ? static_cast<uint32_t>(elapsed) : 0u;
}

void setAgentConnected(bool connected) {
  portENTER_CRITICAL(&g_mux);
  g_agent_connected = connected;
  portEXIT_CRITICAL(&g_mux);
}

bool agentConnected() {
  portENTER_CRITICAL(&g_mux);
  const bool connected = g_agent_connected;
  portEXIT_CRITICAL(&g_mux);
  return connected;
}

void setRosTimeOffsetNs(int64_t offset_ns) {
  portENTER_CRITICAL(&g_mux);
  g_time_offset_ns = offset_ns;
  g_time_valid = true;
  portEXIT_CRITICAL(&g_mux);
}

bool rosTimeValid() {
  portENTER_CRITICAL(&g_mux);
  const bool valid = g_time_valid;
  portEXIT_CRITICAL(&g_mux);
  return valid;
}

int64_t toRosTimeNs(int64_t esp_time_us, bool* synced) {
  portENTER_CRITICAL(&g_mux);
  const bool valid = g_time_valid;
  const int64_t offset = valid ? g_time_offset_ns : 0;
  portEXIT_CRITICAL(&g_mux);
  if (synced != nullptr) *synced = valid;
  return esp_time_us * 1000LL + offset;
}

void setLidarStats(const LidarStats& stats) {
  portENTER_CRITICAL(&g_mux);
  g_lidar = stats;
  portEXIT_CRITICAL(&g_mux);
}

LidarStats lidarStats() {
  portENTER_CRITICAL(&g_mux);
  const LidarStats copy = g_lidar;
  portEXIT_CRITICAL(&g_mux);
  return copy;
}

}  // namespace shared
