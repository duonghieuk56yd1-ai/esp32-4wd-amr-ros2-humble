#include "control_task.h"

#include <Arduino.h>
#include <esp_timer.h>

#include <cmath>

#include "amr_diff_drive.h"
#include "amr_pid.h"
#include "amr_quadrature.h"
#include "app_log.h"
#include "encoders.h"
#include "motor_driver.h"
#include "pinout.h"
#include "robot_config.h"
#include "shared_state.h"

static_assert(CONTROL_LOOP_HZ >= 10 && CONTROL_LOOP_HZ <= 500, "CONTROL_LOOP_HZ ngoài [10, 500]");
static_assert(ODOM_PUBLISH_HZ > 0 && ODOM_PUBLISH_HZ <= CONTROL_LOOP_HZ,
              "ODOM_PUBLISH_HZ phải <= CONTROL_LOOP_HZ (odometry cập nhật ở CONTROL_LOOP_HZ)");
static_assert(WHEEL_RADIUS_M > 0.0f && WHEEL_SEPARATION_M > 0.0f && ENCODER_PPR > 0,
              "Thông số cơ khí không hợp lệ");
static_assert(WHEEL_VEL_FILTER_ALPHA > 0.0f && WHEEL_VEL_FILTER_ALPHA <= 1.0f,
              "WHEEL_VEL_FILTER_ALPHA phải trong (0, 1]");
static_assert(FAILSAFE_TIMEOUT_MS > 1000 / CONTROL_LOOP_HZ, "FAILSAFE_TIMEOUT_MS quá nhỏ");

namespace control {
namespace {

constexpr uint32_t kPeriodMs = 1000 / CONTROL_LOOP_HZ;
constexpr float kCmdEpsilon = 1e-4f;     // |lệnh| nhỏ hơn -> coi là lệnh dừng
constexpr float kMinWheelTarget = 0.1f;  // rad/s (~3 mm/s): nhỏ hơn -> 0, tránh motor ù/rung
constexpr float kStoppedSpeed = 0.3f;    // rad/s: bánh coi như đứng yên

// Phát hiện lỗi động cơ (DRIVE_FAULT_DETECTION)
constexpr float kFaultDuty = 0.9f * MOTOR_MAX_DUTY;               // duty coi là bão hoà
constexpr float kRunawaySpeed = 0.25f * MAX_WHEEL_SPEED_RAD_S;    // quay ngược nhanh hơn
constexpr uint32_t kRunawayMs = 300;
constexpr float kStallSpeed = 0.5f;                               // gần như không quay
constexpr uint32_t kStallMs = 1000;

constexpr uint32_t kTaskStackBytes = 6144;

MotorDriverTB6612 g_motors;
QuadratureEncoder g_encoders[2];
TaskHandle_t g_task = nullptr;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

[[maybe_unused]] const char* sideName(int i) { return i == 0 ? "TRAI" : "PHAI"; }

void cutMotors() {
#if FAILSAFE_BRAKE
  g_motors.brake();
#else
  g_motors.standby();
#endif
}

void controlTask(void*) {
  const amr::DiffDriveKinematics kin(WHEEL_RADIUS_M, WHEEL_SEPARATION_M);
  amr::Odometry2D odom;
  const amr::PidGains gains{PID_KP, PID_KI, PID_KD};
  const amr::PidLimits limits{MOTOR_MAX_DUTY, PID_INTEGRAL_LIMIT};
  amr::PidController pid[2] = {amr::PidController(gains, limits, PID_D_FILTER_TAU_S),
                               amr::PidController(gains, limits, PID_D_FILTER_TAU_S)};

  int32_t prev_ticks[2] = {g_encoders[0].count(), g_encoders[1].count()};
  int64_t total_ticks[2] = {0, 0};
  float vel[2] = {0.0f, 0.0f};
  uint32_t runaway_ms[2] = {0, 0};
  uint32_t stall_ms[2] = {0, 0};

  // Khởi động ở FAILSAFE: chỉ chạy khi đã có kết nối và nhận /cmd_vel đầu tiên.
  DriveState state = DriveState::kFailsafe;
  uint32_t failsafe_cmd_seq = 0;
  uint32_t last_cmd_seq = 0;
  RobotStateSample sample;

  cutMotors();
  int64_t prev_us = esp_timer_get_time();
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kPeriodMs));

    const int64_t now_us = esp_timer_get_time();
    const float dt = static_cast<float>(now_us - prev_us) * 1e-6f;
    prev_us = now_us;
    if (!(dt > 0.0f)) continue;
    [[maybe_unused]] const uint32_t dt_ms = static_cast<uint32_t>(dt * 1000.0f + 0.5f);
    const uint32_t now_ms = millis();

    // ---- 1. Encoder -> góc quay & vận tốc bánh (rad, rad/s) -------------------
    double d_rad[2];
    for (int i = 0; i < 2; ++i) {
      const int32_t ticks = g_encoders[i].count();
      const int32_t d = amr::quad::tickDelta(ticks, prev_ticks[i]);  // đúng cả khi tràn 32 bit
      prev_ticks[i] = ticks;
      total_ticks[i] += d;
      d_rad[i] = amr::quad::ticksToRadians(d, ENCODER_TICKS_PER_REV);
      const float raw_vel = static_cast<float>(d_rad[i]) / dt;
      vel[i] += WHEEL_VEL_FILTER_ALPHA * (raw_vel - vel[i]);
    }

    // ---- 2. Odometry (động học vi sai) ---------------------------------------
    odom.update(d_rad[0] * WHEEL_RADIUS_M, d_rad[1] * WHEEL_RADIUS_M, WHEEL_SEPARATION_M);
    amr::WheelPair measured;
    measured.left = vel[0];
    measured.right = vel[1];
    const amr::BodyTwist twist = kin.toBodyTwist(measured);

    // ---- 3. Lệnh /cmd_vel & failsafe -----------------------------------------
    const CmdVelSample cmd = shared::cmdVel();
    const bool new_cmd = cmd.seq != last_cmd_seq;
    last_cmd_seq = cmd.seq;
    const bool cmd_fresh =
        cmd.seq != 0 && (CMD_VEL_TIMEOUT_MS == 0 ||
                         static_cast<int32_t>(now_ms - cmd.rx_ms) <= CMD_VEL_TIMEOUT_MS);
    const bool cmd_is_zero = std::fabs(cmd.linear) < kCmdEpsilon && std::fabs(cmd.angular) < kCmdEpsilon;
    const bool link_ok = shared::msSinceLinkAlive() <= FAILSAFE_TIMEOUT_MS;

    if (!link_ok) {
      if (state == DriveState::kIdle || state == DriveState::kRunning) {
        state = DriveState::kFailsafe;
        cutMotors();  // ngắt ngay, trước khi in log
        APP_LOG("CTRL", "FAILSAFE: mat ket noi > %d ms -> ngat dong co", FAILSAFE_TIMEOUT_MS);
      }
      failsafe_cmd_seq = cmd.seq;  // chỉ chạy lại khi có lệnh MỚI sau khi kết nối lại
    } else if (state == DriveState::kFailsafe) {
      if (cmd.seq != failsafe_cmd_seq && cmd_fresh) {
        APP_LOG("CTRL", "Ket noi OK + co /cmd_vel moi -> cho phep chay");
        state = DriveState::kIdle;
      }
    } else if (state == DriveState::kFault && new_cmd && cmd_is_zero) {
      APP_LOG("CTRL", "Nhan /cmd_vel = 0 -> xoa loi dong co");
      state = DriveState::kIdle;
    }

    // ---- 4. Vận tốc đặt cho từng bánh -------------------------------------------
    const bool drive_enabled = state == DriveState::kIdle || state == DriveState::kRunning;
    amr::WheelPair target;
    if (drive_enabled && cmd_fresh) {
      const float v = clampf(cmd.linear, -MAX_LINEAR_VEL_MPS, MAX_LINEAR_VEL_MPS);
      const float w = clampf(cmd.angular, -MAX_ANGULAR_VEL_RAD_S, MAX_ANGULAR_VEL_RAD_S);
      target = amr::limitWheelSpeeds(kin.toWheelSpeeds(v, w), MAX_WHEEL_SPEED_RAD_S);
      if (std::fabs(target.left) < kMinWheelTarget) target.left = 0.0f;
      if (std::fabs(target.right) < kMinWheelTarget) target.right = 0.0f;
    }
    const float target_arr[2] = {target.left, target.right};

    // ---- 5. PID -> PWM ----------------------------------------------------------
    float duty[2] = {0.0f, 0.0f};
    if (!drive_enabled) {
      cutMotors();
      pid[0].reset();
      pid[1].reset();
      runaway_ms[0] = runaway_ms[1] = stall_ms[0] = stall_ms[1] = 0;
    } else {
      const bool target_zero =
          std::fabs(target.left) < kCmdEpsilon && std::fabs(target.right) < kCmdEpsilon;
      const bool stopped = std::fabs(vel[0]) < kStoppedSpeed && std::fabs(vel[1]) < kStoppedSpeed;
      if (target_zero && stopped) {
        // Đã dừng hẳn: xoá PID (không ù motor, không tích phân vô ích), duty = 0 -> phanh
        pid[0].reset();
        pid[1].reset();
        g_motors.setDuty(MotorDriverTB6612::kLeft, 0.0f);
        g_motors.setDuty(MotorDriverTB6612::kRight, 0.0f);
        runaway_ms[0] = runaway_ms[1] = stall_ms[0] = stall_ms[1] = 0;
        state = DriveState::kIdle;
      } else {
        for (int i = 0; i < 2; ++i) {
          duty[i] = pid[i].update(target_arr[i], vel[i], dt);
          g_motors.setDuty(static_cast<MotorDriverTB6612::Side>(i), duty[i]);
        }
        state = DriveState::kRunning;

#if DRIVE_FAULT_DETECTION
        for (int i = 0; i < 2 && state == DriveState::kRunning; ++i) {
          const bool saturated = std::fabs(duty[i]) >= kFaultDuty;
          const bool reversed = saturated && duty[i] * vel[i] < 0.0f && std::fabs(vel[i]) > kRunawaySpeed;
          const bool stalled = saturated && std::fabs(vel[i]) < kStallSpeed;
          runaway_ms[i] = reversed ? runaway_ms[i] + dt_ms : 0;
          stall_ms[i] = stalled ? stall_ms[i] + dt_ms : 0;
          if (runaway_ms[i] >= kRunawayMs) {
            state = DriveState::kFault;
            cutMotors();
            APP_LOG("CTRL",
                    "FAULT: banh %s quay NGUOC chieu lenh (duty=%.2f, w=%.1f rad/s) -> kiem tra "
                    "MOTOR_/ENCODER_*_INVERTED",
                    sideName(i), duty[i], vel[i]);
          } else if (stall_ms[i] >= kStallMs) {
            state = DriveState::kFault;
            cutMotors();
            APP_LOG("CTRL", "FAULT: banh %s khong quay du duty=%.2f -> ket banh / mat encoder",
                    sideName(i), duty[i]);
          }
        }
        if (state == DriveState::kFault) {
          cutMotors();
          pid[0].reset();
          pid[1].reset();
          duty[0] = duty[1] = 0.0f;
          runaway_ms[0] = runaway_ms[1] = stall_ms[0] = stall_ms[1] = 0;
        }
#endif
      }
    }

    // ---- 6. Chia sẻ trạng thái cho micro-ROS task ---------------------------------
    sample.stamp_us = now_us;
    ++sample.seq;
    sample.x = odom.x();
    sample.y = odom.y();
    sample.yaw = odom.yaw();
    sample.linear = twist.linear;
    sample.angular = twist.angular;
    for (int i = 0; i < 2; ++i) {
      sample.wheel_pos_rad[i] = amr::quad::ticksToRadians(total_ticks[i], ENCODER_TICKS_PER_REV);
      sample.wheel_vel_rad_s[i] = vel[i];
      sample.wheel_target_rad_s[i] = target_arr[i];
      sample.duty[i] = duty[i];
      sample.ticks[i] = prev_ticks[i];
      sample.encoder_errors[i] = g_encoders[i].errorCount();
    }
    sample.state = state;
    shared::setRobotState(sample);
  }
}

}  // namespace

void init() {
  g_motors.begin();  // motor về trạng thái an toàn trước mọi thứ khác
  g_encoders[0].begin(PIN_ENC_L_A, PIN_ENC_L_B, ENCODER_LEFT_INVERTED != 0);
  g_encoders[1].begin(PIN_ENC_R_A, PIN_ENC_R_B, ENCODER_RIGHT_INVERTED != 0);
}

void start() {
  xTaskCreatePinnedToCore(controlTask, "control", kTaskStackBytes, nullptr, CONTROL_TASK_PRIORITY,
                          &g_task, CONTROL_TASK_CORE);
}

unsigned stackHighWaterMark() {
  return g_task != nullptr ? static_cast<unsigned>(uxTaskGetStackHighWaterMark(g_task)) : 0u;
}

}  // namespace control
