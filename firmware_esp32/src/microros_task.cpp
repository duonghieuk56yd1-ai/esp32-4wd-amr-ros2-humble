#include "microros_task.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_timer.h>

#include <cmath>
#include <cstring>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <tf2_msgs/msg/tf_message.h>

#include "app_log.h"
#include "robot_config.h"
#include "shared_state.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#error "Can board_microros_transport = wifi trong platformio.ini"
#endif

namespace microros {
namespace {

inline void ignoreResult(rcl_ret_t) {}  // (void) không tắt được warn_unused_result của GCC

// ---- Thời gian / timeout -----------------------------------------------------
constexpr int64_t kPublishPeriodUs = 1000000LL / ODOM_PUBLISH_HZ;
constexpr int kReliablePublishTimeoutMs = 50;  // chờ agent xác nhận gói reliable (mặc định 1000)
constexpr uint32_t kPingPeriodMs = 100;        // ping khi không có bằng chứng liên lạc gần đây
constexpr int kPingTimeoutMs = 50;
constexpr uint32_t kWaitAgentPingPeriodMs = 500;
constexpr int kWaitAgentPingTimeoutMs = 100;
constexpr uint32_t kAgentLostMs = 1500;        // không liên lạc được -> huỷ & tạo lại phiên
constexpr uint32_t kSessionDeadMs = 2500;      // không gói reliable nào được xác nhận
constexpr int kTimeSyncTimeoutMs = 100;
constexpr uint32_t kTimeSyncRetryMs = 250;
constexpr uint32_t kTaskStackBytes = 12288;

enum class AgentState : uint8_t { kWaitingWifi, kWaitingAgent, kCreating, kConnected, kDestroying };

struct Entities {
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rcl_publisher_t odom_pub;
  rcl_publisher_t joint_pub;
  rcl_publisher_t tf_pub;
  rcl_subscription_t cmd_sub;
  rclc_executor_t executor;
  bool support_ok;
  bool node_ok;
  bool odom_ok;
  bool joint_ok;
  bool tf_ok;
  bool cmd_ok;
  bool executor_ok;
};

Entities g_e{};
micro_ros_agent_locator g_locator;
TaskHandle_t g_task = nullptr;
uint32_t g_client_key = 0;

// ---- Message cấp phát tĩnh (không malloc lúc chạy) -----------------------------
nav_msgs__msg__Odometry g_odom_msg;
sensor_msgs__msg__JointState g_joint_msg;
tf2_msgs__msg__TFMessage g_tf_msg;
geometry_msgs__msg__TransformStamped g_tf_transform;
geometry_msgs__msg__Twist g_cmd_msg;

char g_odom_frame[] = ODOM_FRAME_ID;
char g_base_frame[] = BASE_FRAME_ID;
char g_empty_str[] = "";
char g_left_joint[] = LEFT_WHEEL_JOINT_NAME;
char g_right_joint[] = RIGHT_WHEEL_JOINT_NAME;
rosidl_runtime_c__String g_joint_names[2];
double g_joint_position[2];
double g_joint_velocity[2];

void bindString(rosidl_runtime_c__String& s, char* buffer) {
  s.data = buffer;
  s.size = strlen(buffer);
  s.capacity = s.size + 1;
}

void initMessages() {
  // /odom
  bindString(g_odom_msg.header.frame_id, g_odom_frame);
  bindString(g_odom_msg.child_frame_id, g_base_frame);
  double* pc = g_odom_msg.pose.covariance;
  double* tc = g_odom_msg.twist.covariance;
  for (int i = 0; i < 36; ++i) pc[i] = tc[i] = 0.0;
  pc[0] = pc[7] = ODOM_COV_POSE_XY;            // x, y
  pc[14] = pc[21] = pc[28] = ODOM_COV_UNUSED;  // z, roll, pitch
  pc[35] = ODOM_COV_POSE_YAW;                  // yaw
  tc[0] = tc[7] = ODOM_COV_TWIST_VX;           // vx, vy (vy ~ 0 với robot vi sai)
  tc[14] = tc[21] = tc[28] = ODOM_COV_UNUSED;  // vz, wx, wy
  tc[35] = ODOM_COV_TWIST_WZ;                  // wz

  // /joint_states
  bindString(g_joint_msg.header.frame_id, g_empty_str);
  bindString(g_joint_names[0], g_left_joint);
  bindString(g_joint_names[1], g_right_joint);
  g_joint_msg.name.data = g_joint_names;
  g_joint_msg.name.size = g_joint_msg.name.capacity = 2;
  g_joint_msg.position.data = g_joint_position;
  g_joint_msg.position.size = g_joint_msg.position.capacity = 2;
  g_joint_msg.velocity.data = g_joint_velocity;
  g_joint_msg.velocity.size = g_joint_msg.velocity.capacity = 2;
  g_joint_msg.effort.data = nullptr;
  g_joint_msg.effort.size = g_joint_msg.effort.capacity = 0;

  // /tf: odom -> base_footprint
  bindString(g_tf_transform.header.frame_id, g_odom_frame);
  bindString(g_tf_transform.child_frame_id, g_base_frame);
  g_tf_msg.transforms.data = &g_tf_transform;
  g_tf_msg.transforms.size = g_tf_msg.transforms.capacity = 1;
}

void setStamp(builtin_interfaces__msg__Time& t, int64_t ns) {
  if (ns < 0) ns = 0;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

// ---- Callback /cmd_vel ------------------------------------------------------------
void onCmdVel(const void* msg_in) {
  const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  const float v = static_cast<float>(msg->linear.x);
  const float w = static_cast<float>(msg->angular.z);
  if (!std::isfinite(v) || !std::isfinite(w)) return;
  shared::setCmdVel(v, w);
  shared::markLinkAlive();
}

// ---- Tạo / huỷ entity ---------------------------------------------------------------
void destroyEntities() {
  if (g_e.support_ok) {
    // Không chờ agent trả lời khi huỷ (agent có thể đã mất)
    rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&g_e.support.context);
    ignoreResult(rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0));
  }
  if (g_e.executor_ok) ignoreResult(rclc_executor_fini(&g_e.executor));
  if (g_e.cmd_ok) ignoreResult(rcl_subscription_fini(&g_e.cmd_sub, &g_e.node));
  if (g_e.tf_ok) ignoreResult(rcl_publisher_fini(&g_e.tf_pub, &g_e.node));
  if (g_e.joint_ok) ignoreResult(rcl_publisher_fini(&g_e.joint_pub, &g_e.node));
  if (g_e.odom_ok) ignoreResult(rcl_publisher_fini(&g_e.odom_pub, &g_e.node));
  if (g_e.node_ok) ignoreResult(rcl_node_fini(&g_e.node));
  if (g_e.support_ok) ignoreResult(rclc_support_fini(&g_e.support));
  g_e = Entities{};
}

bool createEntities() {
  g_e = Entities{};
  g_e.allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, g_e.allocator) != RCL_RET_OK) return false;
  ignoreResult(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  // Client key cố định theo MAC: agent thay phiên cũ khi kết nối lại thay vì
  // giữ node "ma" trùng tên.
  ignoreResult(rmw_uros_options_set_client_key(
      g_client_key, rcl_init_options_get_rmw_init_options(&init_options)));
  const rcl_ret_t rc =
      rclc_support_init_with_options(&g_e.support, 0, nullptr, &init_options, &g_e.allocator);
  ignoreResult(rcl_init_options_fini(&init_options));  // rcl_init đã sao chép options
  g_e.support_ok = rcl_context_is_valid(&g_e.support.context);
  if (rc != RCL_RET_OK) return false;

  g_e.node_ok = rclc_node_init_default(&g_e.node, ROS_NODE_NAME, ROS_NAMESPACE, &g_e.support) ==
                RCL_RET_OK;
  if (!g_e.node_ok) return false;

  // /odom reliable: tương thích subscriber mặc định (Nav2, robot_localization)
  g_e.odom_ok = rclc_publisher_init_default(&g_e.odom_pub, &g_e.node,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                            TOPIC_ODOM) == RCL_RET_OK;
  if (!g_e.odom_ok) return false;

  // /joint_states best effort: robot_state_publisher dùng SensorDataQoS
  g_e.joint_ok = rclc_publisher_init_best_effort(
                     &g_e.joint_pub, &g_e.node,
                     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                     TOPIC_JOINT_STATES) == RCL_RET_OK;
  if (!g_e.joint_ok) return false;

#if PUBLISH_ODOM_TF
  // /tf reliable: tf2_ros::TransformListener chỉ nhận publisher reliable
  g_e.tf_ok = rclc_publisher_init_default(&g_e.tf_pub, &g_e.node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
                                          TOPIC_TF) == RCL_RET_OK;
  if (!g_e.tf_ok) return false;
#endif

  // /cmd_vel best effort: nhận được từ mọi publisher (reliable lẫn best effort)
  // và không bị agent gửi bù lệnh cũ sau khi Wi-Fi chập chờn.
  g_e.cmd_ok = rclc_subscription_init_best_effort(
                   &g_e.cmd_sub, &g_e.node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                   TOPIC_CMD_VEL) == RCL_RET_OK;
  if (!g_e.cmd_ok) return false;

  g_e.executor = rclc_executor_get_zero_initialized_executor();
  g_e.executor_ok =
      rclc_executor_init(&g_e.executor, &g_e.support.context, 1, &g_e.allocator) == RCL_RET_OK;
  if (!g_e.executor_ok) return false;
  if (rclc_executor_add_subscription(&g_e.executor, &g_e.cmd_sub, &g_cmd_msg, &onCmdVel,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  // rcl_publish() reliable chặn tới khi agent xác nhận: giới hạn thời gian chờ
  ignoreResult(rmw_uros_set_publisher_session_timeout(
      rcl_publisher_get_rmw_handle(&g_e.odom_pub), kReliablePublishTimeoutMs));
#if PUBLISH_ODOM_TF
  ignoreResult(rmw_uros_set_publisher_session_timeout(
      rcl_publisher_get_rmw_handle(&g_e.tf_pub), kReliablePublishTimeoutMs));
#endif
  return true;
}

// ---- Đồng bộ thời gian với agent --------------------------------------------------
bool syncTime() {
  if (rmw_uros_sync_session(kTimeSyncTimeoutMs) != RMW_RET_OK || !rmw_uros_epoch_synchronized()) {
    return false;
  }
  const int64_t esp_us = esp_timer_get_time();
  const int64_t ros_ns = rmw_uros_epoch_nanos();
  shared::setRosTimeOffsetNs(ros_ns - esp_us * 1000LL);
  return true;
}

// ---- Publish /odom, /joint_states, /tf ----------------------------------------------
// Trả về true nếu mọi gói reliable được agent xác nhận.
bool publishState(const RobotStateSample& s) {
  const int64_t stamp_ns = shared::toRosTimeNs(s.stamp_us);
  const double qz = std::sin(0.5 * s.yaw);
  const double qw = std::cos(0.5 * s.yaw);

  setStamp(g_odom_msg.header.stamp, stamp_ns);
  geometry_msgs__msg__Pose& pose = g_odom_msg.pose.pose;
  pose.position.x = s.x;
  pose.position.y = s.y;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  g_odom_msg.twist.twist.linear.x = s.linear;
  g_odom_msg.twist.twist.angular.z = s.angular;
  bool ok = rcl_publish(&g_e.odom_pub, &g_odom_msg, nullptr) == RCL_RET_OK;

  setStamp(g_joint_msg.header.stamp, stamp_ns);
  g_joint_position[0] = s.wheel_pos_rad[0];
  g_joint_position[1] = s.wheel_pos_rad[1];
  g_joint_velocity[0] = s.wheel_vel_rad_s[0];
  g_joint_velocity[1] = s.wheel_vel_rad_s[1];
  ignoreResult(rcl_publish(&g_e.joint_pub, &g_joint_msg, nullptr));  // best effort: không chờ

#if PUBLISH_ODOM_TF
  setStamp(g_tf_transform.header.stamp, stamp_ns);
  g_tf_transform.transform.translation.x = s.x;
  g_tf_transform.transform.translation.y = s.y;
  g_tf_transform.transform.translation.z = 0.0;
  g_tf_transform.transform.rotation.x = 0.0;
  g_tf_transform.transform.rotation.y = 0.0;
  g_tf_transform.transform.rotation.z = qz;
  g_tf_transform.transform.rotation.w = qw;
  ok = (rcl_publish(&g_e.tf_pub, &g_tf_msg, nullptr) == RCL_RET_OK) && ok;
#endif
  return ok;
}

// ---- Task chính ---------------------------------------------------------------------
void microrosTask(void*) {
  initMessages();
  ignoreResult(rmw_uros_set_custom_transport(false, &g_locator, platformio_transport_open,
                                             platformio_transport_close,
                                             platformio_transport_write, platformio_transport_read));

  AgentState state = AgentState::kWaitingWifi;
  uint32_t last_ping_ms = 0;
  uint32_t last_sync_ms = 0;
  uint32_t last_pub_ok_ms = 0;
  uint32_t last_pub_seq = 0;
  bool synced_this_session = false;
  int64_t next_pub_us = 0;

  for (;;) {
    const uint32_t now_ms = millis();
    const bool wifi_ok = WiFi.status() == WL_CONNECTED;

    if (!wifi_ok && state != AgentState::kWaitingWifi) {
      APP_LOG("UROS", "Mat Wi-Fi -> cho ket noi lai");
      destroyEntities();
      shared::setAgentConnected(false);
      shared::markLinkDown();
      state = AgentState::kWaitingWifi;
    }

    switch (state) {
      case AgentState::kWaitingWifi:
        if (wifi_ok) {
          APP_LOG("UROS", "Wi-Fi OK: IP %s, RSSI %d dBm -> tim agent %s:%d",
                  WiFi.localIP().toString().c_str(), static_cast<int>(WiFi.RSSI()),
                  g_locator.address.toString().c_str(), g_locator.port);
          state = AgentState::kWaitingAgent;
          last_ping_ms = now_ms - kWaitAgentPingPeriodMs;
        } else {
          vTaskDelay(pdMS_TO_TICKS(50));
        }
        break;

      case AgentState::kWaitingAgent:
        if (now_ms - last_ping_ms >= kWaitAgentPingPeriodMs) {
          last_ping_ms = now_ms;
          if (rmw_uros_ping_agent(kWaitAgentPingTimeoutMs, 1) == RMW_RET_OK) {
            state = AgentState::kCreating;
            break;
          }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        break;

      case AgentState::kCreating:
        if (createEntities()) {
          APP_LOG("UROS", "Da ket noi micro-ROS agent (node '%s', client key 0x%08lX)",
                  ROS_NODE_NAME, static_cast<unsigned long>(g_client_key));
          shared::setAgentConnected(true);
          shared::markLinkAlive();
          state = AgentState::kConnected;
          last_pub_ok_ms = millis();
          last_ping_ms = last_pub_ok_ms;
          last_sync_ms = last_pub_ok_ms - kTimeSyncRetryMs;
          synced_this_session = false;
          next_pub_us = esp_timer_get_time();
        } else {
          APP_LOG("UROS", "Tao entity that bai -> thu lai");
          destroyEntities();
          state = AgentState::kWaitingAgent;
          last_ping_ms = millis();
        }
        break;

      case AgentState::kConnected: {
        // (a) Đồng bộ đồng hồ: thử lại nhanh tới khi thành công, sau đó định kỳ
        const uint32_t sync_period = synced_this_session ? TIME_SYNC_PERIOD_MS : kTimeSyncRetryMs;
        if (now_ms - last_sync_ms >= sync_period) {
          last_sync_ms = now_ms;
          if (syncTime()) {
            if (!synced_this_session) APP_LOG("UROS", "Dong bo thoi gian voi agent OK");
            synced_this_session = true;
            shared::markLinkAlive();
          }
        }

        // (b) Nhận /cmd_vel tới hạn publish kế tiếp. Timeout >= 1 ms: với 0 ms
        //     transport của micro_ros_platformio không đọc gói UDP nào.
        int64_t wait_ms = (next_pub_us - esp_timer_get_time()) / 1000;
        if (wait_ms < 1) wait_ms = 1;
        if (wait_ms > 20) wait_ms = 20;
        ignoreResult(rclc_executor_spin_some(&g_e.executor, RCL_MS_TO_NS(wait_ms)));

        // (c) Publish ODOM_PUBLISH_HZ (cần đồng hồ đã đồng bộ để stamp đúng)
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_pub_us) {
          next_pub_us += kPublishPeriodUs;
          if (next_pub_us <= now_us) next_pub_us = now_us + kPublishPeriodUs;  // trễ: bỏ nhịp
          const RobotStateSample s = shared::robotState();
          // Không gửi lại cùng một mẫu (tf2 cảnh báo TF_REPEATED_DATA)
          if (shared::rosTimeValid() && s.seq != 0 && s.seq != last_pub_seq) {
            last_pub_seq = s.seq;
            if (publishState(s)) {
              last_pub_ok_ms = millis();
              shared::markLinkAlive();
            }
          }
        }

        // (d) Nhịp tim: ping agent khi lâu không có bằng chứng liên lạc
        const uint32_t t = millis();
        if (shared::msSinceLinkAlive() >= kPingPeriodMs && t - last_ping_ms >= kPingPeriodMs) {
          last_ping_ms = t;
          if (rmw_uros_ping_agent(kPingTimeoutMs, 1) == RMW_RET_OK) shared::markLinkAlive();
        }

        // (e) Mất agent, hoặc agent khởi động lại (ping được nhưng phiên cũ đã chết)
        const uint32_t since_alive = shared::msSinceLinkAlive();
        if (since_alive > kAgentLostMs || millis() - last_pub_ok_ms > kSessionDeadMs) {
          APP_LOG("UROS", "Mat lien lac voi agent -> tao lai phien");
          state = AgentState::kDestroying;
        }
        break;
      }

      case AgentState::kDestroying:
        destroyEntities();
        shared::setAgentConnected(false);
        shared::markLinkDown();
        state = AgentState::kWaitingAgent;
        last_ping_ms = millis();
        break;
    }
  }
}

}  // namespace

void start(const IPAddress& agent_ip, uint16_t agent_port) {
  g_locator.address = agent_ip;
  g_locator.port = agent_port;

  const uint64_t mac = ESP.getEfuseMac();
  g_client_key = static_cast<uint32_t>(mac ^ (mac >> 32));
  if (g_client_key == 0) g_client_key = 0x0A5A5A5Au;

  xTaskCreatePinnedToCore(microrosTask, "microros", kTaskStackBytes, nullptr,
                          MICROROS_TASK_PRIORITY, &g_task, MICROROS_TASK_CORE);
}

unsigned stackHighWaterMark() {
  return g_task != nullptr ? static_cast<unsigned>(uxTaskGetStackHighWaterMark(g_task)) : 0u;
}

}  // namespace microros
