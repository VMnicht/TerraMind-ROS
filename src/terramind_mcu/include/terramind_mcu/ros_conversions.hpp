#pragma once
#include "terramind_mcu/mcu_session.hpp"
#include "terramind_interfaces/msg/mcu_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
namespace terramind::mcu
{
// 原样映射板卡反馈；stamp 为 PC 收帧时间，不推断真实传感器采样时刻。
inline terramind_interfaces::msg::McuState to_ros(
  const protocol::State & s, uint16_t seq,
  const Session & session, bool simulated,
  const rclcpp::Time & stamp)
{
  terramind_interfaces::msg::McuState m;
  m.header.stamp = stamp;
  m.header.frame_id = "base_link";
  m.connection_id = session.connection_id;
  m.link_ready = session.ready;
  m.simulated = simulated;
  m.frame_seq = seq;
  auto & x = s.system;
  m.uptime_ms = x.uptime_ms;
  m.last_command_seq = x.last_command_seq;
  m.result = x.result;
  m.mode = x.mode;
  m.faults = x.faults;
  m.capabilities = x.capabilities;
  m.command_age_ms = x.command_age_ms;
  m.rx_error_count = x.rx_error_count;
  auto & c = m.chassis;
  c.linear_mps = s.chassis.linear;
  c.angular_radps = s.chassis.angular;
  c.left_target_rpm = s.chassis.left_target;
  c.right_target_rpm = s.chassis.right_target;
  c.left_actual_rpm = s.chassis.left_actual;
  c.right_actual_rpm = s.chassis.right_actual;
  auto spread = [](const protocol::Spreader & a, auto & b) {
      b.on = a.on;
      b.target_rpm = a.target;
      b.actual_rpm = a.actual;
      b.servo_set_angle_deg = a.servo_angle;
    };
  spread(s.left, m.left_spreader);
  spread(s.right, m.right_spreader);
  m.mower.on = s.mower.on;
  m.mower.throttle_set_percent = s.mower.value;
  m.lift.on = s.lift.on;
  m.lift.target_mm = s.lift.target;
  m.lift.actual_mm = s.lift.actual;
  m.lift.feedback_valid = s.lift.valid;
  m.sprayer.on = s.sprayer.on;
  m.sprayer.target_percent = s.sprayer.target;
  m.sprayer.actual_percent = s.sprayer.actual;
  m.sprayer.feedback_valid = s.sprayer.valid;
  return m;
}
// device 填入实际端口路径，便于面板显示自动识别结果。
inline diagnostic_msgs::msg::DiagnosticArray diagnostic(
  const rclcpp::Time & stamp,
  const std::string & name,
  const std::string & device, int level,
  const std::string & reason, uint64_t errors)
{
  diagnostic_msgs::msg::DiagnosticArray a;
  a.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus d;
  d.name = name;
  d.hardware_id = device;
  d.level = level;
  d.message = reason;
  diagnostic_msgs::msg::KeyValue k;
  k.key = "parser_errors";
  k.value = std::to_string(errors);
  d.values.push_back(k);
  a.status.push_back(d);
  return a;
}
}
