#pragma once
#include "terramind_control/command_guard.hpp"
#include "terramind_interfaces/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
namespace terramind::control
{
inline bool fresh_stamp(
  const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now,
  double timeout)
{
  // ROS 时间只用于判断消息年龄，允许最多 20 ms 的超前误差。
  double t = double(stamp.sec) + stamp.nanosec * 1e-9;
  double age = now.seconds() - t;
  return t > 0 && age >= -0.020 && age <= timeout;
}
inline double source_time_on_steady_clock(
  const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now)
{
  // 将源时间映射到单调时钟；接收旧消息不能重新获得完整的超时预算。
  const double source = double(stamp.sec) + stamp.nanosec * 1e-9;
  return steady_seconds() - std::max(0.0, now.seconds() - source);
}
inline protocol::Control from_ros(const terramind_interfaces::msg::ControlCommand & m)
{
  const auto & a = m.implements;
  return {m.enable, m.stop, m.linear_mps, m.angular_radps, {a.left_on, a.left_rpm},
    {a.right_on, a.right_rpm},
    {a.mower_on, a.mower_percent}, {a.lift_on, a.lift_mm}, {a.sprayer_on, a.sprayer_percent}};
}
inline void fill_ros(const protocol::Control & c, terramind_interfaces::msg::ControlCommand & m)
{
  // 外层与作业快照共用时间戳，避免底盘和作业字段表示不同时间的目标。
  m.enable = c.enable;
  m.stop = c.stop;
  m.linear_mps = c.linear;
  m.angular_radps = c.angular;
  auto & a = m.implements;
  a.header = m.header;
  a.left_on = c.left.on;
  a.left_rpm = c.left.value;
  a.right_on = c.right.on;
  a.right_rpm = c.right.value;
  a.mower_on = c.mower.on;
  a.mower_percent = c.mower.value;
  a.lift_on = c.lift.on;
  a.lift_mm = c.lift.value;
  a.sprayer_on = c.sprayer.on;
  a.sprayer_percent = c.sprayer.value;
}
inline bool any_on(const protocol::Control & c)
{
  return c.left.on || c.right.on || c.mower.on || c.lift.on || c.sprayer.on;
}
}
