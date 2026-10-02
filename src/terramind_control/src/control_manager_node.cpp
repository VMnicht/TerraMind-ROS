#include "terramind_control/ros_helpers.hpp"
#include "terramind_interfaces/msg/mcu_state.hpp"
#include "terramind_interfaces/msg/control_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cmath>
namespace ti = terramind_interfaces::msg;
namespace tc = terramind::control;
namespace tp = terramind::protocol;
// 完整控制快照的唯一发布者。单线程 executor 串行处理订阅、服务和定时器。
class ControlManager : public rclcpp::Node
{
public:
  ControlManager()
  : Node("control_manager")
  {
    timeout_ = declare_parameter("command_timeout_s", 0.150);
    state_timeout_ = declare_parameter(
      "state_timeout_s", 0.150);
    if (!(timeout_ > 0 && timeout_ < 0.250 && state_timeout_ > 0 && state_timeout_ < 0.250)) {
      throw std::invalid_argument("timeouts must be below 250 ms");
    }
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("MVP uses wall time; use_sim_time must be false");
    }
    auto q = rclcpp::QoS(1).reliable().durability_volatile();
    // 只保留最新目标，禁止将历史运动指令排队重放或持久化给新订阅者。
    commands_ = create_publisher<ti::ControlCommand>("mcu/command", q);
    status_ = create_publisher<ti::ControlStatus>("control/status", q);
    velocity_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", q, [this](const geometry_msgs::msg::TwistStamped & m) {
        if (!m.header.frame_id.empty() && m.header.frame_id != "base_link") {
          disarm("velocity must be expressed in base_link");
          return;
        }
        if (!tc::fresh_stamp(m.header.stamp, now(), timeout_)) {
          disarm("expired velocity timestamp");
          return;
        }
        const auto & v = m.twist;
        if (v.linear.y != 0 || v.linear.z != 0 || v.angular.x != 0 || v.angular.y != 0) {
          disarm("only linear.x/angular.z are supported");
          return;
        }
        auto c = tp::Control{};
        c.linear = v.linear.x;
        c.angular = v.angular.z;
        try {
          tp::validate(c);
        } catch (const tp::ProtocolError & e) {
          disarm(e.what());
          return;
        }
        double stamp = rclcpp::Time(m.header.stamp).seconds();
        if (have_velocity_ && stamp <= velocity_stamp_) {
          disarm(
            "out-of-order velocity");
          return;
        }
        velocity_stamp_ = stamp;
        linear_ = c.linear;
        angular_ = c.angular;
        velocity_time_ = tc::source_time_on_steady_clock(m.header.stamp, now());
        have_velocity_ = true;
      });
    // 作业目标是一整组替换，不是对上次指令的增量修改。
    implements_ = create_subscription<ti::ImplementCommand>(
      "implements/command", q, [this](const ti::ImplementCommand & m) {
        if (!tc::fresh_stamp(m.header.stamp, now(), timeout_)) {
          disarm("expired implement timestamp");
          return;
        }
        ti::ControlCommand complete;
        complete.implements = m;
        complete.enable = true;
        auto c = tc::from_ros(complete);
        try {
          tp::validate(c, capabilities_);
        } catch (const tp::ProtocolError & e) {
          disarm(e.what());
          return;
        }
        double stamp = rclcpp::Time(m.header.stamp).seconds();
        if (have_implements_ && stamp <= implement_stamp_) {
          disarm("out-of-order implement command");
          return;
        }
        implement_stamp_ = stamp;
        implement_command_ = c;
        implement_time_ = tc::source_time_on_steady_clock(m.header.stamp, now());
        have_implements_ = true;
      });
    board_ = create_subscription<ti::McuState>(
      "mcu/state", q, [this](const ti::McuState & m) {
        if (!tc::fresh_stamp(m.header.stamp, now(), state_timeout_)) {
          return;
        }
        if (connection_ != m.connection_id) {
          // 拔插、后端重启或仿真复位都会产生新代次，旧目标不能继承。
          connection_ = m.connection_id;
          disarm("new board connection; enable required");
        }
        state_time_ = tc::source_time_on_steady_clock(m.header.stamp, now());
        capabilities_ = m.capabilities;
        link_ready_ = m.link_ready && m.result == 0 && !(m.faults & 3) && m.mode != 0 && m.command_age_ms < state_timeout_ * 1000;
        if (!link_ready_) {
          disarm("board not ready or command rejected");
        }
      });
    enable_ = create_service<std_srvs::srv::SetBool>(
      "control/set_enabled",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (!req->data) {
          disarm("disabled by request");
          res->success = true;
          res->message = reason_;
          return;
        }
        if (!healthy()) {
          res->success = false;
          res->message = "fresh healthy board state required";
          return;
        }
        if (enabled_) {
          // 重复使能是幂等操作，不能通过反复调用服务延长指令有效期。
          res->success = true;
          res->message = "already enabled";
          return;
        }
        disarm(
          "enabled; waiting for new command");
        enabled_ = true;
        enabled_at_ = tc::steady_seconds();
        res->success = true;
        res->message = reason_;
      });
    stop_ = create_service<std_srvs::srv::Trigger>(
      "control/stop",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
        disarm(
          "stop requested; enable required");
        res->success = true;
        res->message = reason_;
        publish();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {publish();});
  }

private:
  bool healthy()const
  {
    return link_ready_ && connection_ && tc::steady_seconds() - state_time_ <= state_timeout_;
  }
  void disarm(const std::string & reason)
  {
    // 同时丢弃底盘和作业缓存；再次使能必须取得新的输入。
    enabled_ = false;
    have_velocity_ = have_implements_ = false;
    linear_ = angular_ = 0;
    implement_command_ = {};
    reason_ = reason;
  }
  void publish()
  {
    // 底盘心跳始终必需；作业装置开启时还独立检查作业指令有效期。
    // 仅操作刀盘/喷洒时，也应持续发布零速度 cmd_vel。
    double t = tc::steady_seconds();
    if (enabled_ && !healthy()) {
      disarm("board status timeout or fault");
    }
    if (enabled_ &&
      ((!have_velocity_ && t - enabled_at_ > timeout_) ||
      (have_velocity_ && t - velocity_time_ > timeout_)))
    {
      disarm("velocity command timeout");
    }
    if (enabled_ && have_implements_ && tc::any_on(implement_command_) &&
      t - implement_time_ > timeout_)
    {
      disarm("implement command timeout");
    }
    tp::Control c = tp::stopped();
    // 缺省输出始终是停机快照，只有所有保护条件满足时才构造运行目标。
    if (enabled_) {
      c = implement_command_;
      c.enable = true;
      c.stop = false;
      c.linear = have_velocity_ ? linear_ : 0;
      c.angular = have_velocity_ ? angular_ : 0;
      reason_ = "enabled";
    }
    try {
      tp::validate(c, capabilities_);
    } catch (const tp::ProtocolError & e) {
      disarm(e.what());
      c = tp::stopped();
    }
    ti::ControlCommand m;
    m.header.stamp = now();
    m.header.frame_id = "base_link";
    m.connection_id = connection_;
    tc::fill_ros(c, m);
    commands_->publish(m);
    ti::ControlStatus s;
    s.header = m.header;
    s.enabled = enabled_;
    s.link_ready = healthy();
    s.connection_id = connection_;
    s.reason = reason_;
    status_->publish(s);
  }
  double timeout_, state_timeout_, state_time_ = -1, enabled_at_ = 0, velocity_time_ = 0,
    implement_time_ = 0, velocity_stamp_ = 0, implement_stamp_ = 0;
  uint64_t connection_ = 0;
  uint16_t capabilities_ = tp::CURRENT_CAPABILITIES;
  bool enabled_ = false, link_ready_ = false, have_velocity_ = false, have_implements_ = false;
  float linear_ = 0, angular_ = 0;
  tp::Control implement_command_;
  std::string reason_ = "startup disabled";
  rclcpp::Publisher<ti::ControlCommand>::SharedPtr commands_;
  rclcpp::Publisher<ti::ControlStatus>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_;
  rclcpp::Subscription<ti::ImplementCommand>::SharedPtr implements_;
  rclcpp::Subscription<ti::McuState>::SharedPtr board_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ControlManager>());
  } catch (const std::exception & e) {
    fprintf(stderr, "control_manager: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
