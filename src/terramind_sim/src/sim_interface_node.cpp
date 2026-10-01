#include "terramind_sim/board_model.hpp"
#include "terramind_mcu/ros_conversions.hpp"
#include "terramind_control/ros_helpers.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>
namespace tp = terramind::protocol;
namespace tc = terramind::control;
namespace mcu = terramind::mcu;
namespace ts = terramind::sim;
namespace ti = terramind_interfaces::msg;
class SimInterface : public rclcpp::Node
{
public:
  SimInterface()
  : Node("sim_interface_node")
  {
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("MVP simulator runs at wall-clock 1x");
    }
    timeout_ = declare_parameter("command_timeout_s", 0.150);
    double state_timeout = declare_parameter("state_timeout_s", 0.150);
    guard_ = tc::CommandGuard(timeout_);
    session_ = mcu::Session(state_timeout);
    ts::Geometry g{declare_parameter("wheel_diameter_m", 0.20), declare_parameter(
        "wheel_separation_m", 0.45)};
    auto caps = declare_parameter("capabilities", int(tp::CURRENT_CAPABILITIES));
    if (caps < 0 || caps > 63) {
      throw std::invalid_argument("invalid capabilities");
    }
    double a = declare_parameter("linear_acceleration_mps2", 0.7), aa = declare_parameter(
      "angular_acceleration_radps2", 4.0);
    model_ = std::make_unique<ts::BoardModel>(g, caps, a, aa);
    auto q = rclcpp::QoS(1).reliable().durability_volatile();
    states_ = create_publisher<ti::McuState>(
      "mcu/state",
      q);
    truth_ = create_publisher<nav_msgs::msg::Odometry>("sim/ground_truth", 10);
    joints_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    reset_model();
    commands_ = create_subscription<ti::ControlCommand>(
      "mcu/command", q, [this](const ti::ControlCommand & m) {
        if (m.connection_id != session_.connection_id) {
          return;
        }
        double stamp = rclcpp::Time(m.header.stamp).seconds();
        guard_.receive(
          tc::from_ros(m), m.connection_id, tc::source_time_on_steady_clock(m.header.stamp, now()),
          tc::fresh_stamp(m.header.stamp, now(), timeout_) && stamp > last_stamp_,
          model_->state().system.capabilities);
        last_stamp_ = std::max(last_stamp_, stamp);
      });
    reset_ =
      create_service<std_srvs::srv::Trigger>(
      "sim/reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr r) {
        reset_model();
        r->success = true;
        r->message = "reset; explicit enable required";
      });
    drop_commands_ =
      create_service<std_srvs::srv::SetBool>(
      "sim/drop_commands",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr q,
      std_srvs::srv::SetBool::Response::SharedPtr r) {drop_tx_ = q->data;r->success = true;});
    drop_status_ =
      create_service<std_srvs::srv::SetBool>(
      "sim/drop_status",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr q,
      std_srvs::srv::SetBool::Response::SharedPtr r) {drop_rx_ = q->data;r->success = true;});
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] {tick();});
  }

private:
  void reset_model()
  {
    double t = tc::steady_seconds();
    model_->reset(t);
    session_.reset(tc::new_connection_id(), t);
    guard_.reset(session_.connection_id);
    next_tx_ = next_state_ = next_diagnostic_ = t;
    last_stamp_ = 0;
    seq_ = status_seq_ = 0;
  }
  void tick()
  {
    double t = tc::steady_seconds();
    if (session_.expired(t)) {
      session_.reset(tc::new_connection_id(), t);
      guard_.reset(session_.connection_id);
      last_stamp_ = 0;
    }
    if (t >= next_tx_) {
      auto c = guard_.sample(t, session_.ready);
      if (!session_.ready) {
        c = {};
      }
      session_.sent(seq_, c, t);
      if (!drop_tx_) {
        model_->accept(c, seq_, t);
      }
      ++seq_;
      next_tx_ = t + 0.020;
    }
    model_->step(t);
    if (t >= next_state_) {
      auto s = model_->state();
      if (!drop_rx_) {
        bool ready = session_.ready;
        session_.observe(s, status_seq_, t);
        if (ready && !session_.ready) {
          guard_.trip(session_.reason);
        }
        states_->publish(mcu::to_ros(s, status_seq_, session_, true, now()));
      }
      ++status_seq_;
      publish_truth();
      next_state_ = t + 0.050;
    }
    if (t >= next_diagnostic_) {
      diagnostics_->publish(
        mcu::diagnostic(
          now(), "mcu_sim", "simulation", session_.ready ? 0 : 1,
          session_.reason, model_->state().system.rx_error_count));
      next_diagnostic_ = t + 1;
    }
  }
  void publish_truth()
  {
    auto stamp = now();
    const auto & d = model_->drive();
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = "sim_world";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = d.x;
    o.pose.pose.position.y = d.y;
    o.pose.pose.orientation.z = std::sin(
      d.yaw / 2);
    o.pose.pose.orientation.w = std::cos(d.yaw / 2);
    o.twist.twist.linear.x = d.linear;
    o.twist.twist.angular.z = d.angular;
    truth_->publish(o);
    geometry_msgs::msg::TransformStamped f;
    f.header = o.header;
    f.child_frame_id = o.child_frame_id;
    f.transform.translation.x = d.x;
    f.transform.translation.y = d.y;
    f.transform.rotation = o.pose.pose.orientation;
    tf_->sendTransform(f);
    sensor_msgs::msg::JointState j;
    j.header.stamp = stamp;
    j.name = {"left_wheel_joint", "right_wheel_joint"};
    j.position = {d.left_position, d.right_position};
    j.velocity =
    {model_->state().chassis.left_actual * 0.104719755,
      model_->state().chassis.right_actual * 0.104719755};
    joints_->publish(j);
  }
  std::unique_ptr<ts::BoardModel> model_;
  tc::CommandGuard guard_;
  mcu::Session session_;
  double timeout_ = 0.15, last_stamp_ = 0, next_tx_ = 0, next_state_ = 0, next_diagnostic_ = 0;
  uint16_t seq_ = 0, status_seq_ = 0;
  bool drop_tx_ = false, drop_rx_ = false;
  rclcpp::Publisher<ti::McuState>::SharedPtr states_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<ti::ControlCommand>::SharedPtr commands_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr drop_commands_, drop_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SimInterface>());
  } catch (const std::exception & e) {
    fprintf(stderr, "sim_interface_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
