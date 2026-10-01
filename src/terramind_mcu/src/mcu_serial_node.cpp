#include "terramind_mcu/ros_conversions.hpp"
#include "terramind_mcu/port_discovery.hpp"
#include "terramind_control/ros_helpers.hpp"
#include "terramind_protocol/frame_parser.hpp"
#include "terramind_transport/serial_port.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>
namespace tp = terramind::protocol;
namespace tc = terramind::control;
namespace mcu = terramind::mcu;
namespace ti = terramind_interfaces::msg;
class McuSerialNode : public rclcpp::Node
{
public:
  McuSerialNode()
  : Node("mcu_serial_node"), guard_(0.150), session_(0.150)
  {
    configured_device_ = declare_parameter("device", std::string("auto"));
    if (configured_device_.empty()) {
      throw std::invalid_argument("device must be auto or an explicit serial path");
    }
    automatic_ = configured_device_ == "auto";
    path_ = configured_device_;
    discovery_patterns_ = declare_parameter("discovery_patterns", std::vector<std::string>{
      "/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"});
    listen_seconds_ = declare_parameter("discovery_listen_s", 0.6);
    if (!std::isfinite(listen_seconds_) || listen_seconds_ < 0.15 || listen_seconds_ > 10 ||
      (automatic_ && discovery_patterns_.empty()))
    {
      throw std::invalid_argument("discovery requires nonempty patterns and listen_s in [0.15, 10]");
    }
    int baud = declare_parameter("baud", 115200);
    if (baud != 115200) {
      throw std::invalid_argument("protocol v1 requires 115200 baud");
    }
    timeout_ = declare_parameter("command_timeout_s", 0.150);
    state_timeout_ = declare_parameter(
      "state_timeout_s", 0.150);
    guard_ = tc::CommandGuard(timeout_);
    session_ = mcu::Session(state_timeout_);
    period_ = declare_parameter("tx_period_ms", 20);
    if (period_ < 20 || period_ > 50) {
      throw std::invalid_argument("tx_period_ms must be 20..50");
    }
    retry_ = declare_parameter("reconnect_interval_s", 1.0);
    if (!(retry_ >= 0.1 && retry_ <= 30)) {
      throw std::invalid_argument("invalid reconnect interval");
    }
    simulated_ = declare_parameter("simulated", false);
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("serial watchdog requires wall time");
    }
    auto q = rclcpp::QoS(1).reliable().durability_volatile();
    states_ = create_publisher<ti::McuState>(
      "mcu/state",
      q);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    commands_ = create_subscription<ti::ControlCommand>(
      "mcu/command", q, [this](const ti::ControlCommand & m) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (m.connection_id != connection_) {
          return;
        }
        double stamp = rclcpp::Time(m.header.stamp).seconds();
        bool valid = tc::fresh_stamp(m.header.stamp, now(), timeout_) && stamp > last_stamp_;
        guard_.receive(
          tc::from_ros(m), m.connection_id,
          tc::source_time_on_steady_clock(m.header.stamp, now()), valid, capabilities_);
        last_stamp_ = std::max(last_stamp_, stamp);
      });
    worker_ = std::thread([this] {run();});
  }
  ~McuSerialNode()override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void report(int level, const std::string & reason)
  {
    // Shutdown can invalidate the ROS context while the serial worker exits.
    if (rclcpp::ok()) {
      try {
        diagnostics_->publish(
          mcu::diagnostic(now(), "mcu_serial", path_, level, reason, parser_.errors()));
      } catch (const rclcpp::exceptions::RCLError &) {
        // The final wire stop below must not depend on ROS publication succeeding.
      }
    }
  }
  void run()
  {
    uint16_t seq = 0;
    double next_tx = 0, next_open = 0, next_report = 0;
    std::unique_ptr<terramind::transport::SerialPort> port;
    while (running_ && rclcpp::ok()) {
      double t = tc::steady_seconds();
      try {
        if (!port) {
          if (t < next_open) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }
          if (automatic_) {
            path_ = "auto";
            report(1, "正在扫描串口，监听控制板状态帧");
            auto discovered = mcu::discover_control_board(
              discovery_patterns_, listen_seconds_, [this] {return running_ && rclcpp::ok();});
            if (!running_ || !rclcpp::ok()) {
              break;
            }
            if (!discovered.port) {
              std::string reason;
              if (discovered.matches.size() > 1) {
                reason = "发现多个控制板，请指定 device：";
                for (const auto & match : discovered.matches) {
                  reason += " " + match;
                }
              } else if (discovered.candidate_count == 0) {
                reason = "未发现候选串口，等待 USB 设备接入";
              } else {
                reason = "已扫描 " + std::to_string(discovered.candidate_count) +
                  " 个端口，未识别到持续上报状态的控制板";
              }
              for (size_t i = 0; i < std::min<size_t>(3, discovered.errors.size()); ++i) {
                reason += "; " + discovered.errors[i];
              }
              report(discovered.matches.size() > 1 ? 2 : 1, reason);
              if (reason != last_discovery_report_) {
                RCLCPP_WARN(get_logger(), "%s", reason.c_str());
                last_discovery_report_ = reason;
              }
              next_open = tc::steady_seconds() + retry_;
              continue;
            }
            path_ = discovered.device;
            port = std::move(discovered.port);
            last_discovery_report_.clear();
            RCLCPP_INFO(get_logger(), "Identified control board at %s", path_.c_str());
          } else {
            auto manual = std::make_unique<terramind::transport::SerialPort>();
            manual->open(configured_device_);
            port = std::move(manual);
          }
          t = tc::steady_seconds();
          parser_.reset();
          seq = 0;
          session_.reset(tc::new_connection_id(), t);
          {
            std::lock_guard<std::mutex> lock(mutex_);
            connection_ = session_.connection_id;
            capabilities_ = tp::CURRENT_CAPABILITIES;
            guard_.reset(connection_);
            last_stamp_ = 0;
          }
          next_tx = t;
          RCLCPP_INFO(
            get_logger(), "Opened %s; sending disabled handshake",
            path_.c_str());
          report(1, session_.reason);
        }
        if (session_.expired(t)) {
          throw std::runtime_error("status or handshake timeout");
        }
        if (t >= next_tx) {
          tp::Control c;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            c = guard_.sample(t, session_.ready);
          }
          if (!session_.ready) {
            c = {};
          }
          port->write_all(tp::encode_control(c, seq));
          session_.sent(seq++, c, t);
          next_tx = t + period_ * 0.001;
        }
        auto bytes = port->read_some(5);
        for (const auto & f:parser_.feed(bytes)) {
          if (f.type != tp::STATUS) {
            continue;
          }
          tp::State s;
          try {
            s = tp::decode_state(f);
          } catch (const tp::ProtocolError &) {
            continue;
          }
          bool was_ready = session_.ready;
          if (!session_.observe(s, f.seq, tc::steady_seconds())) {
            continue;
          }
          if (session_.rebooted) {
            throw std::runtime_error("board restarted; new handshake required");
          }
          {
            std::lock_guard<std::mutex> lock(mutex_);
            capabilities_ = session_.capabilities;
            if (was_ready && !session_.ready) {
              guard_.trip(session_.reason);
            }
          }
          states_->publish(mcu::to_ros(s, f.seq, session_, simulated_, now()));
        }
        if (t >= next_report) {
          report(session_.ready ? 0 : 1, session_.reason);
          next_report = t + 1;
        }
      } catch (const std::exception & e) {
        // A stop is best effort; firmware timeout remains the fallback if TX is broken.
        try {
          if (port && port->is_open()) {
            port->write_all(tp::encode_control(tp::stopped(), seq++));
          }
        } catch (...) {
        }
        port.reset();
        session_.ready = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          guard_.trip(e.what());
        }
        RCLCPP_WARN(get_logger(), "%s", e.what());
        report(2, e.what());
        next_open = tc::steady_seconds() + retry_;
      }
    }
    try {
      if (port && port->is_open()) {
        port->write_all(tp::encode_control(tp::stopped(), seq));
      }
    } catch (...) {
    }
  }
  std::string path_, configured_device_, last_discovery_report_;
  std::vector<std::string> discovery_patterns_;
  double listen_seconds_ = 0.6;
  bool automatic_ = true;
  double timeout_ = 0.15, state_timeout_ = 0.15, retry_ = 1;
  int period_ = 20;
  bool simulated_ = false;
  uint64_t connection_ = 0;
  uint16_t capabilities_ = tp::CURRENT_CAPABILITIES;
  double last_stamp_ = 0;
  std::atomic<bool> running_{true};
  std::thread worker_;
  std::mutex mutex_;
  tc::CommandGuard guard_;
  mcu::Session session_;
  tp::FrameParser parser_;
  rclcpp::Publisher<ti::McuState>::SharedPtr states_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<ti::ControlCommand>::SharedPtr commands_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<McuSerialNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & e) {
    fprintf(stderr, "mcu_serial_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
