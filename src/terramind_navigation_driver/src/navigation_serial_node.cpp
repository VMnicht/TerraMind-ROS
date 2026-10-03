#include "terramind_navigation_driver/coordinates.hpp"
#include "terramind_navigation_driver/session.hpp"
#include "terramind_transport/serial_port.hpp"
#include "terramind_interfaces/msg/navigation_frame.hpp"
#include "terramind_interfaces/msg/navigation_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <chrono>
#include <thread>

namespace nav = terramind::navigation;
namespace ti = terramind_interfaces::msg;
namespace
{
double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
constexpr double pi = 3.14159265358979323846;
}
// 串口、组帧、会话和坐标基准仅由工作线程操作；ROS executor 不参与串口等待。
class NavigationSerialNode : public rclcpp::Node
{
public:
  NavigationSerialNode() : Node("navigation_serial_node")
  {
    device_ = declare_parameter("device", std::string("/dev/terramind-navigation"));
    if (device_.empty() || device_ == "auto") {
      throw std::invalid_argument("navigation device requires an explicit serial path");
    }
    baud_ = declare_parameter("baud", 460800);
    if (baud_ != 460800) {throw std::invalid_argument("navigation v1 requires 460800 baud");}
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("serial navigation requires wall time");
    }
    retry_ = declare_parameter("reconnect_interval_s", 1.0);
    if (!std::isfinite(retry_) || retry_ < 0.1 || retry_ > 30) {
      throw std::invalid_argument("reconnect interval must be in [0.1, 30]");
    }
    nav::Limits limits;
    limits.frame_timeout_s = declare_parameter("connection_timeout_s", 3.0);
    limits.navigation_timeout_s = declare_parameter("navigation_timeout_s", 0.1);
    int max_age = declare_parameter("max_age_ms", 500);
    if (max_age < 0 || max_age > 65534) {throw std::invalid_argument("invalid max_age_ms");}
    limits.max_age_ms = max_age;
    limits.allow_simulated = declare_parameter("allow_simulated", false);
    session_ = nav::Session(limits);
    // session_id 在进程重启后也尽量不同；协议没有设备会话号，不能承诺完整识别重启。
    session_.id = static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());
    parser_ = nav::Parser(declare_parameter("assembly_timeout_s", 0.1));
    frame_ = declare_parameter("map_frame", std::string("navigation_map"));
    imu_frame_ = declare_parameter("imu_frame", std::string("navigation_imu"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    if (frame_.empty() || imu_frame_.empty() || base_frame_.empty() || frame_ == imu_frame_ ||
      frame_ == base_frame_ || imu_frame_ == base_frame_) {
      throw std::invalid_argument("navigation frame names must be nonempty and distinct");
    }
    calibrated_ = declare_parameter("extrinsics_calibrated", false);
    auto t = declare_parameter("imu_translation_in_base_m", std::vector<double>{0, 0, 0});
    auto q = declare_parameter("imu_rotation_in_base_wxyz", std::vector<double>{1, 0, 0, 0});
    if (t.size() != 3 || q.size() != 4) {throw std::invalid_argument("extrinsics require xyz and wxyz");}
    coordinates_.set_extrinsics({t[0], t[1], t[2]}, {q[0], q[1], q[2], q[3]});
    origin_mode_ = declare_parameter("origin_mode", std::string("first_valid"));
    origin_fixed_rtk_ = declare_parameter("origin_require_rtk_fixed", true);
    const double lat = declare_parameter("origin_latitude_deg", 0.0);
    const double lon = declare_parameter("origin_longitude_deg", 0.0);
    const double height = declare_parameter("origin_height_m", 0.0);
    if (origin_mode_ == "fixed") {
      coordinates_.set_origin({lat * pi / 180, lon * pi / 180, height});
    } else if (origin_mode_ != "first_valid") {
      throw std::invalid_argument("origin_mode must be fixed or first_valid");
    }
    raw_ = create_publisher<ti::NavigationFrame>("navigation/raw", rclcpp::SensorDataQoS());
    status_ = create_publisher<ti::NavigationStatus>("navigation/status", rclcpp::QoS(1).reliable());
    pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("navigation/pose", rclcpp::SensorDataQoS());
    velocity_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("navigation/velocity", rclcpp::SensorDataQoS());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    worker_ = std::thread([this] {run();});
  }
  ~NavigationSerialNode() override
  {
    running_ = false;
    if (worker_.joinable()) {worker_.join();}
  }
private:
  void receive(const nav::Frame & f, double received)
  {
    const auto stamp = now();
    const auto observation = session_.observe(f, received);
    if (observation.restarted) {last_orientation_.reset();}
    latest_ = f;
    ti::NavigationFrame raw;
    raw.header.stamp = stamp; raw.header.frame_id = imu_frame_ + "_frd";
    raw.session_id = session_.id; raw.status = f.status; raw.rtk_state = f.rtk_state();
    raw.packet_seq = f.sequence; raw.state_time_s = f.state_time_s;
    raw.latitude_rad = f.latitude_rad; raw.longitude_rad = f.longitude_rad; raw.height_m = f.height_m;
    raw.velocity_ned_mps = f.velocity_ned; raw.rpy_rad = f.rpy;
    raw.quaternion_wxyz = f.quaternion_wxyz; raw.age_ms = f.age_ms;
    raw_->publish(raw);
    if (!observation.publish) {return;}
    if (!coordinates_.has_origin()) {
      // 自动原点默认要求固定解且实际参与过近期 GNSS 更新，不能仅凭 RTK 高位。
      if (origin_fixed_rtk_ && (f.rtk_state() != 4 || !(f.status & nav::GNSS_RECENT))) {return;}
      coordinates_.set_origin({f.latitude_rad, f.longitude_rad, f.height_m});
      RCLCPP_INFO(get_logger(), "Established fixed navigation origin at %.9f, %.9f, %.3f m",
        f.latitude_rad * 180 / pi, f.longitude_rad * 180 / pi, f.height_m);
    }
    const auto result = coordinates_.convert(f);
    Eigen::Quaterniond orientation = calibrated_ ? result.base_orientation : result.imu_orientation;
    if (last_orientation_ && orientation.dot(*last_orientation_) < 0) {orientation.coeffs() *= -1;}
    last_orientation_ = orientation;
    const auto & position = calibrated_ ? result.base_position : result.imu_position;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp; pose.header.frame_id = frame_;
    pose.pose.position.x = position.x(); pose.pose.position.y = position.y(); pose.pose.position.z = position.z();
    pose.pose.orientation.w = orientation.w(); pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y(); pose.pose.orientation.z = orientation.z();
    pose_->publish(pose);
    // 协议没有角速度；该速度始终属于 IMU 中心，不伪造车辆控制点速度。
    geometry_msgs::msg::Vector3Stamped velocity;
    velocity.header = pose.header;
    velocity.vector.x = result.imu_velocity.x(); velocity.vector.y = result.imu_velocity.y();
    velocity.vector.z = result.imu_velocity.z(); velocity_->publish(velocity);
  }
  void report(double t)
  {
    ti::NavigationStatus s;
    s.header.stamp = now(); s.header.frame_id = frame_; s.session_id = session_.id;
    s.online = session_.online(t); s.usable = session_.usable(t);
    s.origin_ready = coordinates_.has_origin(); s.extrinsics_calibrated = calibrated_;
    s.pose_child_frame = calibrated_ ? base_frame_ : imu_frame_;
    s.velocity_point_frame = imu_frame_;
    s.stamp_reference = "receive_time"; s.reason = session_.reason(t);
    if (s.usable && !s.origin_ready) {s.reason = "waiting_for_origin_quality";}
    if (!port_.is_open() && !port_error_.empty()) {s.reason = port_error_;}
    s.age_ms = 65535;
    if (latest_) {
      const auto & f = *latest_;
      s.nav_valid = f.status & nav::NAV_VALID; s.nav_fault = f.status & nav::NAV_FAULT;
      s.time_locked = f.status & nav::TIME_LOCKED; s.gnss_recent = f.status & nav::GNSS_RECENT;
      s.simulated = f.status & nav::SIMULATED; s.rtk_state = f.rtk_state();
      s.age_ms = f.age_ms; s.packet_seq = f.sequence;
    }
    if (coordinates_.has_origin()) {
      const auto & o = coordinates_.origin();
      s.origin_latitude_rad = o.latitude_rad; s.origin_longitude_rad = o.longitude_rad; s.origin_height_m = o.height_m;
    }
    const auto & p = parser_.stats();
    s.received_frames = p.frames; s.crc_errors = p.crc_errors; s.version_errors = p.version_errors;
    s.assembly_timeouts = p.timeouts; s.discarded_bytes = p.discarded;
    s.missing_packets = session_.missing; s.duplicate_measurements = session_.duplicates;
    s.rejected_measurements = session_.rejected; s.board_restarts = session_.restarts; s.reconnects = reconnects_;
    status_->publish(s);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header = s.header;
    diagnostic_msgs::msg::DiagnosticStatus d;
    d.name = "navigation_serial"; d.hardware_id = device_; d.message = s.reason;
    d.level = !s.online || s.nav_fault ? 2 : (!s.usable || !s.origin_ready ? 1 : 0);
    for (const auto & pair : std::vector<std::pair<std::string, std::string>>{
        {"frames", std::to_string(p.frames)}, {"crc_errors", std::to_string(p.crc_errors)},
        {"assembly_timeouts", std::to_string(p.timeouts)}, {"missing_packets", std::to_string(s.missing_packets)},
        {"stamp_reference", s.stamp_reference}, {"pose_child_frame", s.pose_child_frame},
        {"velocity_point_frame", s.velocity_point_frame}}) {
      diagnostic_msgs::msg::KeyValue kv; kv.key = pair.first; kv.value = pair.second; d.values.push_back(kv);
    }
    diagnostics.status.push_back(d); diagnostics_->publish(diagnostics);
  }
  void run()
  {
    double next_open = 0, next_report = 0;
    bool opened_once = false;
    while (running_ && rclcpp::ok()) {
      try {
        double t = steady_seconds();
        if (!port_.is_open() && t >= next_open) {
          port_.open(device_, baud_); parser_.reset(); session_.connect(t);
          if (opened_once) {++reconnects_;}
          opened_once = true; latest_.reset(); last_orientation_.reset(); port_error_.clear();
          RCLCPP_INFO(get_logger(), "Receiving navigation on %s at %d baud", device_.c_str(), baud_);
        }
        if (port_.is_open()) {
          auto bytes = port_.read_some(5);
          t = steady_seconds();
          // 空读取也调用 feed，使 100 ms 残帧超时不依赖后续数据到达。
          for (const auto & frame : parser_.feed(bytes.data(), bytes.size(), t)) {receive(frame, t);}
          if (session_.expired(t)) {throw std::runtime_error("navigation connection timeout");}
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      } catch (const std::exception & e) {
        port_.close(); parser_.reset(); session_.disconnect(); latest_.reset(); last_orientation_.reset();
        if (port_error_ != e.what()) {RCLCPP_WARN(get_logger(), "%s", e.what());}
        port_error_ = e.what(); next_open = steady_seconds() + retry_;
      }
      const double t = steady_seconds();
      if (t >= next_report && rclcpp::ok()) {
        try {report(t);} catch (const rclcpp::exceptions::RCLError &) {}
        next_report = t + 0.05;
      }
    }
    port_.close();
  }
  std::string device_, frame_, imu_frame_, base_frame_, origin_mode_, port_error_;
  int baud_ = 460800;
  double retry_ = 1;
  bool calibrated_ = false, origin_fixed_rtk_ = true;
  uint64_t reconnects_ = 0;
  std::atomic<bool> running_{true};
  std::thread worker_;
  terramind::transport::SerialPort port_;
  nav::Parser parser_;
  nav::Session session_;
  nav::Coordinates coordinates_;
  std::optional<nav::Frame> latest_;
  std::optional<Eigen::Quaterniond> last_orientation_;
  rclcpp::Publisher<ti::NavigationFrame>::SharedPtr raw_;
  rclcpp::Publisher<ti::NavigationStatus>::SharedPtr status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<NavigationSerialNode>();
    rclcpp::spin(node); node.reset();
  } catch (const std::exception & e) {
    fprintf(stderr, "navigation_serial_node: %s\n", e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
