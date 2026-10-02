#include "terramind_control/command_guard.hpp"
#include <chrono>
#include <random>
namespace terramind::control
{
double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
uint64_t new_connection_id()
{
  std::random_device r;
  uint64_t v = (uint64_t(r()) << 32) | r();
  return v ? v : 1;
}
CommandGuard::CommandGuard(double timeout)
: timeout_(timeout)
{
  if (!(timeout > 0 && timeout < protocol::FIRMWARE_TIMEOUT)) {
    throw std::invalid_argument("command timeout must be in (0, 0.250) seconds");
  }
}
void CommandGuard::reset(uint64_t connection)
{
  connection_ = connection;
  trip("new connection requires disabled command");
}
void CommandGuard::trip(const std::string & reason)
{
  latest_ = protocol::stopped();
  may_enable_ = false;
  received_ = -1;
  reason_ = reason;
}
bool CommandGuard::receive(
  const protocol::Control & c, uint64_t connection, double now,
  bool valid_stamp, uint16_t capabilities)
{
  if (connection != connection_ || !connection) {
    return false;
  }
  if (!valid_stamp) {
    trip("invalid or expired timestamp");
    return false;
  }
  try {
    protocol::validate(c, capabilities);
  } catch (const protocol::ProtocolError & e) {
    trip(e.what());
    return false;
  }
  if (!c.enable || c.stop) {
    // 先看到明确的停机状态才能解除故障锁；恢复消息流本身不构成重新使能。
    may_enable_ = true;
    latest_ = protocol::stopped();
    received_ = now;
    reason_ = "disabled";
    return true;
  }
  if (!may_enable_) {
    return false;
  }
  latest_ = c;
  received_ = now;
  reason_ = "active";
  return true;
}
protocol::Control CommandGuard::sample(double now, bool link_ready)
{
  // 发送周期主动采样，而不是只在收到消息时检查，才能发现上游静默退出。
  if (!link_ready) {
    if (latest_.enable) {
      trip("link not ready");
    }
    return protocol::stopped();
  }
  if (received_ < 0 || now < received_ || now - received_ > timeout_) {
    trip("command timeout");
    return protocol::stopped();
  }
  return latest_;
}
}
