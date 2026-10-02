#include "terramind_mcu/mcu_session.hpp"
#include <stdexcept>
namespace terramind::mcu
{
Session::Session(double timeout)
: timeout_(timeout)
{
  if (!(timeout > 0 && timeout < 0.250)) {
    throw std::invalid_argument("state timeout must be in (0, 0.250)");
  }
}
void Session::reset(uint64_t id, double now)
{
  connection_id = id;
  ready = false;
  rebooted = false;
  have_state_ = false;
  sent_.clear();
  started_ = last_rx_ = now;
  capabilities = protocol::CURRENT_CAPABILITIES;
  reason = "zero command handshake";
}
void Session::sent(uint16_t seq, const protocol::Control & c, double now)
{
  // 状态 20 Hz、命令 50 Hz，回传只确认最近命令；不要求逐帧 ACK。
  sent_.push_back({seq, now, !c.enable || c.stop});
  while (sent_.size() > 64) {sent_.pop_front();}
}
bool Session::observe(const protocol::State & s, uint16_t frame_seq, double now)
{
  if (have_state_ && s.system.uptime_ms < uptime_ &&
    !(uptime_ > 0xffff0000u && s.system.uptime_ms < 0x10000u))
  {
    // uptime 回退视为板卡重启；上方排除了 uint32 在边界附近的正常回绕。
    ready = false;
    rebooted = true;
    reason = "board reboot";
    return true;
  }
  if (have_state_) {
    uint16_t delta = frame_seq - state_seq_;
    // 半区间比较允许 uint16 正常回绕，同时拒绝重复和倒序状态。
    if (delta == 0 || delta >= 0x8000) {
      return false;
    }
  }
  have_state_ = true;
  state_seq_ = frame_seq;
  uptime_ = s.system.uptime_ms;
  last_rx_ = now;
  capabilities = s.system.capabilities;
  const Sent * matched = nullptr;
  // 序号必须匹配本次连接实际发送的近期记录，并验证板卡命令年龄和结果。
  for (auto i = sent_.rbegin(); i != sent_.rend(); ++i) {
    if (i->seq == s.system.last_command_seq) {
      matched = &*i;
      break;
    }
  }
  if (!matched || now - matched->time > timeout_ || s.system.command_age_ms >= timeout_ * 1000 ||
    s.system.result != 0 || (s.system.faults & 3) || s.system.mode == 0)
  {
    ready = false;
    reason = "status does not confirm a fresh accepted command";
    return true;
  }
  if (!ready && (!matched->disabled || s.system.mode != 2)) {
    // 首次就绪或故障恢复，必须确认一次被板卡接纳的未使能快照。
    reason = "awaiting accepted disabled snapshot";
    return true;
  }
  ready = true;
  reason = "ready";
  return true;
}
bool Session::expired(double now)const
{
  return now - last_rx_ > timeout_ || (!have_state_ && now - started_ > timeout_);
}
}
