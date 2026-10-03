#include "terramind_navigation_driver/session.hpp"
#include <cmath>
#include <stdexcept>

namespace terramind::navigation
{
Session::Session(Limits limits) : limits_(limits)
{
  if (!std::isfinite(limits.frame_timeout_s) || limits.frame_timeout_s <= 0 ||
    !std::isfinite(limits.navigation_timeout_s) || limits.navigation_timeout_s <= 0 ||
    limits.max_age_ms == 65535) {throw std::invalid_argument("invalid navigation limits");}
}
void Session::new_epoch()
{
  ++id; eligible_ = false; sequence_.reset(); measurement_time_.reset();
}
void Session::connect(double now)
{
  new_epoch(); connected_ = true; opened_ = now; last_frame_.reset(); reason_ = "waiting_for_frame";
}
void Session::disconnect()
{
  connected_ = false; eligible_ = false; last_frame_.reset(); reason_ = "disconnected";
}
bool Session::online(double now) const
{
  return connected_ && last_frame_ && now - *last_frame_ <= limits_.frame_timeout_s;
}
bool Session::expired(double now) const
{
  return connected_ && now - last_frame_.value_or(opened_) > limits_.frame_timeout_s;
}
bool Session::usable(double now) const
{
  return online(now) && eligible_ && now - last_good_ <= limits_.navigation_timeout_s &&
    last_age_ + (now - last_good_) * 1000 <= limits_.max_age_ms;
}
std::string Session::reason(double now) const
{
  if (!connected_) {return "disconnected";}
  if (!online(now)) {return "waiting_for_frame_or_connection_timeout";}
  if (!eligible_) {return reason_;}
  if (now - last_good_ > limits_.navigation_timeout_s) {return "navigation_stream_timeout";}
  if (last_age_ + (now - last_good_) * 1000 > limits_.max_age_ms) {return "measurement_expired";}
  return "ok";
}
Observation Session::observe(const Frame & f, double now)
{
  if (!connected_) {throw std::logic_error("observe requires connected session");}
  last_frame_ = now;
  Observation result;
  const std::string invalid = invalid_reason(f);
  // 只用数值有效的导航快照判断测量时间回退，初始化状态帧的零不能触发误报。
  const bool time_back = invalid.empty() && measurement_time_ && f.state_time_s < *measurement_time_;
  uint32_t delta = sequence_ ? f.sequence - *sequence_ : 1;
  if ((sequence_ && delta >= 0x80000000u) || time_back) {
    new_epoch(); ++restarts; result.restarted = true; delta = 1;
  }
  const bool duplicate_sequence = sequence_ && delta == 0;
  if (sequence_ && delta > 1) {missing += uint64_t(delta) - 1;}
  sequence_ = f.sequence;
  auto reject = [&](const std::string & why) {
      eligible_ = false; reason_ = why; result.reason = why; ++rejected; return result;
    };
  if (!invalid.empty()) {return reject(invalid);}
  if (!limits_.allow_simulated && (f.status & SIMULATED)) {return reject("simulated_data_disabled");}
  if (f.age_ms == 65535 || f.age_ms > limits_.max_age_ms) {return reject("measurement_expired_or_unknown");}
  if (duplicate_sequence || (measurement_time_ && f.state_time_s == *measurement_time_)) {
    ++duplicates; result.reason = "duplicate_measurement"; return result;
  }
  measurement_time_ = f.state_time_s; last_good_ = now; last_age_ = f.age_ms;
  eligible_ = true; reason_ = "ok"; result.publish = true; result.reason = "ok";
  return result;
}
}
