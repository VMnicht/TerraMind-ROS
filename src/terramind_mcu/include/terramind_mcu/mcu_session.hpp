#pragma once
#include "terramind_protocol/protocol_types.hpp"
#include <deque>
#include <string>
namespace terramind::mcu
{
class Session
{
public:
  explicit Session(double timeout = 0.150);
  void reset(uint64_t id, double now);
  void sent(uint16_t seq, const protocol::Control & c, double now);
  // False means a duplicate/out-of-order status that must not renew freshness.
  bool observe(const protocol::State & s, uint16_t frame_seq, double now);
  bool expired(double now) const;
  bool ready = false, rebooted = false;
  uint64_t connection_id = 0;
  uint16_t capabilities = protocol::CURRENT_CAPABILITIES;
  std::string reason = "disconnected";

private:
  struct Sent
  {
    uint16_t seq;
    double time;
    bool disabled;
  };
  std::deque<Sent> sent_;
  double timeout_, last_rx_ = 0, started_ = 0;
  bool have_state_ = false;
  uint16_t state_seq_ = 0;
  uint32_t uptime_ = 0;
};
}
