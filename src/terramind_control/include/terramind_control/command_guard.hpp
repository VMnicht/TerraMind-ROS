#pragma once
#include "terramind_protocol/codec.hpp"
#include <string>
namespace terramind::control
{
double steady_seconds();
uint64_t new_connection_id();
class CommandGuard
{
public:
  explicit CommandGuard(double timeout = 0.150);
  void reset(uint64_t connection);
  bool receive(
    const protocol::Control & command, uint64_t connection, double now, bool valid_stamp,
    uint16_t capabilities);
  protocol::Control sample(double now, bool link_ready);
  void trip(const std::string & reason);
  const std::string & reason() const
  {
    return reason_;
  }

private:
  double timeout_, received_ = -1;
  uint64_t connection_ = 0;
  bool may_enable_ = false;
  protocol::Control latest_ = protocol::stopped();
  std::string reason_ = "no command";
};
}
