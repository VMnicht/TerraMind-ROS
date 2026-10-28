#pragma once
#include "terramind_protocol/protocol_types.hpp"
#include <stdexcept>

namespace terramind::mcu
{
// Only the serial boundary changes convention; ROS uses positive CCW.
// Wheel RPM remains in the firmware's original motor convention.
class AngularMapping
{
public:
  explicit AngularMapping(int sign = 1) : sign_(sign)
  {
    if (sign != 1 && sign != -1) {
      throw std::invalid_argument("angular_command_sign must be +1 or -1");
    }
  }
  protocol::Control to_board(protocol::Control c) const
  {
    c.angular *= sign_;
    return c;
  }
  protocol::State to_ros(protocol::State s) const
  {
    s.chassis.angular *= sign_;
    return s;
  }
private:
  int sign_;
};
}
