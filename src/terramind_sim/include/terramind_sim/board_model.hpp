#pragma once
#include "terramind_protocol/codec.hpp"
#include "terramind_sim/differential_drive_model.hpp"
namespace terramind::sim
{
class BoardModel
{
public:
  explicit BoardModel(
    Geometry geometry = {},
    uint16_t capabilities = protocol::CURRENT_CAPABILITIES,
    double acceleration = 0.7, double angular_acceleration = 4.0);
  void reset(double now);
  void receive(const protocol::Frame & frame, double now);
  void accept(const protocol::Control & c, uint16_t seq, double now);
  void step(double now);
  void add_rx_errors(uint64_t count);
  const protocol::State & state()const
  {
    return state_;
  }
  const DifferentialDriveModel & drive()const
  {
    return drive_;
  }

private:
  void apply(const protocol::Control & c, uint16_t seq, double now);
  void targets();
  protocol::State state_;
  protocol::Control accepted_;
  DifferentialDriveModel drive_;
  uint16_t capabilities_;
  double boot_ = 0, last_step_ = 0, last_accepted_ = 0;
  bool owned_ = false;
};
}
