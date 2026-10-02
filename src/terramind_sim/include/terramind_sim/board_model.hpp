#pragma once
#include "terramind_protocol/codec.hpp"
#include "terramind_sim/differential_drive_model.hpp"
namespace terramind::sim
{
// 两种模拟后端共用的板卡行为模型：接管、拒绝命令、看门狗和装置反馈。
// 不依赖 ROS，时间由调用者注入；目标变化与物理速度变化分别建模。
class BoardModel
{
public:
  explicit BoardModel(
    Geometry geometry = {},
    uint16_t capabilities = protocol::CURRENT_CAPABILITIES,
    double acceleration = 0.7, double angular_acceleration = 4.0);
  void reset(double now);
  // 字节级模拟器入口：完整控制帧还需通过 TLV 解码和语义检查。
  void receive(const protocol::Frame & frame, double now);
  // 直接仿真入口：跳过串口编解码，但仍执行同一能力/范围/看门狗逻辑。
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
