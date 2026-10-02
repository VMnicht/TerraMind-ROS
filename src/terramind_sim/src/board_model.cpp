#include "terramind_sim/board_model.hpp"
#include <algorithm>
#include <cmath>
namespace terramind::sim
{
namespace tp = protocol;
BoardModel::BoardModel(Geometry g, uint16_t capabilities, double a, double aa)
: drive_(g, a, aa), capabilities_(capabilities)
{
  reset(0);
}
void BoardModel::reset(double now)
{
  boot_ = last_step_ = last_accepted_ = now;
  owned_ = false;
  accepted_ = {};
  state_ = {};
  state_.system.capabilities = capabilities_;
  drive_.reset();
}
void BoardModel::add_rx_errors(uint64_t n)
{
  state_.system.rx_error_count = std::min<uint64_t>(
    65535, uint64_t(
      state_.system.rx_error_count) + n);
}
void BoardModel::receive(const tp::Frame & f, double now)
{
  if (f.type != tp::CONTROL) {
    return;
  }
  step(now);
  state_.system.last_command_seq = f.seq;
  // 合法帧中的语义错误仍报告序号和拒绝原因，但不改变目标或续期看门狗。
  try {
    apply(tp::decode_control(f), f.seq, now);
  } catch (const tp::ProtocolError & e) {
    state_.system.result = uint8_t(e.result);
    add_rx_errors(1);
  }
}
void BoardModel::accept(const tp::Control & c, uint16_t seq, double now)
{
  step(now);
  state_.system.last_command_seq = seq;
  try {
    apply(c, seq, now);
  } catch (const tp::ProtocolError & e) {
    state_.system.result = uint8_t(e.result);
    add_rx_errors(1);
  }
}
void BoardModel::apply(const tp::Control & c, uint16_t seq, double now)
{
  tp::validate(c, capabilities_);
  // 只有整帧通过检查才接管 USART3；停机后保持接管，不退回其他控制口。
  accepted_ = c;
  owned_ = true;
  last_accepted_ = now;
  state_.system.last_command_seq = seq;
  state_.system.result = 0;
  state_.system.command_age_ms = 0;
  state_.system.faults &= ~1;
  targets();
}
void BoardModel::targets()
{
  bool active = owned_ && accepted_.enable && !accepted_.stop && !(state_.system.faults & 1);
  state_.system.mode = owned_ ? (active ? 1 : 2) : 0;
  tp::Control c = active ? accepted_ : tp::Control{};
  // 这里 linear/angular 表示目标值；真实固件同名反馈字段的含义需独立核对。
  state_.chassis.linear = c.linear;
  state_.chassis.angular = c.angular;
  state_.chassis.left_target = drive_.left_rpm(c.linear, c.angular);
  state_.chassis.right_target = drive_.right_rpm(c.linear, c.angular);
  state_.left.on = c.left.on;
  state_.left.target = c.left.on ? c.left.value : 0;
  state_.right.on = c.right.on;
  state_.right.target = c.right.on ? c.right.value : 0;
  state_.mower = {c.mower.on, c.mower.on ? c.mower.value : 0};
  state_.lift.on = c.lift.on;
  state_.lift.target = c.lift.on ? c.lift.value : 0;
  state_.lift.valid = capabilities_ & 16;
  state_.sprayer.on = c.sprayer.on;
  state_.sprayer.target = c.sprayer.on ? c.sprayer.value : 0;
  state_.sprayer.valid = capabilities_ & 32;
}
void BoardModel::step(double now)
{
  if (!std::isfinite(now) || now < last_step_) {
    throw std::invalid_argument("simulation clock must be monotonic; reset explicitly");
  }
  // 大时间步跨过看门狗期限时，先推进到期限，再计算停机后的减速过程。
  // 否则会把停止目标错误地应用到超时发生前的整段时间。
  if (owned_ && !(state_.system.faults & 1) && last_step_ < last_accepted_ + tp::FIRMWARE_TIMEOUT &&
    now > last_accepted_ + tp::FIRMWARE_TIMEOUT)
  {
    step(last_accepted_ + tp::FIRMWARE_TIMEOUT);
  }
  double dt = now - last_step_;
  last_step_ = now;
  state_.system.uptime_ms = uint32_t(uint64_t((now - boot_) * 1000));
  double age = now - last_accepted_;
  state_.system.command_age_ms =
    owned_ ? uint16_t(std::min(65535.0, std::max(0.0, age * 1000))) : 65535;
  if (owned_ && age > tp::FIRMWARE_TIMEOUT) {
    state_.system.faults |= 1;
  }
  targets();
  drive_.step(state_.chassis.linear, state_.chassis.angular, dt);
  state_.chassis.left_actual = drive_.left_rpm(drive_.linear, drive_.angular);
  state_.chassis.right_actual = drive_.right_rpm(drive_.linear, drive_.angular);
  double alpha = 1 - std::exp(-dt / 0.15);
  // 作业反馈使用 0.15 s 一阶响应近似；只是模型输出，不等价于真实传感器。
  state_.left.actual += (state_.left.target - state_.left.actual) * alpha;
  state_.right.actual += (state_.right.target - state_.right.actual) * alpha;
  state_.lift.actual += (state_.lift.target - state_.lift.actual) * alpha;
  state_.sprayer.actual += (state_.sprayer.target - state_.sprayer.actual) * alpha;
}
}
