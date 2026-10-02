#pragma once
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace terramind::protocol
{
using Bytes = std::vector<uint8_t>;
constexpr uint8_t CONTROL = 0x01, STATUS = 0x81;
// 能力位 bit0～bit5：底盘、左播撒、右播撒、刀盘、升降、喷洒。
// 当前板卡为 0x002F（不含升降）；运行时以状态帧上报的能力为准。
constexpr uint16_t CURRENT_CAPABILITIES = 0x002f;
constexpr double FIRMWARE_TIMEOUT = 0.250;
enum class Result : uint8_t { OK=0, BAD_FRAME=1, BAD_VALUE=2, UNSUPPORTED=3, TIMEOUT=4 };
struct ProtocolError : std::runtime_error
{
  Result result;
  ProtocolError(Result r, const char * message)
  : std::runtime_error(message), result(r)
  {
  }
};
struct Actuator
{
  // value 的单位由装置决定：播撒 RPM、刀盘/喷洒 %、升降 mm。
  bool on = false;
  float value = 0;
};
struct Control
{
  // 完整目标快照；left/right 是播撒装置，不是底盘左右轮。
  bool enable = false, stop = false;
  float linear = 0, angular = 0;
  Actuator left, right, mower, lift, sprayer;
};
struct System
{
  uint32_t uptime_ms = 0;
  uint16_t last_command_seq = 0;
  uint8_t result = 0, mode = 0;
  uint16_t faults = 0, capabilities = CURRENT_CAPABILITIES, command_age_ms = 65535,
    rx_error_count = 0;
};
struct Chassis
{
  // linear/angular 为 m/s、rad/s，轮速为 RPM；真实反馈轴侧尚待固件确认。
  float linear = 0, angular = 0, left_target = 0, right_target = 0, left_actual = 0,
    right_actual = 0;
};
struct Spreader
{
  bool on = false;
  float target = 0, actual = 0, servo_angle = 0;
};
struct Feedback
{
  bool on = false;
  float target = 0, actual = 0;
  bool valid = false;
};
struct State
{
  System system;
  Chassis chassis;
  Spreader left, right;
  Actuator mower;
  Feedback lift, sprayer;
};
struct Frame
{
  // 已通过帧结构和 CRC 校验的载荷；TLV 语义由 decode_* 再检查。
  uint8_t type = 0;
  uint16_t seq = 0;
  Bytes payload;
};
Control stopped();
// 越界、非有限值或启用不支持的装置会抛出 ProtocolError，不裁剪目标。
void validate(const Control & c, uint16_t capabilities = CURRENT_CAPABILITIES);
}
