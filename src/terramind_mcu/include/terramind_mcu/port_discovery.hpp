#pragma once
#include "terramind_protocol/protocol_types.hpp"
#include "terramind_transport/serial_port.hpp"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace terramind::mcu
{
// 按 glob 配置顺序枚举，同一设备的 by-id 和 ttyUSB 等别名只保留一次。
std::vector<std::string> serial_candidates(const std::vector<std::string> & patterns);

class StatusProbe
{
public:
  // 只接收 FrameParser 已校验 CRC 的帧；需连续状态的序号和 uptime 都前进。
  // 正常整数回绕允许通过；重复状态不能续期。返回 true 表示新推进的样本。
  bool observe(const protocol::Frame & frame);
  bool matched() const {return matched_;}

private:
  bool have_state_ = false, matched_ = false;
  uint16_t sequence_ = 0;
  uint32_t uptime_ = 0;
};

struct DiscoveryResult
{
  // 识别后转移已打开且加锁的句柄，握手前不再次按路径打开设备。
  std::unique_ptr<transport::SerialPort> port;
  std::string device;
  std::vector<std::string> matches;
  std::vector<std::string> errors;
  size_t candidate_count = 0;
};

// 被动识别：会打开并配置候选串口，但不向任何候选端口写入探测字节。
// 零个或多个匹配均不选择，避免把控制指令发给导航板或另一块控制板。
DiscoveryResult discover_control_board(
  const std::vector<std::string> & patterns, double listen_seconds,
  const std::function<bool()> & keep_running);
}
