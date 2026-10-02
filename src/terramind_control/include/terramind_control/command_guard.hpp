#pragma once
#include "terramind_protocol/codec.hpp"
#include <string>
namespace terramind::control
{
double steady_seconds();
// 随机非零连接代次，用于隔离重连前的控制消息；不代表设备身份认证。
uint64_t new_connection_id();
// 后端独立保护层：即使控制管理节点退出，也会按有效期撤销输出。
class CommandGuard
{
public:
  explicit CommandGuard(double timeout = 0.150);
  void reset(uint64_t connection);
  // now 是折算到单调时钟的指令源时间；传输延迟已从有效期中扣除。
  bool receive(
    const protocol::Control & command, uint64_t connection, double now, bool valid_stamp,
    uint16_t capabilities);
  protocol::Control sample(double now, bool link_ready);
  // 故障锁定后先接收停机/未使能快照，才允许后续新使能快照生效。
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
