#pragma once
#include "terramind_protocol/protocol_types.hpp"
#include <deque>
#include <string>
namespace terramind::mcu
{
// 跟踪串口连接的零指令握手和状态新鲜度；时间参数均使用单调时钟秒数。
// 由后端线程独占，不在这里执行串口读写或发布 ROS 消息。
class Session
{
public:
  explicit Session(double timeout = 0.150);
  void reset(uint64_t id, double now);
  void sent(uint16_t seq, const protocol::Control & c, double now);
  // 返回 false 表示重复/倒序状态，调用者不得据此发布“新鲜”反馈。
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
