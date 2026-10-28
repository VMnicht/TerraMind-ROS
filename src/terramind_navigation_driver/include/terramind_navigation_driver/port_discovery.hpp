#pragma once
#include "terramind_navigation_driver/protocol.hpp"
#include "terramind_transport/port_discovery.hpp"
#include <functional>

namespace terramind::navigation
{
// 仅判断协议身份和报文序号推进，不要求导航解有效或 RTK 固定。
class NavigationProbe
{
public:
  bool observe(const Frame & frame);
  bool matched() const {return count_ >= 3;}
private:
  uint32_t sequence_ = 0;
  unsigned count_ = 0;
};

// keep_running 在扫描期间反复调用，可同时负责驱动状态发布和取消。
transport::DiscoveryResult discover_navigation_board(
  const std::vector<std::string> & patterns, double listen_seconds,
  const std::function<bool()> & keep_running);
}
