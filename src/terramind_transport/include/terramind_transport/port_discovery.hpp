#pragma once
#include "terramind_transport/serial_port.hpp"
#include <memory>
#include <string>
#include <vector>

namespace terramind::transport
{
// 按配置顺序枚举字符设备，按设备号去重（包括符号链接别名）。
std::vector<std::string> serial_candidates(const std::vector<std::string> & patterns);

// 同一用户的发现器共享扫描锁；已连接设备仍由 SerialPort 的设备锁保护。
// 非阻塞尝试：忙时交还驱动的重试循环，不长期占用一部分候选设备。
class DiscoveryLock
{
public:
  DiscoveryLock();
  ~DiscoveryLock();
  DiscoveryLock(const DiscoveryLock &) = delete;
  DiscoveryLock & operator=(const DiscoveryLock &) = delete;
  bool acquired() const {return fd_ >= 0;}
private:
  int fd_ = -1;
};

struct DiscoveryResult
{
  std::unique_ptr<SerialPort> port;
  std::string device;
  std::vector<std::string> matches, errors;
  size_t candidate_count = 0;
};
}
