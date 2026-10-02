#pragma once
#include "terramind_protocol/codec.hpp"
namespace terramind::protocol
{
// 有状态流式分帧器：一次输入可以是半帧、多帧或噪声；剩余字节留待下次。
// 每个串口连接独占一个实例，跨线程使用时由调用者同步。
class FrameParser
{
public:
  std::vector<Frame> feed(const uint8_t * data, size_t size);
  std::vector<Frame> feed(const Bytes & data)
  {
    return feed(data.data(), data.size());
  }
  void reset()
  {
    // 重连时清除半帧；累计错误计数保留，用于长期诊断。
    buffer_.clear();
  }
  uint64_t errors() const
  {
    // 统计失步/校验失败时丢弃字节的次数，不等同于坏帧总数。
    return errors_;
  }
  size_t buffered() const
  {
    return buffer_.size();
  }

private:
  Bytes buffer_;
  uint64_t errors_ = 0;
  void parse(std::vector<Frame> & out);
};
}
