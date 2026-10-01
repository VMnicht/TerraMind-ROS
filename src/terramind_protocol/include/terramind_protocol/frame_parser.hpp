#pragma once
#include "terramind_protocol/codec.hpp"
namespace terramind::protocol
{
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
    buffer_.clear();
  }
  uint64_t errors() const
  {
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
