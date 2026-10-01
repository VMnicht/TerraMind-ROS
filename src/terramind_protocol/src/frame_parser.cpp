#include "terramind_protocol/frame_parser.hpp"
namespace terramind::protocol
{
namespace
{
size_t length(const Bytes & b, size_t p)
{
  return size_t(b[p + 6]) | (size_t(b[p + 7]) << 8);
}
bool complete_valid(const Bytes & b, size_t p)
{
  if (p + 8 > b.size() || b[p] != 0xfc || b[p + 1] != 0xfb || b[p + 2] != 1) {
    return false;
  }
  size_t n = length(b, p);
  if (n > 256 || p + n + 12 > b.size()) {
    return false;
  }
  return b[p + n + 10] == 0xfd && b[p + n + 11] == 0xfe &&
         crc16(b.data() + p + 2, n + 6) == (uint16_t(b[p + n + 8]) | (uint16_t(b[p + n + 9]) << 8));
}
}
std::vector<Frame> FrameParser::feed(const uint8_t * data, size_t size)
{
  std::vector<Frame> out;
  // Parsing incrementally keeps memory bounded even for an arbitrarily large read.
  for (size_t i = 0; i < size; ++i) {
    buffer_.push_back(data[i]);
    parse(out);
  }
  return out;
}
void FrameParser::parse(std::vector<Frame> & out)
{
  while (buffer_.size() >= 2) {
    if (buffer_[0] != 0xfc || buffer_[1] != 0xfb) {
      buffer_.erase(buffer_.begin());
      ++errors_;
      continue;
    }
    if (buffer_.size() < 8) {
      return;
    }
    size_t n = length(buffer_, 0);
    if (buffer_[2] != 1 || n > 256) {
      buffer_.erase(buffer_.begin());
      ++errors_;
      continue;
    }
    if (buffer_.size() < n + 12) {
      // A TLV may itself contain header-like bytes, including a valid nested frame.
      // Wait for the bounded candidate before deciding whether to resynchronize.
      return;
    }
    if (!complete_valid(buffer_, 0)) {
      buffer_.erase(buffer_.begin());
      ++errors_;
      continue;
    }
    out.push_back(
      {buffer_[3], uint16_t(buffer_[4] | uint16_t(buffer_[5]) << 8),
        Bytes(buffer_.begin() + 8, buffer_.begin() + 8 + n)});
    buffer_.erase(buffer_.begin(), buffer_.begin() + n + 12);
  }
}
}
