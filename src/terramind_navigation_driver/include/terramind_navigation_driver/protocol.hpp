#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace terramind::navigation
{
constexpr size_t FRAME_SIZE = 80;
constexpr uint8_t NAV_VALID = 1, NAV_FAULT = 2, TIME_LOCKED = 4,
  GNSS_RECENT = 8, SIMULATED = 16;
struct Frame
{
  uint8_t status = 0;
  uint32_t sequence = 0;
  double state_time_s = 0, latitude_rad = 0, longitude_rad = 0;
  float height_m = 0;
  std::array<float, 3> velocity_ned{}, rpy{};
  std::array<float, 4> quaternion_wxyz{};
  uint16_t age_ms = 65535;
  uint8_t rtk_state() const {return (status >> 5) & 7;}
};
uint16_t crc16(const uint8_t * data, size_t size);
// 只校验线格式；无效状态帧中的全零四元数也是合法报文。
Frame decode(const uint8_t * data, size_t size);
// 空字符串表示数值和导航标志可用；年龄/重复历元由 Session 检查。
std::string invalid_reason(const Frame & frame);
struct ParserStats
{
  uint64_t frames = 0, crc_errors = 0, version_errors = 0, timeouts = 0, discarded = 0;
};
class Parser
{
public:
  explicit Parser(double timeout_s = 0.1);
  std::vector<Frame> feed(const uint8_t * data, size_t size, double now);
  std::vector<Frame> tick(double now) {return feed(nullptr, 0, now);}
  void reset();
  size_t buffered() const {return buffer_.size();}
  const ParserStats & stats() const {return stats_;}
private:
  struct Byte {uint8_t value; double received;};
  void parse(double now, std::vector<Frame> & out);
  void discard();
  double timeout_;
  std::deque<Byte> buffer_;
  ParserStats stats_;
};
}
