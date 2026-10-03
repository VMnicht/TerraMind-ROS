#include "terramind_navigation_driver/protocol.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace terramind::navigation
{
namespace
{
uint64_t little(const uint8_t * p, size_t n)
{
  uint64_t v = 0;
  for (size_t i = 0; i < n; ++i) {v |= uint64_t(p[i]) << (8 * i);}
  return v;
}
float f32(const uint8_t * p)
{
  uint32_t bits = static_cast<uint32_t>(little(p, 4));
  float result;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}
double f64(const uint8_t * p)
{
  uint64_t bits = little(p, 8);
  double result;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}
static_assert(sizeof(float) == 4 && sizeof(double) == 8 &&
  std::numeric_limits<float>::is_iec559 && std::numeric_limits<double>::is_iec559);
}
uint16_t crc16(const uint8_t * p, size_t n)
{
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= uint16_t(p[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}
Frame decode(const uint8_t * p, size_t size)
{
  if (size != FRAME_SIZE || !p || p[0] != 0xaa || p[1] != 0x55 || p[2] != 1) {
    throw std::invalid_argument("invalid navigation frame header/size/version");
  }
  if (crc16(p + 2, 76) != little(p + 78, 2)) {
    throw std::invalid_argument("invalid navigation CRC");
  }
  Frame f;
  f.status = p[3]; f.sequence = static_cast<uint32_t>(little(p + 4, 4));
  f.state_time_s = f64(p + 8); f.latitude_rad = f64(p + 16);
  f.longitude_rad = f64(p + 24); f.height_m = f32(p + 32);
  for (size_t i = 0; i < 3; ++i) {
    f.velocity_ned[i] = f32(p + 36 + i * 4);
    f.rpy[i] = f32(p + 48 + i * 4);
  }
  for (size_t i = 0; i < 4; ++i) {f.quaternion_wxyz[i] = f32(p + 60 + i * 4);}
  f.age_ms = static_cast<uint16_t>(little(p + 76, 2));
  return f;
}
std::string invalid_reason(const Frame & f)
{
  if ((f.status & NAV_VALID) && (f.status & NAV_FAULT)) {return "conflicting_flags";}
  if (f.status & NAV_FAULT) {return "navigation_fault";}
  if (!(f.status & NAV_VALID)) {return "navigation_invalid";}
  constexpr double pi = 3.14159265358979323846;
  if (!std::isfinite(f.state_time_s) || !std::isfinite(f.latitude_rad) ||
    !std::isfinite(f.longitude_rad) || !std::isfinite(f.height_m)) {return "nonfinite_position_time";}
  if (std::abs(f.latitude_rad) > pi / 2 || std::abs(f.longitude_rad) > pi) {
    return "coordinate_range";
  }
  for (float v : f.velocity_ned) {if (!std::isfinite(v)) {return "nonfinite_velocity";}}
  for (float v : f.rpy) {if (!std::isfinite(v)) {return "nonfinite_attitude";}}
  double norm = 0;
  for (float v : f.quaternion_wxyz) {
    if (!std::isfinite(v)) {return "nonfinite_quaternion";}
    norm += double(v) * v;
  }
  if (std::abs(norm - 1) > 0.001) {return "quaternion_norm";}
  return {};
}
Parser::Parser(double timeout_s) : timeout_(timeout_s)
{
  if (!std::isfinite(timeout_s) || timeout_s <= 0) {throw std::invalid_argument("invalid parser timeout");}
}
void Parser::reset() {buffer_.clear();}
void Parser::discard() {buffer_.pop_front(); ++stats_.discarded;}
std::vector<Frame> Parser::feed(const uint8_t * p, size_t size, double now)
{
  std::vector<Frame> out;
  // 在追加新字节前过期旧候选，避免慢速残包靠后续字节无限续期。
  parse(now, out);
  for (size_t i = 0; i < size; ++i) {
    buffer_.push_back({p[i], now});
    parse(now, out);
  }
  return out;
}
void Parser::parse(double now, std::vector<Frame> & out)
{
  while (!buffer_.empty()) {
    if (buffer_[0].value != 0xaa) {discard(); continue;}
    if (buffer_.size() >= 2 && buffer_[1].value != 0x55) {discard(); continue;}
    if (buffer_.size() < FRAME_SIZE) {
      if (now - buffer_[0].received >= timeout_) {++stats_.timeouts; discard(); continue;}
      return;
    }
    if (buffer_[2].value != 1) {++stats_.version_errors; discard(); continue;}
    std::array<uint8_t, FRAME_SIZE> bytes{};
    for (size_t i = 0; i < FRAME_SIZE; ++i) {bytes[i] = buffer_[i].value;}
    if (crc16(bytes.data() + 2, 76) != little(bytes.data() + 78, 2)) {
      ++stats_.crc_errors; discard(); continue;
    }
    out.push_back(decode(bytes.data(), bytes.size())); ++stats_.frames;
    for (size_t i = 0; i < FRAME_SIZE; ++i) {buffer_.pop_front();}
  }
}
}
