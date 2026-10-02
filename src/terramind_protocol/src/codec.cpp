#include "terramind_protocol/codec.hpp"
#include <cmath>
#include <cstring>
#include <map>
#include <limits>
namespace terramind::protocol
{
namespace
{
static_assert(
  sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
  "IEEE-754 float32 required");
void u16(Bytes & b, uint16_t v)
{
  b.push_back(v & 255);
  b.push_back(v >> 8);
}
void u32(Bytes & b, uint32_t v)
{
  for (int i = 0; i < 4; ++i) {
    b.push_back((v >> (i * 8)) & 255);
  }
}
void f32(Bytes & b, float v)
{
  // 先复制 IEEE-754 位模式，再按小端序逐字节写入，避免结构体布局和别名问题。
  uint32_t bits;
  std::memcpy(&bits, &v, 4);
  u32(b, bits);
}
void block(Bytes & b, uint8_t id, const Bytes & d)
{
  b.push_back(id);
  b.push_back(d.size());
  b.insert(b.end(), d.begin(), d.end());
}
void actuator(Bytes & b, uint8_t id, const Actuator & a)
{
  Bytes d{uint8_t(a.on)};
  f32(d, a.value);
  block(b, id, d);
}
struct Reader
{
  const Bytes & b;
  size_t p = 0;
  uint8_t u8()
  {
    if (p >= b.size()) {
      throw ProtocolError(Result::BAD_FRAME, "truncated field");
    }
    return b[p++];
  }
  uint16_t u16()
  {
    auto a = u8();
    return uint16_t(a) | uint16_t(u8()) << 8;
  }
  uint32_t u32()
  {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      v |= uint32_t(u8()) << (i * 8);
    }
    return v;
  }
  bool boolean()
  {
    auto v = u8();
    if (v > 1) {
      throw ProtocolError(Result::BAD_FRAME, "invalid boolean");
    }
    return v != 0;
  }
  float f32()
  {
    auto bits = u32();
    float v;
    std::memcpy(&v, &bits, 4);
    if (!std::isfinite(v)) {
      throw ProtocolError(Result::BAD_VALUE, "nonfinite float");
    }
    return v;
  }
};
using Blocks = std::map<uint8_t, Bytes>;
Blocks blocks(const Frame & f, uint8_t expected, const std::array<size_t, 7> & lens)
{
  // 七个已知 TLV 必须各出现一次；未知 ID 按长度跳过，允许未来协议扩展。
  if (f.type != expected) {
    throw ProtocolError(Result::BAD_FRAME, "wrong frame type");
  }
  const std::array<uint8_t, 7> ids{1, 0x10, 0x20, 0x21, 0x30, 0x40, 0x50};
  Blocks out;
  for (size_t p = 0; p < f.payload.size(); ) {
    if (p + 2 > f.payload.size()) {
      throw ProtocolError(Result::BAD_FRAME, "truncated TLV");
    }
    auto id = f.payload[p++];
    size_t n = f.payload[p++];
    if (p + n > f.payload.size()) {
      throw ProtocolError(Result::BAD_FRAME, "truncated TLV data");
    }
    for (size_t i = 0; i < ids.size(); ++i) {
      if (id == ids[i]) {
        if (out.count(id) || n != lens[i]) {
          throw ProtocolError(Result::BAD_FRAME, "duplicate block or wrong length");
        }
        out[id] = Bytes(f.payload.begin() + p, f.payload.begin() + p + n);
      }
    }
    p += n;
  }
  if (out.size() != 7) {
    throw ProtocolError(Result::BAD_FRAME, "missing required block");
  }
  return out;
}
void range(float v, float lo, float hi)
{
  if (!std::isfinite(v) || v < lo || v > hi) {
    throw ProtocolError(Result::BAD_VALUE, "target outside protocol range");
  }
}
}
Control stopped()
{
  Control c;
  c.stop = true;
  return c;
}
void validate(const Control & c, uint16_t capabilities)
{
  // 即使装置关闭也校验数值范围；整帧合法后才能更新目标或续期看门狗。
  range(c.linear, -0.35f, 0.35f);
  range(c.angular, -2, 2);
  range(c.left.value, -500, 500);
  range(c.right.value, -500, 500);
  range(c.mower.value, 0, 100);
  range(c.lift.value, 0, 1000);
  range(c.sprayer.value, 0, 100);
  if (c.enable && !c.stop) {
    // 停机快照不要求设备具备相应能力；只有实际启用的装置参与能力检查。
    uint16_t required = (c.linear != 0 || c.angular != 0) ? 1 : 0;
    if (c.left.on) {
      required |= 2;
    }
    if (c.right.on) {
      required |= 4;
    }
    if (c.mower.on) {
      required |= 8;
    }
    if (c.lift.on) {
      required |= 16;
    }
    if (c.sprayer.on) {
      required |= 32;
    }
    if (required & ~capabilities) {
      throw ProtocolError(Result::UNSUPPORTED, "unsupported actuator");
    }
  }
}
Bytes encode_frame(const Frame & f)
{
  if (f.payload.size() > 256) {
    throw ProtocolError(Result::BAD_FRAME, "payload too long");
  }
  Bytes b{0xfc, 0xfb, 1, f.type};
  u16(b, f.seq);
  u16(b, f.payload.size());
  b.insert(b.end(), f.payload.begin(), f.payload.end());
  u16(b, crc16(b.data() + 2, b.size() - 2));
  b.push_back(0xfd);
  b.push_back(0xfe);
  return b;
}
Bytes encode_control(const Control & c, uint16_t seq)
{
  // 编码器只验证格式和范围；具体板卡能力由控制管理/后端按状态反馈判断。
  validate(c, 0xffff);
  Bytes b;
  block(b, 1, Bytes{uint8_t(c.enable | (c.stop << 1))});
  Bytes d;
  f32(d, c.linear);
  f32(d, c.angular);
  block(b, 0x10, d);
  actuator(b, 0x20, c.left);
  actuator(b, 0x21, c.right);
  actuator(b, 0x30, c.mower);
  actuator(b, 0x40, c.lift);
  actuator(b, 0x50, c.sprayer);
  return encode_frame({CONTROL, seq, b});
}
Control decode_control(const Frame & f)
{
  auto b = blocks(f, CONTROL, {1, 8, 5, 5, 5, 5, 5});
  Control c;
  auto flags = b.at(1)[0];
  if (flags & ~3) {
    throw ProtocolError(Result::BAD_FRAME, "reserved flags");
  }
  c.enable = flags & 1;
  c.stop = flags & 2;
  Reader r{b.at(0x10)};
  c.linear = r.f32();
  c.angular = r.f32();
  auto a = [&](uint8_t id) {
      Reader v{b.at(id)};
      return Actuator{v.boolean(), v.f32()};
    };
  c.left = a(0x20);
  c.right = a(0x21);
  c.mower = a(0x30);
  c.lift = a(0x40);
  c.sprayer = a(0x50);
  validate(c, 0xffff);
  return c;
}
Bytes encode_state(const State & s, uint16_t seq)
{
  Bytes b, d;
  const auto & x = s.system;
  u32(d, x.uptime_ms);
  u16(d, x.last_command_seq);
  d.push_back(x.result);
  d.push_back(x.mode);
  u16(d, x.faults);
  u16(d, x.capabilities);
  u16(d, x.command_age_ms);
  u16(d, x.rx_error_count);
  block(
    b,
    1,
    d);
  d.clear();
  for (auto v:
    {s.chassis.linear, s.chassis.angular, s.chassis.left_target, s.chassis.right_target,
      s.chassis.left_actual, s.chassis.right_actual})
  {
    f32(d, v);
  }
  block(b, 0x10, d);
  auto spread = [&](uint8_t id, const Spreader & v) {
      Bytes q{uint8_t(v.on)};
      f32(q, v.target);
      f32(q, v.actual);
      f32(q, v.servo_angle);
      block(b, id, q);
    };
  spread(0x20, s.left);
  spread(0x21, s.right);
  actuator(b, 0x30, s.mower);
  auto feedback = [&](uint8_t id, const Feedback & v) {
      Bytes q{uint8_t(v.on)};
      f32(q, v.target);
      f32(q, v.actual);
      q.push_back(v.valid);
      block(b, id, q);
    };
  feedback(0x40, s.lift);
  feedback(0x50, s.sprayer);
  return encode_frame({STATUS, seq, b});
}
State decode_state(const Frame & f)
{
  auto b = blocks(f, STATUS, {16, 24, 13, 13, 5, 10, 10});
  State s;
  auto & x = s.system;
  Reader r{b.at(1)};
  x.uptime_ms = r.u32();
  x.last_command_seq = r.u16();
  x.result = r.u8();
  x.mode = r.u8();
  if (x.result > 4 || x.mode > 2) {
    throw ProtocolError(Result::BAD_FRAME, "invalid system enum");
  }
  x.faults = r.u16();
  x.capabilities = r.u16();
  x.command_age_ms = r.u16();
  x.rx_error_count = r.u16();
  Reader c{b.at(0x10)};
  s.chassis = {c.f32(), c.f32(), c.f32(), c.f32(), c.f32(), c.f32()};
  auto spread = [&](uint8_t id) {
      Reader q{b.at(id)};
      return Spreader{q.boolean(), q.f32(), q.f32(), q.f32()};
    };
  s.left = spread(0x20);
  s.right = spread(0x21);
  Reader m{b.at(0x30)};
  s.mower = {m.boolean(), m.f32()};
  auto feedback = [&](uint8_t id) {
      Reader q{b.at(id)};
      return Feedback{q.boolean(), q.f32(), q.f32(), q.boolean()};
    };
  s.lift = feedback(0x40);
  s.sprayer = feedback(0x50);
  return s;
}
}
