// 协议回归：黄金帧、CRC、任意分片、失步恢复、TLV 兼容和非法值拒绝。
#include "terramind_protocol/frame_parser.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace terramind::protocol;
Frame unpack(const Bytes & b)
{
  FrameParser p;
  auto f = p.feed(b);
  if (f.size() != 1) {
    throw std::runtime_error("not one frame");
  }
  return f[0];
}
TEST(Protocol, CrcAndGoldenZero) {
  const std::string s = "123456789";
  EXPECT_EQ(crc16(reinterpret_cast<const uint8_t *>(s.data()), s.size()), 0x31c3);
  Bytes golden = {0xfc, 0xfb, 1, 1, 0, 0, 0x30, 0, 1, 1, 0, 0x10, 8, 0, 0, 0, 0, 0, 0, 0, 0,
    0x20, 5, 0, 0, 0, 0, 0, 0x21, 5, 0, 0, 0, 0, 0, 0x30, 5, 0, 0, 0, 0, 0, 0x40, 5, 0, 0, 0, 0, 0,
    0x50, 5, 0, 0, 0, 0, 0, 0xbb, 0x1e, 0xfd, 0xfe};
  EXPECT_EQ(encode_control({}, 0), golden);
}
TEST(Protocol, ChunksNoiseAndBackToBack) {
  Control c;
  c.enable = true;
  c.linear = .2;
  c.angular = -.5;
  c.left = {true, -200};
  auto b = encode_control(c, 65535);
  auto z = encode_control({}, 0);
  b.insert(
    b.end(),
    z.begin(), z.end());
  for (size_t split = 1; split <= b.size(); ++split) {
    FrameParser p;
    p.feed(Bytes{0, 0xfc, 7, 0xfb});
    std::vector<Frame> frames;
    for (size_t off = 0; off < b.size(); off += split) {
      auto q = p.feed(b.data() + off, std::min(split, b.size() - off));
      frames.insert(
        frames.end(), q.begin(), q.end());
    }
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].seq, 65535);
    EXPECT_EQ(frames[1].seq, 0);
    EXPECT_FLOAT_EQ(decode_control(frames[0]).left.value, -200);
  }
}
TEST(Protocol, CorruptAndTruncatedLengthRecovery) {
  auto good = encode_control({}, 42);
  auto bad = good;
  bad[10] ^= 0x80;
  bad.insert(
    bad.end(), good.begin(), good.end());
  FrameParser p;
  auto f = p.feed(bad);
  ASSERT_EQ(f.size(), 1u);
  EXPECT_EQ(f[0].seq, 42);
  p.reset();
  Bytes false_header{0xfc, 0xfb, 1, 1, 0, 0, 255, 0};
  false_header.insert(
    false_header.end(), good.begin(), good.end());
  auto recovered = p.feed(false_header);
  for (int i = 0; i < 4; ++i) {
    auto more = p.feed(good);
    recovered.insert(recovered.end(), more.begin(), more.end());
  }
  ASSERT_FALSE(recovered.empty());
  EXPECT_EQ(recovered.back().seq, 42);
  p.reset();
  EXPECT_TRUE(p.feed(Bytes(10000, 0xfc)).empty());
  EXPECT_LE(p.buffered(), 268u);
}
TEST(Protocol, UnknownReorderedAndDuplicateBlocks) {
  auto f = unpack(encode_control({}, 0));
  auto first = Bytes(f.payload.begin(), f.payload.begin() + 3);
  f.payload.erase(
    f.payload.begin(), f.payload.begin() + 3);
  f.payload.insert(f.payload.end(), {0xa0, 3, 1, 2, 3});
  f.payload.insert(
    f.payload.end(), first.begin(), first.end());
  EXPECT_NO_THROW(decode_control(f));
  f.payload.insert(f.payload.end(), first.begin(), first.end());
  EXPECT_THROW(decode_control(f), ProtocolError);
}
TEST(Protocol, UnknownBlockMayContainAFrame) {
  auto outer = unpack(encode_control({}, 8));
  auto inner = encode_control({}, 99);
  outer.payload.push_back(0xa0);
  outer.payload.push_back(inner.size());
  outer.payload.insert(outer.payload.end(), inner.begin(), inner.end());
  auto frames = FrameParser{}.feed(encode_frame(outer));
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 8);
  EXPECT_NO_THROW(decode_control(frames[0]));
}
TEST(Protocol, MissingLengthsBooleansFlagsAndFinite) {
  auto f = unpack(encode_control({}, 0));
  f.payload.pop_back();
  EXPECT_THROW(decode_control(f), ProtocolError);
  f = unpack(encode_control({}, 0));
  f.payload[1] = 2;
  EXPECT_THROW(decode_control(f), ProtocolError);
  f = unpack(encode_control({}, 0));
  f.payload[2] = 4;
  EXPECT_THROW(decode_control(f), ProtocolError);
  f = unpack(encode_control({}, 0));
  f.payload[15] = 2;
  EXPECT_THROW(decode_control(f), ProtocolError);
  Control c;
  c.linear = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(encode_control(c, 0), ProtocolError);
  c = {};
  c.mower.value = 101;
  EXPECT_THROW(encode_control(c, 0), ProtocolError);
  c = {};
  c.lift = {true, 100};
  c.enable = true;
  EXPECT_THROW(validate(c), ProtocolError);
  c.stop = true;
  EXPECT_NO_THROW(validate(c));
}
TEST(Protocol, StatusFullRoundTrip) {
  State s;
  s.system = {0xfffffffe, 65535, 3, 2, 4, 15, 65535, 400};
  s.chassis = {.1f, -.2f, 12, -12, 11, -11};
  s.left = {true, -200, -190, 45};
  s.right = {false, 0, -10, 5};
  s.mower = {true, 20};
  s.lift = {false, 0, 0, false};
  auto b = encode_state(s, 65535);
  ASSERT_EQ(b.size(), 117u);
  auto d = decode_state(unpack(b));
  EXPECT_EQ(d.system.uptime_ms, s.system.uptime_ms);
  EXPECT_EQ(d.system.command_age_ms, 65535);
  EXPECT_FLOAT_EQ(d.right.actual, -10);
  EXPECT_FLOAT_EQ(d.mower.value, 20);
  EXPECT_FALSE(d.lift.valid);
}
TEST(Protocol, SprayerV1WireAndReportedCapability) {
  Control c;
  c.enable = true;
  c.sprayer = {true, 37.5f};
  EXPECT_NO_THROW(validate(c, 0x002f));
  EXPECT_THROW(validate(c, 0x000f), ProtocolError);
  auto wire = encode_control(c, 123);
  ASSERT_EQ(wire.size(), 60u);
  // 既有 v1 喷洒块保持开关 + 小端 float32 百分比，接入硬件不改变字节布局。
  const Bytes spray_block{0x50, 5, 1, 0, 0, 0x16, 0x42};
  EXPECT_EQ(Bytes(wire.begin() + 49, wire.begin() + 56), spray_block);
  const auto decoded = decode_control(unpack(wire));
  EXPECT_TRUE(decoded.sprayer.on);
  EXPECT_FLOAT_EQ(decoded.sprayer.value, 37.5f);

  State state;
  state.system.capabilities = 0x002f;
  state.sprayer = {true, 37.5f, 32.25f, true};
  auto feedback = decode_state(unpack(encode_state(state, 124)));
  EXPECT_EQ(feedback.system.capabilities, 0x002f);
  EXPECT_TRUE(feedback.sprayer.on);
  EXPECT_FLOAT_EQ(feedback.sprayer.target, 37.5f);
  EXPECT_FLOAT_EQ(feedback.sprayer.actual, 32.25f);
  EXPECT_TRUE(feedback.sprayer.valid);
  state.sprayer.valid = false;
  feedback = decode_state(unpack(encode_state(state, 125)));
  EXPECT_FALSE(feedback.sprayer.valid);
}
