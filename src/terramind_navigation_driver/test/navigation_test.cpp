#include "terramind_navigation_driver/coordinates.hpp"
#include "terramind_navigation_driver/session.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
namespace n = terramind::navigation;
namespace
{
constexpr double pi = 3.14159265358979323846;
std::vector<uint8_t> golden()
{
  // 协议中的独立固定向量，不能由被测编码器生成。
  std::istringstream in(
    "AA 55 01 9D 01 00 00 00 00 00 00 00 00 40 8F 40 "
    "00 00 00 00 00 00 E0 3F 00 00 00 00 00 00 00 40 "
    "00 00 A0 41 00 00 00 00 00 00 80 3F 00 00 00 00 "
    "00 00 00 00 00 00 00 00 DB 0F C9 3F F3 04 35 3F "
    "00 00 00 00 00 00 00 00 F3 04 35 3F C8 00 3D 23");
  unsigned int v;
  std::vector<uint8_t> bytes;
  while (in >> std::hex >> v) {bytes.push_back(v);}
  return bytes;
}
n::Frame good()
{
  auto b = golden(); auto f = n::decode(b.data(), b.size()); f.status &= ~n::SIMULATED; return f;
}
void repair_crc(std::vector<uint8_t> & b)
{
  auto crc = n::crc16(b.data() + 2, 76); b[78] = crc & 255; b[79] = crc >> 8;
}
}
TEST(Protocol, GoldenAndStandardCRC)
{
  const std::string check = "123456789";
  EXPECT_EQ(n::crc16(reinterpret_cast<const uint8_t *>(check.data()), check.size()), 0x29b1);
  const auto b = golden(); ASSERT_EQ(b.size(), 80u);
  EXPECT_EQ(n::crc16(b.data() + 2, 76), 0x233d);
  auto f = n::decode(b.data(), b.size());
  EXPECT_EQ(f.status, 0x9d); EXPECT_EQ(f.rtk_state(), 4); EXPECT_EQ(f.sequence, 1u);
  EXPECT_DOUBLE_EQ(f.state_time_s, 1000); EXPECT_DOUBLE_EQ(f.latitude_rad, .5);
  EXPECT_DOUBLE_EQ(f.longitude_rad, 2); EXPECT_FLOAT_EQ(f.height_m, 20);
  EXPECT_FLOAT_EQ(f.velocity_ned[1], 1); EXPECT_NEAR(f.rpy[2], pi / 2, 1e-7);
  EXPECT_NEAR(f.quaternion_wxyz[0], std::sqrt(.5), 1e-7);
  EXPECT_NEAR(f.quaternion_wxyz[3], std::sqrt(.5), 1e-7); EXPECT_EQ(f.age_ms, 200);
  EXPECT_TRUE(n::invalid_reason(f).empty());
  EXPECT_THROW(n::decode(b.data(), 79), std::invalid_argument);
}
TEST(Protocol, AllSplitPointsAndMultipleFrames)
{
  auto b = golden();
  for (size_t cut = 0; cut <= b.size(); ++cut) {
    n::Parser p;
    auto first = p.feed(b.data(), cut, 0);
    auto second = p.feed(b.data() + cut, b.size() - cut, .01);
    EXPECT_EQ(first.size() + second.size(), 1u) << cut;
    EXPECT_EQ(p.buffered(), 0u);
  }
  std::vector<uint8_t> stream{0, 0xaa, 0xaa, 0x10};
  for (int i = 0; i < 20; ++i) {stream.insert(stream.end(), b.begin(), b.end());}
  n::Parser p; EXPECT_EQ(p.feed(stream.data(), stream.size(), 0).size(), 20u);
}
TEST(Protocol, CorruptionResyncAndEmbeddedHeader)
{
  auto b = golden(); b[40] = 0xaa; b[41] = 0x55; repair_crc(b);
  n::Parser p; ASSERT_EQ(p.feed(b.data(), b.size(), 0).size(), 1u);
  auto corrupt = golden(); corrupt[10] ^= 1;
  auto unknown = golden(); unknown[2] = 2; repair_crc(unknown);
  std::vector<uint8_t> stream = corrupt;
  stream.insert(stream.end(), unknown.begin(), unknown.end());
  // 损坏半帧后紧跟完整帧；必须逐字节寻找后续帧头。
  stream.insert(stream.end(), corrupt.begin(), corrupt.begin() + 33);
  stream.insert(stream.end(), b.begin(), b.end());
  EXPECT_EQ(p.feed(stream.data(), stream.size(), .01).size(), 1u);
  EXPECT_GT(p.stats().crc_errors, 0u); EXPECT_GT(p.stats().version_errors, 0u);
}
TEST(Protocol, TimeoutWithoutInputAndBoundedNoise)
{
  auto b = golden(); n::Parser p;
  EXPECT_TRUE(p.feed(b.data(), 30, 0).empty());
  EXPECT_EQ(p.buffered(), 30u); EXPECT_TRUE(p.tick(.11).empty());
  EXPECT_EQ(p.buffered(), 0u); EXPECT_EQ(p.stats().timeouts, 1u);
  EXPECT_EQ(p.feed(b.data(), 80, .12).size(), 1u);
  std::vector<uint8_t> noise(100000, 0xaa);
  EXPECT_TRUE(p.feed(noise.data(), noise.size(), .13).empty()); EXPECT_LE(p.buffered(), 79u);
  p.reset(); EXPECT_EQ(p.buffered(), 0u);
}
TEST(Protocol, StatusAndNumerics)
{
  auto f = good();
  for (int rtk = 0; rtk < 8; ++rtk) {
    f.status = (rtk << 5) | 0x0d;
    EXPECT_EQ(f.rtk_state(), rtk); EXPECT_TRUE(n::invalid_reason(f).empty());
  }
  f.status |= n::NAV_FAULT; EXPECT_EQ(n::invalid_reason(f), "conflicting_flags");
  f = good(); f.quaternion_wxyz = {}; EXPECT_EQ(n::invalid_reason(f), "quaternion_norm");
  f = good(); f.latitude_rad = pi; EXPECT_EQ(n::invalid_reason(f), "coordinate_range");
  f = good(); f.longitude_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(n::invalid_reason(f).empty());
  f = good(); f.velocity_ned[0] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(n::invalid_reason(f).empty());
  f = {}; EXPECT_EQ(n::invalid_reason(f), "navigation_invalid");
}
TEST(Session, DuplicatesCannotRenewStreamOrConnectionBaseline)
{
  n::Session s; s.connect(0); auto f = good(); ASSERT_TRUE(s.observe(f, 0).publish);
  EXPECT_TRUE(s.usable(.05));
  ++f.sequence; EXPECT_FALSE(s.observe(f, .08).publish);
  EXPECT_FALSE(s.usable(.11)); EXPECT_TRUE(s.online(.11)); EXPECT_EQ(s.duplicates, 1u);
  EXPECT_FALSE(s.expired(3)); EXPECT_TRUE(s.expired(3.09));
  f.state_time_s += .01; ++f.sequence; EXPECT_TRUE(s.observe(f, 3.1).publish);
  EXPECT_TRUE(s.usable(3.1));
}
TEST(Session, StatusFramesFaultAndAge)
{
  n::Session s; s.connect(0); auto f = good(); s.observe(f, 0);
  f.status = n::TIME_LOCKED; ++f.sequence;
  EXPECT_FALSE(s.observe(f, .01).publish); EXPECT_FALSE(s.usable(.01));
  EXPECT_TRUE(s.online(.01));
  f = good(); f.sequence = 3; f.age_ms = 501;
  EXPECT_FALSE(s.observe(f, .02).publish);
  f.age_ms = 65535; ++f.sequence; EXPECT_FALSE(s.observe(f, .03).publish);
  f.age_ms = 500; ++f.sequence; f.state_time_s += .01;
  EXPECT_TRUE(s.observe(f, .04).publish);
  EXPECT_TRUE(s.usable(.04)); EXPECT_FALSE(s.usable(.041));
  f.status |= n::SIMULATED; ++f.sequence; f.state_time_s += 1;
  EXPECT_EQ(s.observe(f, .05).reason, "simulated_data_disabled");
}
TEST(Session, WrapGapRebootAndReconnect)
{
  n::Session s; s.connect(0); auto f = good(); f.sequence = 0xfffffffeu;
  ASSERT_TRUE(s.observe(f, 0).publish); auto id = s.id;
  f.sequence = 1; f.state_time_s += .01;
  auto r = s.observe(f, .01); EXPECT_TRUE(r.publish); EXPECT_FALSE(r.restarted);
  EXPECT_EQ(s.missing, 2u); EXPECT_EQ(s.id, id);
  f.sequence = 0; f.state_time_s = 0;
  r = s.observe(f, .02); EXPECT_TRUE(r.restarted); EXPECT_TRUE(r.publish); EXPECT_GT(s.id, id);
  f.sequence = 2; f.state_time_s = -1;
  EXPECT_TRUE(s.observe(f, .03).restarted);
  id = s.id; s.disconnect(); EXPECT_FALSE(s.online(.04)); EXPECT_FALSE(s.usable(.04));
  s.connect(.05); EXPECT_GT(s.id, id); EXPECT_FALSE(s.online(.05));
  EXPECT_TRUE(s.expired(3.06));
}
TEST(Session, InvalidStateZeroTimeDoesNotReboot)
{
  n::Session s; s.connect(0); auto f = good(); s.observe(f, 0); auto id = s.id;
  f.status = 0; f.state_time_s = 0; ++f.sequence;
  EXPECT_FALSE(s.observe(f, .1).restarted); EXPECT_EQ(s.id, id);
}
TEST(Coordinates, IndependentProjReference)
{
  // PROJ 8 cct: cart WGS84; topocentric lat_0=30 lon_0=114 h_0=50。
  n::Geodetic p{30.000234 * pi / 180, 114.000123 * pi / 180, 52.3};
  auto e = n::to_ecef(p);
  EXPECT_NEAR(e.x(), -2248568.536417964846, 1e-8);
  EXPECT_NEAR(e.y(), 5050338.443143481389, 1e-8);
  EXPECT_NEAR(e.z(), 3170422.349783850368, 1e-8);
  n::Coordinates c; c.set_origin({30 * pi / 180, 114 * pi / 180, 50});
  auto local = c.forward(p);
  EXPECT_NEAR(local.x(), 11.867881860335, 1e-8);
  EXPECT_NEAR(local.y(), 25.939691972497, 1e-8);
  EXPECT_NEAR(local.z(), 2.299935998117, 1e-8);
  auto back = c.reverse(local);
  EXPECT_NEAR(back.latitude_rad, p.latitude_rad, 1e-14);
  EXPECT_NEAR(back.longitude_rad, p.longitude_rad, 1e-14);
  EXPECT_NEAR(back.height_m, p.height_m, 1e-8);
}
TEST(Coordinates, OriginAxesPolesAndDateline)
{
  n::Coordinates c; c.set_origin({0, 0, 0}); EXPECT_LT(c.forward({0, 0, 0}).norm(), 1e-9);
  EXPECT_GT(c.forward({0, 1e-6, 0}).x(), 6);
  EXPECT_GT(c.forward({1e-6, 0, 0}).y(), 6);
  EXPECT_NEAR(c.forward({0, 0, 10}).z(), 10, 1e-9);
  for (double lat : {-pi / 2, -1.2, 0.0, 1.2, pi / 2}) {
    for (double lon : {-pi, -1.0, 0.0, pi}) {
      n::Geodetic p{lat, lon, -20};
      auto back = n::from_ecef(n::to_ecef(p));
      EXPECT_NEAR(back.latitude_rad, lat, 1e-14); EXPECT_NEAR(back.height_m, -20, 1e-8);
    }
  }
  c.set_origin({.7, pi - 1e-6, 10});
  auto across = c.forward({.7, -pi + 1e-6, 10}); EXPECT_GT(across.x(), 0); EXPECT_LT(across.norm(), 10);
  EXPECT_THROW(n::to_ecef({pi, 0, 0}), std::invalid_argument);
}
TEST(Coordinates, AttitudeVelocityAndQuaternionSign)
{
  auto f = good(); n::Coordinates c; c.set_origin({f.latitude_rad, f.longitude_rad, f.height_m});
  auto s = c.convert(f);
  EXPECT_LT(s.imu_position.norm(), 1e-9);
  EXPECT_LT((s.imu_velocity - Eigen::Vector3d(1, 0, 0)).norm(), 1e-12);
  // 基准帧朝东，在原点 ENU/FLU 下应为单位旋转。
  EXPECT_NEAR(s.imu_orientation.angularDistance(Eigen::Quaterniond::Identity()), 0, 1e-12);
  f.quaternion_wxyz = {1, 0, 0, 0}; s = c.convert(f);
  EXPECT_LT((s.imu_orientation * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitY()).norm(), 1e-12);
  auto previous = s.imu_orientation; f.quaternion_wxyz = {-1, 0, 0, 0};
  EXPECT_NEAR(c.convert(f).imu_orientation.angularDistance(previous), 0, 1e-12);
  // 大俯仰时完全忽略欧拉角的奇异区表示，输出仍为有效正交旋转。
  f.quaternion_wxyz = {float(std::sqrt(.5)), 0, float(std::sqrt(.5)), 0};
  auto rotation = c.convert(f).imu_orientation.toRotationMatrix();
  EXPECT_NEAR(rotation.determinant(), 1, 1e-12);
}
TEST(Coordinates, MountingAndLeverArm)
{
  auto f = good(); n::Coordinates c; c.set_origin({f.latitude_rad, f.longitude_rad, f.height_m});
  c.set_extrinsics({1, 0, 0}, Eigen::Quaterniond::Identity());
  EXPECT_LT((c.convert(f).base_position - Eigen::Vector3d(-1, 0, 0)).norm(), 1e-12);
  // IMU 相对车辆绕 z 安装 +90°；IMU 朝东意味着车辆朝南。
  c.set_extrinsics({1, 0, 0}, Eigen::Quaterniond(Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitZ())));
  auto s = c.convert(f);
  EXPECT_LT((s.base_orientation * Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitY()).norm(), 1e-12);
  EXPECT_LT((s.base_position - Eigen::Vector3d(0, 1, 0)).norm(), 1e-12);
  EXPECT_THROW(c.set_extrinsics({0, 0, 0}, Eigen::Quaterniond(0, 0, 0, 0)), std::invalid_argument);
}
TEST(Coordinates, FixedOriginVelocityUsesCurrentTangentPlane)
{
  n::Coordinates c; c.set_origin({0, 0, 0});
  auto f = good(); f.latitude_rad = 0; f.longitude_rad = pi / 2; f.height_m = 0;
  f.velocity_ned = {0, 1, 0};
  // 东经90°处的东向速度，在原点(0,0)的 ENU 中朝 -Up。
  auto s = c.convert(f); EXPECT_LT((s.imu_velocity - Eigen::Vector3d(0, 0, -1)).norm(), 1e-12);
}
