#include "terramind_sim/board_model.hpp"
#include "terramind_protocol/frame_parser.hpp"
#include <gtest/gtest.h>
#include <cmath>
namespace tp = terramind::protocol;
namespace ts = terramind::sim;
TEST(Model, TakeoverTimeoutAndNoFallback) {
  ts::BoardModel m;
  EXPECT_EQ(m.state().system.mode, 0);
  EXPECT_EQ(m.state().system.command_age_ms, 65535);
  m.accept({}, 0, 0);
  EXPECT_EQ(m.state().system.mode, 2);
  tp::Control c;
  c.enable = true;
  c.linear = .2;
  m.accept(c, 1, .01);
  m.step(.20);
  EXPECT_EQ(m.state().system.mode, 1);
  EXPECT_GT(m.drive().x, 0);
  m.step(.261);
  EXPECT_EQ(m.state().system.mode, 2);
  EXPECT_EQ(m.state().chassis.left_target, 0);
  EXPECT_TRUE(m.state().system.faults & 1);
  EXPECT_EQ(m.state().system.result, 0);
  m.accept({}, 2, .3);
  EXPECT_EQ(m.state().system.mode, 2);
  EXPECT_FALSE(m.state().system.faults & 1);
}
TEST(Model, RejectedCommandsDoNotRenewWatchdog) {
  ts::BoardModel m;
  tp::Control c;
  c.enable = true;
  c.linear = .2;
  m.accept(c, 10, 0);
  c.lift.on = true;
  m.accept(c, 11, .20);
  EXPECT_EQ(m.state().system.result, 3);
  EXPECT_EQ(m.state().system.last_command_seq, 11);
  EXPECT_FLOAT_EQ(m.state().chassis.linear, .2);
  m.step(.251);
  EXPECT_TRUE(m.state().system.faults & 1);
  EXPECT_EQ(m.state().system.result, 3);
}
TEST(Model, SemanticFailureAndStop) {
  ts::BoardModel m;
  tp::Frame f{tp::CONTROL, 33, {1, 1, 0}};
  m.receive(f, 0);
  EXPECT_EQ(m.state().system.result, 1);
  EXPECT_EQ(m.state().system.last_command_seq, 33);
  EXPECT_EQ(m.state().system.mode, 0);
  tp::Control c;
  c.enable = true;
  c.mower = {true, 50};
  c.left = {true, -200};
  m.accept(c, 34, .01);
  EXPECT_FLOAT_EQ(m.state().mower.value, 50);
  m.accept(tp::stopped(), 35, .02);
  EXPECT_FALSE(m.state().mower.on);
  EXPECT_EQ(m.state().left.target, 0);
}
TEST(Model, GeometryAndArc) {
  EXPECT_THROW(ts::DifferentialDriveModel(ts::Geometry{0, .4}), std::invalid_argument);
  ts::DifferentialDriveModel d({.2, .4}, 1e6, 1e6);
  EXPECT_NEAR(d.left_rpm(0, 1), -19.098593, 1e-5);
  EXPECT_NEAR(d.right_rpm(0, 1), 19.098593, 1e-5);
  d.step(.2, 1, 1);
  EXPECT_NEAR(d.x, .2 * std::sin(1), .003);
  EXPECT_NEAR(d.y, .2 * (1 - std::cos(1)), .003);
}
TEST(Model, SprayerOnOffTimeoutAndLegacyBoard) {
  ts::BoardModel current;
  EXPECT_EQ(current.state().system.capabilities, 0x002f);
  tp::Control c;
  c.enable = true;
  c.sprayer = {true, 40};
  current.accept(c, 1, 0);
  EXPECT_EQ(current.state().system.result, 0);
  EXPECT_TRUE(current.state().sprayer.on);
  EXPECT_FLOAT_EQ(current.state().sprayer.target, 40);
  current.step(.1);
  EXPECT_TRUE(current.state().sprayer.valid);
  EXPECT_GT(current.state().sprayer.actual, 0);
  EXPECT_FALSE(current.state().lift.valid);
  c.sprayer.on = false;
  current.accept(c, 2, .11);
  EXPECT_FALSE(current.state().sprayer.on);
  EXPECT_FLOAT_EQ(current.state().sprayer.target, 0);
  c.sprayer.on = true;
  current.accept(c, 3, .12);
  current.step(.371);
  EXPECT_TRUE(current.state().system.faults & 1);
  EXPECT_FALSE(current.state().sprayer.on);
  EXPECT_FLOAT_EQ(current.state().sprayer.target, 0);

  ts::BoardModel legacy({}, 0x000f);
  legacy.accept(c, 1, 0);
  EXPECT_EQ(legacy.state().system.result, uint8_t(tp::Result::UNSUPPORTED));
  EXPECT_FALSE(legacy.state().sprayer.on);
  EXPECT_FALSE(legacy.state().sprayer.valid);
}
