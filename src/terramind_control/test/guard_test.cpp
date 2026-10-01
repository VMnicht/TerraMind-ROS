#include "terramind_control/command_guard.hpp"
#include "terramind_control/ros_helpers.hpp"
#include <gtest/gtest.h>
namespace tc = terramind::control;
namespace tp = terramind::protocol;
TEST(Guard,TransportDelayConsumesValidityBudget) {
  tc::CommandGuard g;
  g.reset(1);
  const double before = tc::steady_seconds();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100;
  const rclcpp::Time now(100, 100000000, RCL_ROS_TIME);
  ASSERT_TRUE(tc::fresh_stamp(stamp, now, 0.150));
  const double source = tc::source_time_on_steady_clock(stamp, now);
  g.receive({}, 1, source, true, 15);
  tp::Control c;
  c.enable = true;
  c.linear = 0.1;
  ASSERT_TRUE(g.receive(c, 1, source, true, 15));
  EXPECT_TRUE(g.sample(before, true).enable);
  // Already 100 ms old on arrival: it cannot drive for another 150 ms.
  EXPECT_FALSE(g.sample(before + 0.060, true).enable);
}
TEST(Guard, RequiresDisabledHandshakeAndFreshConnection) {
  tc::CommandGuard g;
  g.reset(1);
  tp::Control c;
  c.enable = true;
  c.linear = .2;
  EXPECT_FALSE(g.receive(c, 1, 0, true, 15));
  EXPECT_TRUE(g.receive({}, 1, 0, true, 15));
  EXPECT_TRUE(g.receive(c, 1, .02, true, 15));
  EXPECT_TRUE(g.sample(.03, true).enable);
  EXPECT_FALSE(g.receive(c, 2, .04, true, 15));
  g.reset(2);
  EXPECT_FALSE(g.receive(c, 1, .05, true, 15));
  EXPECT_FALSE(g.sample(.06, true).enable);
}
TEST(Guard, TimeoutLatchesUntilDisabled) {
  tc::CommandGuard g;
  g.reset(1);
  g.receive({}, 1, 0, true, 15);
  tp::Control c;
  c.enable = true;
  c.linear = .2;
  g.receive(c, 1, .01, true, 15);
  EXPECT_TRUE(g.sample(.10, true).enable);
  EXPECT_FALSE(g.sample(.17, true).enable);
  EXPECT_FALSE(g.receive(c, 1, .18, true, 15));
  g.receive({}, 1, .19, true, 15);
  EXPECT_TRUE(g.receive(c, 1, .20, true, 15));
  EXPECT_TRUE(g.sample(.21, true).enable);
  EXPECT_FALSE(g.sample(.22, false).enable);
  EXPECT_FALSE(g.receive(c, 1, .23, true, 15));
}
TEST(Guard, InvalidValuesAndStampsStop) {
  tc::CommandGuard g;
  g.reset(1);
  g.receive({}, 1, 0, true, 15);
  tp::Control c;
  c.enable = true;
  g.receive(c, 1, .01, true, 15);
  c.linear = 1;
  EXPECT_FALSE(g.receive(c, 1, .02, true, 15));
  EXPECT_FALSE(g.sample(.03, true).enable);
  EXPECT_FALSE(g.receive({}, 1, .04, false, 15));
  EXPECT_THROW(tc::CommandGuard(.25), std::invalid_argument);
}
