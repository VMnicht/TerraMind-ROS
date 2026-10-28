#include <gtest/gtest.h>
#include "terramind_mcu/angular_mapping.hpp"
#include "terramind_protocol/codec.hpp"
#include "terramind_protocol/frame_parser.hpp"
namespace tp = terramind::protocol;
using terramind::mcu::AngularMapping;

TEST(AngularMapping, WireCommandAndFeedbackUseSameConvention)
{
  for (int sign : {-1, 1}) {
    AngularMapping map(sign);
    for (float omega : {-0.6f, 0.0f, 0.6f}) {
      tp::Control c;
      c.enable = true; c.linear = 0.2f; c.angular = omega;
      auto bytes = tp::encode_control(map.to_board(c), 42);
      tp::FrameParser parser;
      auto frames = parser.feed(bytes.data(), bytes.size());
      ASSERT_EQ(frames.size(), 1u);
      auto wire = tp::decode_control(frames[0]);
      EXPECT_FLOAT_EQ(wire.angular, omega * sign);
      EXPECT_FLOAT_EQ(wire.linear, c.linear);
      EXPECT_TRUE(wire.enable);
      EXPECT_FALSE(wire.mower.on);
      tp::State state;
      state.chassis.angular = wire.angular;
      state.chassis.left_actual = 17.0f;
      state.chassis.right_actual = -18.0f;
      auto feedback = map.to_ros(state);
      EXPECT_FLOAT_EQ(feedback.chassis.angular, omega);
      EXPECT_FLOAT_EQ(feedback.chassis.left_actual, 17.0f);
      EXPECT_FLOAT_EQ(feedback.chassis.right_actual, -18.0f);
    }
    auto stopped = map.to_board(tp::stopped());
    EXPECT_TRUE(stopped.stop);
    EXPECT_FALSE(stopped.enable);
    EXPECT_FLOAT_EQ(stopped.angular, 0.0f);
  }
}
TEST(AngularMapping, RejectsInvalidSigns)
{
  EXPECT_THROW(AngularMapping{0}, std::invalid_argument);
  EXPECT_THROW(AngularMapping{2}, std::invalid_argument);
  EXPECT_THROW(AngularMapping{-2}, std::invalid_argument);
}
