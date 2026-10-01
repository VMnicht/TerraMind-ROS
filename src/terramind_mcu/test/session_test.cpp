#include "terramind_mcu/mcu_session.hpp"
#include <gtest/gtest.h>
using terramind::mcu::Session;
namespace tp = terramind::protocol;
TEST(Session, HandshakeDuplicateAndAckAge) {
  Session s;
  s.reset(123, 0);
  tp::State state;
  state.system.mode = 2;
  state.system.command_age_ms = 0;
  s.sent(0, {}, 0);
  EXPECT_TRUE(s.observe(state, 0, .05));
  EXPECT_TRUE(s.ready);
  EXPECT_FALSE(s.observe(state, 0, .10));
  EXPECT_TRUE(s.expired(.21));
  state.system.uptime_ms = 100;
  EXPECT_TRUE(s.observe(state, 1, .22));
  EXPECT_FALSE(s.ready);
}
TEST(Session, RejectRebootAndWrap) {
  Session s;
  s.reset(1, 0);
  tp::State st;
  st.system.mode = 2;
  st.system.command_age_ms = 0;
  st.system.uptime_ms = 1000;
  s.sent(65535, {}, 0);
  st.system.last_command_seq = 65535;
  s.observe(st, 65535, .01);
  ASSERT_TRUE(s.ready);
  s.sent(0, {}, .02);
  st.system.last_command_seq = 0;
  st.system.uptime_ms = 1010;
  s.observe(st, 0, .03);
  EXPECT_TRUE(s.ready);
  st.system.result = 3;
  st.system.uptime_ms = 1020;
  s.observe(st, 1, .04);
  EXPECT_FALSE(s.ready);
  st.system.uptime_ms = 0;
  s.observe(st, 0, .05);
  EXPECT_TRUE(s.rebooted);
}
