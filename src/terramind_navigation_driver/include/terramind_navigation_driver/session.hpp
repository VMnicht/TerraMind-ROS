#pragma once
#include "terramind_navigation_driver/protocol.hpp"
#include <optional>

namespace terramind::navigation
{
struct Limits
{
  double frame_timeout_s = 3, navigation_timeout_s = 0.1;
  uint16_t max_age_ms = 500;
  bool allow_simulated = false;
};
struct Observation {bool publish = false, restarted = false; std::string reason;};
class Session
{
public:
  explicit Session(Limits limits = {});
  void connect(double now);
  void disconnect();
  Observation observe(const Frame & frame, double now);
  bool online(double now) const;
  bool expired(double now) const;
  bool usable(double now) const;
  std::string reason(double now) const;
  uint64_t id = 0, missing = 0, duplicates = 0, restarts = 0, rejected = 0;
private:
  void new_epoch();
  Limits limits_;
  bool connected_ = false, eligible_ = false;
  double opened_ = 0, last_good_ = 0;
  uint16_t last_age_ = 65535;
  std::optional<double> last_frame_, measurement_time_;
  std::optional<uint32_t> sequence_;
  std::string reason_ = "disconnected";
};
}
