#include "terramind_navigation_driver/port_discovery.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace terramind::navigation
{
bool NavigationProbe::observe(const Frame & frame)
{
  const uint32_t delta = frame.sequence - sequence_;
  if (count_ && delta == 0) {return false;}
  count_ = !count_ || delta >= 0x80000000u ? 1 : std::min(3u, count_ + 1);
  sequence_ = frame.sequence;
  return matched();
}

transport::DiscoveryResult discover_navigation_board(
  const std::vector<std::string> & patterns, double listen_seconds,
  const std::function<bool()> & keep_running)
{
  if (!std::isfinite(listen_seconds) || listen_seconds < .15 || listen_seconds > 10) {
    throw std::invalid_argument("discovery_listen_s must be in [0.15, 10]");
  }
  transport::DiscoveryResult result;
  if (!keep_running()) {return result;}
  transport::DiscoveryLock scan_lock;
  if (!scan_lock.acquired()) {
    result.errors.push_back("serial discovery busy; retrying");
    return result;
  }
  using Clock = std::chrono::steady_clock;
  struct Candidate
  {
    std::string path;
    std::unique_ptr<transport::SerialPort> port;
    Parser parser;
    NavigationProbe probe;
    Clock::time_point last_match{}, last_advance{};
    bool have_sequence = false;
    uint32_t sequence = 0;
    double frame_interval_s = 0.;
  };
  std::vector<Candidate> candidates;
  const auto paths = transport::serial_candidates(patterns);
  result.candidate_count = paths.size();
  for (const auto & path : paths) {
    if (!keep_running()) {return result;}
    try {
      auto port = std::make_unique<transport::SerialPort>();
      port->open(path, 460800);
      candidates.push_back({path, std::move(port), Parser{}, {}, {}});
    } catch (const std::exception & error) {
      result.errors.push_back(path + ": " + error.what());
    }
  }
  if (candidates.empty()) {return result;}
  const auto deadline = Clock::now() + std::chrono::duration<double>(listen_seconds);
  while (keep_running() && Clock::now() < deadline) {
    for (auto & candidate : candidates) {
      if (!candidate.port) {continue;}
      try {
        const auto bytes = candidate.port->read_some(0);
        const auto now = Clock::now();
        const double t = std::chrono::duration<double>(now.time_since_epoch()).count();
        for (const auto & frame : candidate.parser.feed(bytes.data(), bytes.size(), t)) {
          const uint32_t delta = frame.sequence - candidate.sequence;
          if (!candidate.have_sequence || delta != 0) {
            // 初始化/无定位时约 1 Hz；只用推进帧估计间隔，重复帧不能续期。
            candidate.frame_interval_s = candidate.have_sequence && delta < 0x80000000u ?
              std::chrono::duration<double>(now - candidate.last_advance).count() : 0.;
            candidate.sequence = frame.sequence;
            candidate.have_sequence = true;
            candidate.last_advance = now;
          }
          if (candidate.probe.observe(frame)) {candidate.last_match = now;}
        }
      } catch (const std::exception & error) {
        result.errors.push_back(candidate.path + ": " + error.what());
        candidate.port.reset();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!keep_running()) {return result;}
  Candidate * selected = nullptr;
  for (auto & candidate : candidates) {
    // 高频流继续使用至少 150 ms 的新鲜度窗口；1 Hz 心跳最多允许 1.5 s。
    const auto freshness = std::chrono::duration<double>(
      std::clamp(1.5 * candidate.frame_interval_s, .15, 1.5));
    if (candidate.port && candidate.probe.matched() &&
      Clock::now() - candidate.last_match < freshness)
    {
      result.matches.push_back(candidate.path);
      selected = &candidate;
    }
  }
  if (result.matches.size() == 1) {
    selected->port->discard_input();
    result.device = selected->path;
    result.port = std::move(selected->port);
  }
  return result;
}
}
