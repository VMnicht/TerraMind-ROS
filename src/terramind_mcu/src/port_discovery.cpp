#include "terramind_mcu/port_discovery.hpp"
#include "terramind_protocol/codec.hpp"
#include "terramind_protocol/frame_parser.hpp"
#include <chrono>
#include <cmath>
#include <glob.h>
#include <set>
#include <stdexcept>
#include <sys/stat.h>
#include <thread>

namespace terramind::mcu
{
std::vector<std::string> serial_candidates(const std::vector<std::string> & patterns)
{
  std::vector<std::string> paths;
  std::set<dev_t> devices;
  for (const auto & pattern : patterns) {
    glob_t expanded{};
    const int result = glob(pattern.c_str(), 0, nullptr, &expanded);
    if (result != 0 && result != GLOB_NOMATCH) {
      globfree(&expanded);
      throw std::runtime_error("cannot enumerate serial pattern: " + pattern);
    }
    for (size_t i = 0; i < expanded.gl_pathc; ++i) {
      struct stat info{};
      const std::string path = expanded.gl_pathv[i];
      if (stat(path.c_str(), &info) == 0 && S_ISCHR(info.st_mode) &&
        devices.insert(info.st_rdev).second)
      {
        paths.push_back(path);
      }
    }
    globfree(&expanded);
  }
  return paths;
}

bool StatusProbe::observe(const protocol::Frame & frame)
{
  if (frame.type != protocol::STATUS) {
    return false;
  }
  protocol::State state;
  try {
    state = protocol::decode_state(frame);
  } catch (const protocol::ProtocolError &) {
    return false;
  }
  const auto uptime = state.system.uptime_ms;
  if (have_state_) {
    const uint16_t sequence_delta = frame.seq - sequence_;
    const uint32_t uptime_delta = uptime - uptime_;
    if (sequence_delta == 0) {
      return false;
    }
    if (sequence_delta < 0x8000 && uptime_delta > 0 && uptime_delta < 0x80000000u) {
      matched_ = true;
    } else {
      matched_ = false;
    }
  }
  have_state_ = true;
  sequence_ = frame.seq;
  uptime_ = uptime;
  return matched_;
}

DiscoveryResult discover_control_board(
  const std::vector<std::string> & patterns, double listen_seconds,
  const std::function<bool()> & keep_running)
{
  if (!std::isfinite(listen_seconds) || listen_seconds < 0.15 || listen_seconds > 10) {
    throw std::invalid_argument("discovery_listen_s must be in [0.15, 10]");
  }
  DiscoveryResult result;
  struct Candidate
  {
    std::string path;
    std::unique_ptr<transport::SerialPort> port;
    protocol::FrameParser parser;
    StatusProbe probe;
    std::chrono::steady_clock::time_point last_match;
  };
  std::vector<Candidate> candidates;
  const auto paths = serial_candidates(patterns);
  result.candidate_count = paths.size();
  for (const auto & path : paths) {
    if (!keep_running()) {
      return result;
    }
    try {
      auto port = std::make_unique<transport::SerialPort>();
      port->open(path);
      candidates.push_back({path, std::move(port), {}, {}, {}});
    } catch (const std::exception & error) {
      result.errors.push_back(path + ": " + error.what());
    }
  }
  if (candidates.empty()) {
    return result;
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(listen_seconds);
  while (keep_running() && std::chrono::steady_clock::now() < deadline) {
    for (auto & candidate : candidates) {
      if (!candidate.port) {
        continue;
      }
      try {
        for (const auto & frame : candidate.parser.feed(candidate.port->read_some(0))) {
          if (candidate.probe.observe(frame)) {
            candidate.last_match = std::chrono::steady_clock::now();
          }
        }
      } catch (const std::exception & error) {
        result.errors.push_back(candidate.path + ": " + error.what());
        candidate.port.reset();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!keep_running()) {
    return result;
  }
  // A device that disappeared or stopped producing state during the scan is
  // not selectable. Keep its descriptor; do not reopen a possibly renamed port.
  Candidate * selected = nullptr;
  for (auto & candidate : candidates) {
    if (candidate.port && candidate.probe.matched() &&
      std::chrono::steady_clock::now() - candidate.last_match < std::chrono::milliseconds(150))
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
