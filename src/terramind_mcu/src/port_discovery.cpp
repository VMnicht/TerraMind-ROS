#include "terramind_mcu/port_discovery.hpp"
#include "terramind_protocol/codec.hpp"
#include "terramind_protocol/frame_parser.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace terramind::mcu
{
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
  if (!keep_running()) {return result;}
  transport::DiscoveryLock scan_lock;
  if (!scan_lock.acquired()) {
    result.errors.push_back("serial discovery busy; retrying");
    return result;
  }
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
  // 所有已打开候选共享监听窗口，不为每个无关串口串行等待完整时长。
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
  // 扫描结束时仍须有 150 ms 内的新状态；曾匹配但已静默的设备不可选择。
  // 保留原句柄，避免重开路径时 USB 设备编号已经被系统重新分配。
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
