#pragma once
#include "terramind_protocol/protocol_types.hpp"
#include "terramind_transport/serial_port.hpp"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace terramind::mcu
{
// glob patterns are visited in priority order; aliases of one device count once.
std::vector<std::string> serial_candidates(const std::vector<std::string> & patterns);

class StatusProbe
{
public:
  // Feed only CRC-checked frames from FrameParser. Require progressing state
  // sequence AND uptime, including normal integer wrap, rather than one packet.
  // True means a newly progressing sample; duplicates never renew freshness.
  bool observe(const protocol::Frame & frame);
  bool matched() const {return matched_;}

private:
  bool have_state_ = false, matched_ = false;
  uint16_t sequence_ = 0;
  uint32_t uptime_ = 0;
};

struct DiscoveryResult
{
  // The selected descriptor stays open/locked from identification to handshake.
  std::unique_ptr<transport::SerialPort> port;
  std::string device;
  std::vector<std::string> matches;
  std::vector<std::string> errors;
  size_t candidate_count = 0;
};

// Passive discovery: this function never writes bytes to any candidate port.
// No selection when zero or multiple ports speak the board protocol.
DiscoveryResult discover_control_board(
  const std::vector<std::string> & patterns, double listen_seconds,
  const std::function<bool()> & keep_running);
}
