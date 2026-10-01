#pragma once
#include "terramind_protocol/protocol_types.hpp"
#include <cstddef>
namespace terramind::protocol
{
uint16_t crc16(const uint8_t * data, size_t size);
Bytes encode_frame(const Frame & f);
Bytes encode_control(const Control & c, uint16_t seq);
Bytes encode_state(const State & s, uint16_t seq);
Control decode_control(const Frame & f);
State decode_state(const Frame & f);
}
