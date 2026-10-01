#include "terramind_protocol/codec.hpp"
namespace terramind::protocol
{
uint16_t crc16(const uint8_t * data, size_t size)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}
}
