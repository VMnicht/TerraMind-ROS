#pragma once
#include "terramind_protocol/protocol_types.hpp"
#include <cstddef>
namespace terramind::protocol
{
// CRC-16/XMODEM；串口帧中只校验版本至载荷末尾，不包含帧头、CRC 和帧尾。
uint16_t crc16(const uint8_t * data, size_t size);
// 以下接口只处理字节和协议结构，不依赖 ROS、串口或系统时间。
Bytes encode_frame(const Frame & f);
Bytes encode_control(const Control & c, uint16_t seq);
Bytes encode_state(const State & s, uint16_t seq);
Control decode_control(const Frame & f);
// 输入应来自 FrameParser；解码仍检查必需 TLV、长度、枚举和有限浮点数。
State decode_state(const Frame & f);
}
