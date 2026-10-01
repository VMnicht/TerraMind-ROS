# TerraMind USART3 PC 控制与状态协议 v1

本文对应固件 `TerraMind_FW/Driver/pc_protocol.*` 和 PC 参考实现 `tools/pc_protocol.py`。

2026-10-01 同步：喷洒模块已接入，用户确认继续使用既有 v1 接口。喷洒块布局、单位及帧长度不变，当前能力配置由 `0x000F` 增加 bit5 为 `0x002F`；实机仍以状态帧实际回传值为准。

## 1. 接口与运行方式

- 开发板控制接口：USART3，PD8=TX、PD9=RX，115200 bit/s、8 数据位、无校验、1 停止位、无流控。
- 调试文本：USART6，PC6=TX、PC7=RX，同为 115200 8N1。USART3 只发送和接收本文的二进制帧。
- PC 连接板卡引脚时使用 **3.3 V TTL USB 串口**，板卡 TX 接适配器 RX，板卡 RX 接适配器 TX，双方共地。先核对板卡接口是否已有板载串口转换器；不能直接接 RS-232 电平。
- PC 每 20–50 ms 发送一帧完整控制快照。开发板每 50 ms 发送一帧完整状态快照，不另发送 ACK 帧。每帧独立，不能只发送发生变化的字段。
- 帧在字节流中可连续发送，没有换行符。PC 接收器必须按帧头、长度、帧尾和 CRC 分帧，不能把一次 `read()` 等同于一帧。

当前 USB 串口上位机只需实现两种帧：`0x01` 控制下发、`0x81` 状态上报。

## 2. 通用帧

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | 帧头 | `FC FB` |
| 2 | 1 | 版本 | 固定 `01` |
| 3 | 1 | 类型 | `01` 控制；`81` 状态 |
| 4 | 2 | 帧序号 | `uint16`，每方向各自递增，回绕到 0 |
| 6 | 2 | 负载长度 N | `uint16`，0–256；v1 控制为 48，状态为 105 |
| 8 | N | 负载 | 功能块序列 |
| 8+N | 2 | CRC16 | 对偏移 **2 至 7+N** 计算，低字节先发 |
| 10+N | 2 | 帧尾 | `FD FE` |

总长度为 `N+12`：v1 控制帧 60 字节，状态帧 117 字节。所有整数和 IEEE-754 `float32` 使用**小端序**。CRC 为 **CRC-16/XMODEM**：多项式 `0x1021`，初值 `0x0000`，不反射，结果异或 `0x0000`。测试向量 `"123456789" → 0x31C3`。

### 功能块 TLV

负载由若干 `ID:u8 | LEN:u8 | DATA:LEN字节` 组成。v1 下列七个块各出现**恰好一次**；次序不限。缺少已知必需块、重复已知块、已知块长度错误或无效字段使整帧失效，固件不会应用其中任何部分。未知 ID 按 LEN 跳过，可用于新增外设。未知块不能替代 v1 必需块。新增兼容字段使用新 ID；改变既有字段含义或布局须升级协议版本。

## 3. PC → 开发板：完整控制帧 `TYPE=0x01`

| ID | LEN | DATA 字节布局 | 单位与取值 |
|---|---:|---|---|
| `01` 系统 | 1 | `flags:u8` | bit0 控制使能；bit1 停止请求；其他位必须为 0。未使能或请求停止时执行器目标全部归零。 |
| `10` 底盘 | 8 | `linear:f32, angular:f32` | 线速度 m/s：-0.35…0.35；角速度 rad/s：-2…2。 |
| `20` 左播撒 | 5 | `on:u8, rpm:f32` | `on` 为 0/1；目标输出轴速度 -500…500 RPM。关闭时速度目标归零。 |
| `21` 右播撒 | 5 | `on:u8, rpm:f32` | 同上；当前默认工作方向为 -200 RPM。 |
| `30` 割草刀盘 | 5 | `on:u8, throttle:f32` | `on` 为 0/1；电调油门 0…100%。 |
| `40` 刀盘升降 | 5 | `on:u8, height:f32` | `on` 为 0/1；距标定零点的**绝对目标高度** 0…1000 mm。当前预留。 |
| `50` 喷洒装置 | 5 | `on:u8, speed:f32` | `on` 为 0/1；速度指令 0…100%。已接入，需 capabilities.bit5=1。 |

PC 即使暂时不用某个装置，也须发送其功能块，并将 `on=0`、目标值置 0。浮点字段不能是 NaN、无穷大或越界值。当前刀盘升降仍未接入，喷洒已接入。在控制使能且未请求停止时，若未被能力位支持的装置 `on=1`，**整帧以 `UNSUPPORTED` 拒绝**，其他装置也不会动作。旧 `0x000F` 板卡仍不能开启喷洒。目标值在 `on=0` 时仍须落入有效范围。

一个全零、未使能、序号为 0 的合法控制帧：

```text
FC FB 01 01 00 00 30 00
01 01 00
10 08 00 00 00 00 00 00 00 00
20 05 00 00 00 00 00
21 05 00 00 00 00 00
30 05 00 00 00 00 00
40 05 00 00 00 00 00
50 05 00 00 00 00 00
BB 1E FD FE
```

`BB 1E` 是这个示例帧的 CRC16 小端字节序。

## 4. 开发板 → PC：完整状态帧 `TYPE=0x81`

所有浮点量均为 `float32` 小端序。`actual` 表示已有传感器反馈；`target` 或 `throttle` 表示控制设定值。PC 应同时检查 `capabilities` 和反馈有效标志。

| ID | LEN | DATA 字节布局 |
|---|---:|---|
| `01` 系统 | 16 | `uptime_ms:u32, last_command_seq:u16, result:u8, mode:u8, faults:u16, capabilities:u16, command_age_ms:u16, rx_error_count:u16` |
| `10` 底盘 | 24 | `linear_mps:f32, angular_radps:f32, left_target_rpm:f32, right_target_rpm:f32, left_actual_rpm:f32, right_actual_rpm:f32` |
| `20` 左播撒 | 13 | `on:u8, target_rpm:f32, actual_rpm:f32, servo_set_angle_deg:f32` |
| `21` 右播撒 | 13 | 同左播撒 |
| `30` 割草刀盘 | 5 | `on:u8, throttle_set_percent:f32` |
| `40` 刀盘升降 | 10 | `on:u8, target_mm:f32, actual_mm:f32, feedback_valid:u8` |
| `50` 喷洒装置 | 10 | `on:u8, target_percent:f32, actual_percent:f32, feedback_valid:u8` |

`last_command_seq` 是最近**帧结构和 CRC 正确**的控制包序号；结合 `result` 判断是否应用。坏 CRC 或坏帧头不会更新序号。`command_age_ms` 是距最近**成功接纳**控制包的毫秒数，上限 65535；尚未接纳过控制包时为 65535。状态帧自身序号独立递增。

| 字段 | 值 | 含义 |
|---|---|---|
| `result` | 0 | OK；控制快照已接纳。 |
|  | 1 | BAD_FRAME；必需块、长度或字段编码错误。 |
|  | 2 | BAD_VALUE；浮点值无效或超限。 |
|  | 3 | UNSUPPORTED；尝试启用未接入的外设。 |
|  | 4 | TIMEOUT；预留结果码；当前超时通过故障位表示。 |
| `mode` | 0 | USART3 尚未接纳控制包，沿用 UART5 控制。 |
|  | 1 | USART3 控制使能且命令新鲜。 |
|  | 2 | USART3 已接管，但当前停机、未使能或超时。 |

`faults` 位：bit0=PC 控制超时，bit1=最近控制周期 CAN 发送失败，bit2=USART3 接收环形缓冲区曾溢出。`capabilities` 位：bit0=底盘、bit1=左播撒、bit2=右播撒、bit3=割草刀盘、bit4=刀盘升降、bit5=喷洒。接入喷洒后的能力配置为 `0x002F`，旧配置为 `0x000F`。未接入的升降状态为 0、反馈无效。喷洒是否可控取决于 bit5，实测百分比是否可用独立取决于 `feedback_valid`；不能因为支持喷洒就假定存在有效传感器反馈。当前 M3508 与播撒电机虽可上报 RPM，但状态帧尚未附带单独的反馈新鲜度标志；PC 应结合系统故障和设备现场状态使用。

## 5. 控制权、失联与异常

1. 上电时沿用现有 UART5 控制。USART3 第一次接纳合法控制快照后取得控制权；**本次上电期间不自动切回 UART5**，避免旧指令重新驱动装置。发送 `enable=0` 或 `stop=1` 可停机，但不会释放控制权。
   UART5 继续使用原有 CmdPort 格式，并按其文档要求校验数据段 CRC16；USART3 帧与 UART5 帧不可互换。
2. USART3 连续超过 **250 ms** 未收到可接纳的完整控制帧，所有运动和作业目标置零，状态 `mode=2` 且 `faults.bit0=1`。再次接纳合法、使能的控制帧即可恢复。
3. CRC 错、帧尾错或版本不支持时直接丢弃；语义错误保持上一次已接纳命令，但不能续期失联计时。PC 可以从 `rx_error_count` 和 `result` 诊断。
4. 上位机启动时先发送未使能的全零快照，观察状态中的 `capabilities`、序号及结果，再发送使能命令。停止时发送 `stop=1` 的完整快照；如果 PC 意外退出，250 ms 超时保护生效。
5. UART 串口不是硬实时总线。超时保护依赖固件控制任务持续运行，不能代替物理急停电路。

## 6. PC 参考代码

`tools/pc_protocol.py` 使用 Python 标准库实现控制帧编码、CRC、状态帧流式分帧与 TLV 解析。运行只读监视示例需要 `pyserial`：

```powershell
python -m pip install pyserial
python tools/pc_protocol.py COM5
```

示例脚本每 50 ms 下发未使能的全零控制快照，并打印完整状态。它不会启动任何执行器。集成到上位机时，更新 `Control` 的全部字段并持续调用 `build_control(command, seq)`；对串口读出的任意长度字节块调用 `StatusParser.feed(data)`，该方法返回零个或多个 `(status_frame_seq, status_dict)`。不要按换行或固定 `read(117)` 分帧。

```python
from tools.pc_protocol import Control, StatusParser, build_control

command = Control(enable=True, linear_mps=0.1)
wire_bytes = build_control(command, seq=1)
parser = StatusParser()
# serial.write(wire_bytes)
# for status_seq, status in parser.feed(serial.read(256)):
#     assert status["last_command_seq"] == 1 and status["result"] == 0
```

新外设扩展时：分配新 TLV ID，定义长度、单位、取值、反馈有效性和能力位；旧 PC 忽略未知状态块，旧固件忽略未知控制块。对当前七个必需块的字节布局保持不变。
