# 完整板卡状态；header.stamp 是 PC 接收/仿真生成时间，不是传感器采样时间。
std_msgs/Header header
# PC 生成的非零连接代次，重新连接或仿真复位后变化，不是板卡序列号。
uint64 connection_id
# PC 会话握手与反馈健康判断，不代表各个传感器都有效。
bool link_ready
bool simulated
# 状态帧序号，uint16 正常回绕。
uint16 frame_seq
# 板卡上电时间 ms，uint32 正常回绕。
uint32 uptime_ms
uint16 last_command_seq
# 0 接纳；1 帧错误；2 参数错误；3 能力不支持；4 协议保留。
uint8 result
# 0 尚未接管；1 运行；2 已接管但停机/禁用。
uint8 mode
# bit0 命令超时；bit1 CAN 故障；bit2 接收缓冲区溢出；其余保留。
uint16 faults
# bit0 底盘；bit1/2 左/右播撒；bit3 刀盘；bit4 升降；bit5 喷洒。
# 当前板卡 47，旧板卡 15，全功能模拟器可设 63。
uint16 capabilities
# 最近接纳指令年龄 ms；饱和值/尚未接纳时为 65535。
uint16 command_age_ms
uint16 rx_error_count
# 以下为嵌套状态，不单独发布各装置话题。
ChassisState chassis
SpreaderState left_spreader
SpreaderState right_spreader
MowerState mower
LiftState lift
SprayerState sprayer
