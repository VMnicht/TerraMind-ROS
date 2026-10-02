# 仅 control_manager 发布此完整快照；后端据此生成串口帧和序号。
std_msgs/Header header
# 必须匹配当前 McuState 的连接代次，重连前的指令会被丢弃。
uint64 connection_id
bool enable
# stop 优先于 enable；软件停止请求不等于物理急停反馈。
bool stop
# 底盘速度：[-0.35, 0.35] m/s，[-2, 2] rad/s，均在 base_link 下表达。
float32 linear_mps
float32 angular_radps
ImplementCommand implements
