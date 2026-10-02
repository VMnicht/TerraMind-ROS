# PC 控制管理状态；enabled 表示管理器允许输出，不是下位机执行确认。
std_msgs/Header header
bool enabled
bool link_ready
uint64 connection_id
# 使能、停机或故障的可读原因；程序逻辑应优先使用布尔状态和连接代次。
string reason
