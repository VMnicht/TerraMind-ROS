# header.stamp 是 PC 接收时间，不是板卡测量时间；原始导航量保持协议坐标。
std_msgs/Header header
uint64 session_id
uint8 status
uint8 rtk_state
uint32 packet_seq
float64 state_time_s
float64 latitude_rad
float64 longitude_rad
float32 height_m
float32[3] velocity_ned_mps
float32[3] rpy_rad
float32[4] quaternion_wxyz
uint16 age_ms
