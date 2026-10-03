std_msgs/Header header
uint64 session_id
bool online
bool usable
bool origin_ready
bool extrinsics_calibrated
bool nav_valid
bool nav_fault
bool time_locked
bool gnss_recent
bool simulated
uint8 rtk_state
uint16 age_ms
uint32 packet_seq
# 首版为 receive_time；TIME_LOCKED 不代表已完成 PC 时间同步。
string stamp_reference
string reason
float64 origin_latitude_rad
float64 origin_longitude_rad
float64 origin_height_m
uint64 received_frames
uint64 crc_errors
uint64 version_errors
uint64 assembly_timeouts
uint64 discarded_bytes
uint64 missing_packets
uint64 duplicate_measurements
uint64 rejected_measurements
uint64 board_restarts
uint64 reconnects
# PoseStamped 没有 child_frame_id，接收方通过此字段确认位姿参考点。
string pose_child_frame
# velocity 始终是 IMU 中心速度，向量表达于 header.frame_id。
string velocity_point_frame
