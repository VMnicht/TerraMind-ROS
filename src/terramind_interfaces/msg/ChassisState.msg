# 底盘状态。仿真 linear/angular 表示目标；真实固件字段语义需交叉核对。
# 控制接口下发 m/s 和 rad/s，轮速 RPM 仅为反馈；轴侧和减速比尚待确认。
float32 linear_mps
float32 angular_radps
float32 left_target_rpm
float32 right_target_rpm
float32 left_actual_rpm
float32 right_actual_rpm
