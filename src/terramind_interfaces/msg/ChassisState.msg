# 底盘状态。linear/angular 是板端应用目标，不是实测速度。
# 真实 RPM 为减速器输出轴：转子 RPM / 19.2032；到车轮的机械传动仍需现场核对。
# 保留固件左右符号；不能直接据此生成车体里程计。
float32 linear_mps
float32 angular_radps
float32 left_target_rpm
float32 right_target_rpm
float32 left_actual_rpm
float32 right_actual_rpm
