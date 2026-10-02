# 左/右播撒装置状态，转速为 RPM，舵机角度为度。
bool on
float32 target_rpm
float32 actual_rpm
# 当前协议只上报舵机设定角度，PC 没有独立的舵机控制字段。
float32 servo_set_angle_deg
