# 升降状态，目标和实际位置均为 mm；是否支持由 McuState.capabilities 判定。
bool on
float32 target_mm
float32 actual_mm
# false 时 actual_mm 不应作为可信测量值使用。
bool feedback_valid
