# 相对初始位置的机构缩短量 mm，不是离地高度；能力由 McuState 判定。
# 当前固件将目标限幅至约 38.562 mm，关闭/停止后目标回零，无位置传感器。
bool on
float32 target_mm
float32 actual_mm
# false 时 actual_mm 不应作为可信测量值使用。
bool feedback_valid
