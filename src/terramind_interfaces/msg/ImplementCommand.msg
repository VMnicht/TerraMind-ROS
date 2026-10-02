# 作业装置完整目标：每次发布替换整组字段，关闭装置时发送零目标。
# left/right 指左右播撒器，不是差速底盘车轮。
std_msgs/Header header
bool left_on
# 播撒转速单位 RPM，范围 [-500, 500]。
float32 left_rpm
bool right_on
float32 right_rpm
bool mower_on
# 刀盘油门百分比 [0, 100]，不是实际转速。
float32 mower_percent
bool lift_on
# 升降目标高度 [0, 1000] mm；需板卡支持 bit4。
float32 lift_mm
bool sprayer_on
# 喷洒目标 [0, 100]%；需板卡支持 bit5。
float32 sprayer_percent
