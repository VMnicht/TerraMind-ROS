# 喷洒状态，目标和实际量均为百分比；不是流量单位。
bool on
float32 target_percent
float32 actual_percent
# 实机反馈来源由固件决定；false 时不得把目标值冒充实测值。
bool feedback_valid
