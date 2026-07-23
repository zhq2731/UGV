std_msgs/Header header

# 控制端上报“轨迹已经耗尽并稳定停车”，供规划器触发无档位约束重规划。
uint8 SEGMENT_END_HOLD=1

uint8 state
float64 remaining_distance
float64 actual_speed
