Header header
# 0：定时距 1：固定间距	   
int32 policy   
#定时距情况下，时间常数 1.5v+5.0
float32 time 
#固定间距情况下，表示车间距
float32 distance
#停车时的间距
float32 stop_distance
# 在菱形或者三角形情况，辆车之间的横向距离
float32 lateral_offset
#是否使用规划，如果规划误差较大，车队情况下考虑直接使用参考线false -no planning true -planning
uint8  use_path_planning
#出队时候的减速度
float32 deceleration  
int32 membersNum










