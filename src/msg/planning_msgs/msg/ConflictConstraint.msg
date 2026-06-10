# 单车视角的冲突预判与消解决策。
#
# 冲突协调器为每辆车分别发布一条约束。role=YIELD 时，速度规划应让本车
# 在 target_entry_time 之前不要进入 ego_entry_point 到 ego_exit_point 表示的冲突路段；
# role=PROCEED 主要表示本车被授权先行，也保留冲突窗口用于诊断和可视化。

std_msgs/Header header

# 本车与该冲突的关系：无冲突、先行、让行。
uint8 ROLE_NONE=0
uint8 ROLE_PROCEED=1
uint8 ROLE_YIELD=2

# conflict_id 用于区分一对车在一段 s 区间形成的冲突。
string conflict_id
string ego_id
string peer_id
uint8 role

# 是否携带空间冲突路段。false 时，下面四个 Point 不能作为有效投影依据。
#
# 冲突路段在 map 坐标系下的入口/出口点。planner 每一帧把这些点投影到
# 当前轨迹上重新计算临时 s_in / s_out / stop_s，避免在消息里携带上一帧轨迹的 s。
# 这样可以适应 planner 每帧轨迹略有变化的情况：冲突模块低频给出地图空间约束，
# planner 高频把空间约束重新解释到当前轨迹坐标系。
bool has_spatial_constraint
# 本车进入/离开冲突重叠区域的地图点。
geometry_msgs/Point ego_entry_point
geometry_msgs/Point ego_exit_point
# 对方车进入/离开同一冲突重叠区域的地图点，主要用于诊断和可视化。
geometry_msgs/Point peer_entry_point
geometry_msgs/Point peer_exit_point

# ego_* 是本车进入/离开冲突区的预测时间窗。
float64 ego_t_in
float64 ego_t_out

# peer_* 是对方车辆在同一冲突中的预测时间窗。
float64 peer_t_in
float64 peer_t_out

# earliest_entry_time 是本车原始预测进入时间；
# target_entry_time 是让行车允许进入冲突区的目标时间。
float64 earliest_entry_time
float64 target_entry_time

# 决策溯源：RULE/SCORE/PRIORITY/LOCK 以及更细的原因码。
bool decision_locked
string decision_source
string decision_reason
