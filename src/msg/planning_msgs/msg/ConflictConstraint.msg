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

# 避让策略由冲突判定层给出，速度消解层只负责执行。
# 这样可以避免速度层根据某一帧 current_entry_time 自行决定“要不要让行”，
# 导致冲突判定层认为需要避让、速度层却释放车辆的语义不一致。
uint8 STRATEGY_NONE=0
uint8 STRATEGY_PROCEED=1
uint8 STRATEGY_STOP_AND_WAIT=2
uint8 STRATEGY_SLOW_DOWN=3
uint8 STRATEGY_FOLLOW=4
uint8 STRATEGY_EMERGENCY_STOP=5
uint8 yield_strategy

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

# 跟车策略附加标记。FOLLOW 时 peer_id 表示前车，速度层自行订阅前车状态高频计算间距和速度。
bool has_follow_constraint

# FOLLOW 初始化兜底快照。刚进入 FOLLOW、速度层还未收到前车高频定位/底盘时，
# 可短时间使用该低频快照计算安全速度上限，避免订阅建立瞬间误触发急停。
bool has_follow_peer_snapshot
geometry_msgs/Point follow_peer_position
float64 follow_peer_speed

# 决策溯源：RULE/SCORE/PRIORITY/LOCK 以及更细的原因码。
bool decision_locked
string decision_source
string decision_reason
