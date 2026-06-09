# 单车视角的结构化 ST 冲突约束。
#
# 冲突协调器为每辆车分别发布一条约束。role=YIELD 时，速度规划应让本车
# 在 target_entry_time 之前不要越过 ego_s_in，并可在 stop_s 前停车等待；
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

# ego_* 是本车进入/离开冲突区的 s 窗口和预测时间窗。
float64 ego_s_in
float64 ego_s_out
float64 ego_t_in
float64 ego_t_out

# peer_* 是对方车辆在同一冲突中的 s 窗口和预测时间窗。
float64 peer_s_in
float64 peer_s_out
float64 peer_t_in
float64 peer_t_out

# earliest_entry_time 是本车原始预测进入时间；
# target_entry_time 是让行车允许进入冲突区的目标时间。
float64 earliest_entry_time
float64 target_entry_time
# stop_s 是建议停车点，max_speed 是当前轻量速度规划直接消费的速度上限。
float64 stop_s
float64 max_speed

# 决策溯源：RULE/SCORE/PRIORITY/LOCK 以及更细的原因码。
bool decision_locked
string decision_source
string decision_reason
