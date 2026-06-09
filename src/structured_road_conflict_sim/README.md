# structured_road_conflict_sim

`structured_road_conflict_sim` 是一个轻量化 ROS1/C++ 多车冲突预判与消解原型包。它用于在结构化道路场景中验证多车候选轨迹的冲突检测、让行顺序判断、纵向重定时和速度约束下发。

当前工程的核心定位是：

- 路径几何由上游或场景节点给出。
- 冲突协调只改变纵向速度、时间和通过顺序。
- 决策结果可以通过结构化 ST 约束发送给速度规划节点。

## 架构

工程分为三层：

- `coordination_core`：纯 C++ 算法层，不依赖 ROS。负责 pairwise 冲突检测、通行/让行判断、决策锁定、让行车限速和轨迹重定时。
- `conflict_resolver_node`：ROS 适配层。订阅车辆状态和候选轨迹，调用协调核心，并发布批准轨迹、速度限制、结构化 ST 约束和调试 marker。
- `road_network_node` / `vehicle_kinematic_sim_node`：轻量结构化道路仿真层。前者生成测试场景轨迹，后者沿批准轨迹运动并消费速度约束。

## 主要接口

轨迹消息复用工程内的 UGV 风格消息：

```text
planning_msgs/TrajectoryPoint
planning_msgs/TrajectoryPointArray
```

本次升级新增结构化 ST 约束消息：

```text
planning_msgs/ConflictConstraint
```

常用 topic：

```text
/vehicle_N/trajectory_candidate    # 候选轨迹
/vehicle_N/approved_trajectory     # 协调器批准/重定时后的轨迹
/vehicle_N/speed_limit             # 兼容旧接口的标量速度上限
/vehicle_N/conflict_constraint     # 结构化 ST 冲突约束
/vehicle_N/current_pose            # 当前位姿
/vehicle_N/odom                    # 当前速度
/vehicle_N/planned_path            # 参考路径可视化
/vehicle_N/vehicle_markers         # 车辆可视化
/trajectory_conflict/markers       # 冲突窗口和状态文本
/structured_road/markers           # 道路可视化
```

## 结构化 ST 约束

`ConflictConstraint.msg` 是单车视角的约束：

```text
role: ROLE_NONE / ROLE_PROCEED / ROLE_YIELD
ego_s_in, ego_s_out, ego_t_in, ego_t_out
peer_s_in, peer_s_out, peer_t_in, peer_t_out
earliest_entry_time
target_entry_time
stop_s
max_speed
decision_locked
decision_source
decision_reason
```

含义：

- `ROLE_NONE`：本车当前无 active 冲突。
- `ROLE_PROCEED`：本车被授权先行，消息保留冲突窗口用于诊断。
- `ROLE_YIELD`：本车应让行，速度规划应在 `target_entry_time` 前避免越过 `ego_s_in`，必要时在 `stop_s` 前停车。
- `max_speed`：当前轻量仿真节点直接消费的速度上限。
- `decision_source` / `decision_reason`：记录本次让行顺序由什么机制确定。

## 冲突检测

旧版本更接近“按时间同步采样后找预测碰撞点”。当前版本改为“先找每辆车自己的冲突区间，再判断时间窗是否冲突”。

处理流程：

1. 对一对车辆分别沿自身轨迹按弧长 `s` 扫描。
2. 找到 footprint 重叠对应的 `s_in / s_out`。
3. 通过轨迹点 `relative_time` 或速度积分得到 `t_in / t_out`。
4. 如果两车时间窗在 `conflict_time_clearance` 内重叠，则认为该 pair 存在 active 冲突。

这种方式不再依赖漂移的 `collision_point` 做控制目标。`collision_point` 只作为兼容字段保留，RViz 中也不再强调碰撞点球体、冲突圆柱和箭头，避免对调试判断造成干扰。

## 避让顺序判断

当前 `chooseOrder()` 的逻辑按优先级从硬到软递进：

1. 决策锁复用：如果 pair 已锁定且未满足解锁条件，沿用原顺序，`source=LOCK`。
2. 硬规则：若某车已经在冲突区内，或已经无法在 `s_in` 前安全停车，则该车先行，`source=RULE`。
3. 集中式级联规则：已经在其他冲突中让行的车尽量继续让行，避免链式抢行，`source=RULE`。
4. 软评分：综合优先级、速度、路线进度、TTC 和让行延误，分高者先行，`source=SCORE`。
5. 优先级兜底：评分接近时用静态优先级决定，`source=PRIORITY`。

常见 `decision_reason`：

```text
inside_conflict_interval
cannot_stop_before_s_in
already_yielding_in_central_order
weighted_priority_speed_progress_ttc_delay
score_tie_priority_fallback
released_lock_score_margin_tie
keep_locked_order
```

终端日志示例：

```text
conflict decision ego=vehicle_2 pair=(vehicle_1,vehicle_2)
proceed=vehicle_1 yield=vehicle_2
source=SCORE reason=weighted_priority_speed_progress_ttc_delay
score=(0.388759,0.367679) locked=false
```

速度节点消费约束时会输出：

```text
speed planner constraint vehicle=vehicle_2 role=YIELD peer=vehicle_1
target_entry_time=19.3493 stop_s=18.4991 max_speed=0.740939
source=LOCK reason=keep_locked_order
```

## 评分项

硬规则无法直接判定时使用软评分：

```text
score =
  priority_weight    * priority_component
+ speed_weight       * normalized_current_speed
+ progress_weight    * route_completion_ratio
+ ttc_weight         * ttc_component
+ yield_delay_weight * yield_delay_component
```

解释：

- `priority_component`：静态优先级，数字越小优先级越高。
- `normalized_current_speed`：速度越高，临时停车代价越大。
- `route_completion_ratio`：越接近通过冲突区域，越倾向继续先行。
- `ttc_component`：TTC 越小，越不适合临时让行。
- `yield_delay_component`：让行导致的时间代价越大，越倾向先行。

## 决策锁定

决策锁用于保证迭代一致性。临近冲突区时，如果每个周期都重新评分，可能出现先行/让行角色反复切换；锁定机制会把 pair 顺序保持住。

关键参数：

```yaml
enable_decision_lock: true
decision_lock_distance: 6.0
decision_lock_ttc: 3.0
decision_unlock_distance: 9.0
minimum_lock_hold_time: 2.0
decision_switch_margin: 0.15
```

逻辑：

- 任一车距离冲突入口小于 `decision_lock_distance`，或 TTC 小于 `decision_lock_ttc`，即可锁定当前 pair 顺序。
- 锁定后至少保持 `minimum_lock_hold_time`。
- 只有两车都距离冲突入口大于 `decision_unlock_distance` 后，才允许重算。
- 解锁重算时需要超过 `decision_switch_margin` 才能改变顺序，否则用优先级兜底。

## 场景

当前 YAML 中只需要在 `road_network_node` 下设置 `scenario`：

```yaml
road_network_node:
  scenario: left_right_turn_conflict
  vehicle_count: 0
```

`vehicle_count: 0` 表示按场景自动选择车辆数。resolver 会读取 `/road_network_node/scenario` 推断有效车辆数，不需要在每个节点重复设置 `scenario`。

已支持场景：

- `unprotected_left_turn`：无保护左转与直行车冲突。
- `left_right_turn_conflict`：左转车与右转车汇入同一出口车道。
- `four_way_straight`：四车十字路口直行冲突。

## 关键参数

配置文件：

```text
config/structured_road_conflict_sim.yaml
```

核心协调参数：

```yaml
prediction_horizon: 20.0
footprint_safety_margin: 0.7
conflict_time_clearance: 2.0
minimum_yield_speed: 0.0
yield_stop_time_threshold: 1.0
comfortable_deceleration: 2.0
stop_margin: 1.0
retiming_acceleration_limit: 1.4
retiming_deceleration_limit: 2.5
priority_weight: 0.0
speed_weight: 0.25
progress_weight: 0.25
ttc_weight: 0.25
yield_delay_weight: 0.25
score_tie_epsilon: 0.02
decision_lock_distance: 6.0
decision_lock_ttc: 3.0
decision_unlock_distance: 9.0
minimum_lock_hold_time: 2.0
decision_switch_margin: 0.15
```

说明：

- `prediction_time_step` 已删除；当前不再按统一时间步扫描冲突。
- resolver/core 层 `conflict_radius` 已删除；控制逻辑不再围绕单个碰撞点半径展开。
- road network 内部仍可保留道路绘制或场景构造需要的几何参数。

## 运行

编译：

```bash
cd /home/zhj/UGV_source/UGV_zhq
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

启动默认场景：

```bash
roslaunch structured_road_conflict_sim two_vehicle_conflict.launch
```

不开 RViz：

```bash
roslaunch structured_road_conflict_sim two_vehicle_conflict.launch rviz:=false
```

检查结构化约束：

```bash
rostopic echo -n 1 /vehicle_2/conflict_constraint
```

## RViz

推荐关注：

- `/structured_road/markers`：道路中心线、停止线、车道等。
- `/vehicle_N/planned_path`：候选参考路径。
- `/vehicle_N/vehicle_markers`：车辆运动。
- `/trajectory_conflict/markers`：冲突状态文本、每辆车的 `s_in / s_out` 边界和时间窗信息。

不再推荐把碰撞点球体、冲突圆柱和箭头作为主要调试依据，因为它们容易让人误以为控制目标是单个几何点；当前控制目标是本车自身轨迹上的 ST 窗口。

## 已验证

最近验证命令：

```bash
source /opt/ros/noetic/setup.bash && catkin_make
timeout 12s roslaunch structured_road_conflict_sim two_vehicle_conflict.launch rviz:=false
```

验证结果：

- `ConflictConstraint.msg` 成功生成。
- `structured_road_conflict_sim_core`、`conflict_resolver_node`、`vehicle_kinematic_sim_node` 编译通过。
- 终端能显示 `source=SCORE`、`source=LOCK` 等决策来源。
- 速度仿真节点能收到 `ROLE_YIELD` 约束并消费 `max_speed`。

## 后续扩展方向

- 将 `vehicle_kinematic_sim_node` 当前的 `max_speed` 消费替换为真正的 ST graph 速度规划。
- 对多 pair 冲突构建图优化或小规模 QP/NLP，以替代顺序 pairwise 消解。
- 把 `ConflictConstraint` 扩展为数组消息，一次发布本车所有 active 冲突约束。
- 接入真实 UGV 轨迹规划节点时，保持 `trajectory_candidate` 和 `conflict_constraint` 接口不变，只替换场景轨迹生成和速度规划实现。
