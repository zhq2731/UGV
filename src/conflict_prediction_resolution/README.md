# conflict_prediction_resolution

`conflict_prediction_resolution` 是当前 UGV 仿真工程中的冲突预判与消解模块。它只保留和本工程两车分布式规划链路相关的部分：

- `coordination_core`：纯 C++ 冲突检测与通行顺序决策核心。
- `conflict_resolver_node`：ROS 节点，订阅两车候选轨迹与状态，发布每辆车自己的冲突消解决策。
- `ConflictConstraintProcessor`：planner 内部调用的约束处理类，把冲突决策解释到当前轨迹并修正速度。

已删除轻量化原型中的道路生成、车辆运动仿真、独立速度规划和旧 RViz demo 文件。当前工程继续使用已有的 route、simulate、reference_line、planner、trajectory_follower 链路。

## 数据流

两车仿真启动文件：

```text
src/launch_node/launch/two-vehicle-distributed-simulate.launch
```

核心数据流：

```text
vehicle_1/planner -> /vehicle_1/trajectory_candidate ┐
                                                     ├-> conflict_resolver_node
vehicle_2/planner -> /vehicle_2/trajectory_candidate ┘

conflict_resolver_node -> /vehicle_1/conflict_constraint
conflict_resolver_node -> /vehicle_2/conflict_constraint

vehicle_N/planner:
  1. 生成当前帧候选轨迹
  2. 发布 trajectory_candidate
  3. 读取最近一次 conflict_constraint
  4. ConflictConstraintProcessor 将冲突入口/出口点投影到当前轨迹
  5. 根据 role 和 target_entry_time 修正速度
  6. 发布最终 /vehicle_N/trajectory
```

当前采用异步一帧延迟方案：冲突模块以较低频率读取两车候选轨迹，planner 下一帧使用最新冲突决策。

## ConflictConstraint

`planning_msgs/ConflictConstraint` 不携带旧候选轨迹上的 `s`。消息只保留冲突路段的 map 绝对坐标和时间决策：

```text
role
ego_id
peer_id
ego_entry_point
ego_exit_point
peer_entry_point
peer_exit_point
ego_t_in / ego_t_out
peer_t_in / peer_t_out
earliest_entry_time
target_entry_time
decision_source
decision_reason
```

这样做的原因是 planner 每一帧轨迹都会变化。若消息直接携带上一帧轨迹的 `s_in/s_out/stop_s`，低频冲突决策可能在高频规划中变成过期坐标。现在由 `ConflictConstraintProcessor` 在每一帧把入口/出口 XY 投影到当前轨迹，临时生成本帧使用的 `s_in/s_out/stop_s`。

## 关键参数

冲突判定/预判/决策节点参数：

```text
src/conflict_prediction_resolution/config/conflict_prediction.yaml
```

常用项：

```yaml
decision_rate: 1.0
prediction_horizon: 20.0
footprint_safety_margin: 0.7
conflict_time_clearance: 2.0
stop_margin: 1.0
enable_decision_lock: true
```

planner 是否启用冲突消解由 `src/launch_node/param/planning/planning.yaml` 控制：

```yaml
enable_conflict_constraint: true
```

planner 侧冲突消解算法参数在冲突模块内单独配置：

```text
src/conflict_prediction_resolution/config/conflict_resolution.yaml
```

```yaml
conflict_constraint_timeout: 2.1
conflict_timeout_max_speed: 1.0
conflict_deceleration_limit: 1.5
conflict_stop_margin: 1.0
conflict_stop_buffer: 0.0
conflict_min_smooth_yield_speed: 0.3
conflict_projection_max_lateral_error: 2.0
conflict_projection_min_s_gap: 0.2
```

其中 `enable_conflict_constraint=false` 时，planner 不发布 `trajectory_candidate`、不订阅 `conflict_constraint`，也不执行冲突速度修正；`conflict_stop_margin` 是从投影出的冲突入口点向后预留的停车距离；`conflict_projection_max_lateral_error` 用于判断上一轮冲突路段是否仍落在当前轨迹上。

## 验证命令

编译：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake --pkg planning_msgs conflict_prediction_resolution planner launch_node
```

查看冲突决策：

```bash
source devel/setup.bash
rostopic echo /vehicle_1/conflict_constraint
rostopic echo /vehicle_2/conflict_constraint
```

启动两车分布式仿真：

```bash
./start-simulate.sh
```
