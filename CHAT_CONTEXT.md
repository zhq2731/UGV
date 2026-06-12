# UGV 仿真与冲突消解移植上下文记录

记录时间：2026-06-09

这个文件用于在后续对话或其他环境中快速恢复上下文。当前工程路径为：

```bash
/home/zhq/UGV
```

## 当前目标

把 `/home/zhq/UGV_zhq/src` 轻量化工程中的“冲突预判与消解”能力移植到当前 UGV 仿真工程中。

明确不移植轻量化工程里的速度规划模块，当前工程仍使用自己的单车规划控制链路：

```text
route -> reference_line_node -> planner -> trajectory_follower -> simulate
```

冲突模块只负责：

1. 观察两车候选轨迹。
2. 判断未来是否存在时空冲突。
3. 给每辆车发布 `ConflictConstraint`。
4. 当前工程的 `planner` 根据约束在自己的速度规划结果上做限速/让行/停车处理。

## 采用的架构

当前使用“方案 A：异步一帧延迟”。

流程是：

```text
vehicle_1/planner 发布 /vehicle_1/trajectory_candidate
vehicle_2/planner 发布 /vehicle_2/trajectory_candidate
        ↓
每辆车自己的 conflict_resolver_node 同时订阅两车 candidate
        ↓
/vehicle_1/conflict_resolver_node 发布 /vehicle_1/conflict_constraint
/vehicle_2/conflict_resolver_node 发布 /vehicle_2/conflict_constraint
        ↓
下一轮 planner 循环使用最近一次 conflict_constraint 做速度约束
```

这样保持了分布式风格，每辆车都有自己的冲突判断节点，但输入都能看到两车轨迹。

## 当前关键数据流

两车 planner 输出：

```text
/vehicle_1/trajectory_candidate
/vehicle_2/trajectory_candidate
```

冲突消解模块直接读取当前工程原生数据，不再使用适配器：

```text
/vehicle_1/odomData
/vehicle_1/chassis
/vehicle_1/trajectory_candidate

/vehicle_2/odomData
/vehicle_2/chassis
/vehicle_2/trajectory_candidate
```

冲突消解模块输出：

```text
/vehicle_1/conflict_constraint
/vehicle_2/conflict_constraint
/trajectory_conflict/markers
```

planner 最终输出：

```text
/vehicle_1/trajectory
/vehicle_2/trajectory
```

## 已完成的主要改动

### 1. 新增冲突模块包

已从轻量化工程迁移：

```text
src/conflict_prediction_resolution/
```

当前只编译核心冲突判断与消解：

```text
conflict_prediction_resolution_core
conflict_resolver_node
conflict_constraint_processor
```

已从该模块目录删除轻量化原型中与当前工程无关的：

```text
speed_planner_node
road_network_node
vehicle_kinematic_sim_node
```

### 2. 新增消息

新增：

```text
src/msg/planning_msgs/msg/ConflictConstraint.msg
```

并已加入：

```text
src/msg/planning_msgs/CMakeLists.txt
```

planner 和 conflict_resolver 使用该消息传递让行/通行/限速约束。

### 3. 取消 C++ 适配器

之前做过 `ugv_conflict_adapter.cpp`，后来根据新的方案取消。

现在已经：

```text
删除 src/launch_node/src/ugv_conflict_adapter.cpp
从 src/launch_node/CMakeLists.txt 移除 ugv_conflict_adapter target
从 two-vehicle-distributed-simulate.launch 移除 conflict_adapter 节点
```

冲突消解模块现在直接订阅 UGV 原生话题。

### 4. 改写 conflict_resolver_node 输入

关键文件：

```text
src/conflict_prediction_resolution/include/conflict_prediction_resolution/conflict_resolver_node.hpp
src/conflict_prediction_resolution/src/conflict_resolver_node.cpp
```

现在回调包括：

```cpp
onLocalization(size_t index, localization_msgs::Localization::ConstPtr)
onChassis(size_t index, driver_msgs::ChassisReport::ConstPtr)
onCandidate(size_t index, planning_msgs::TrajectoryPointArray::ConstPtr)
```

定位来自：

```cpp
msg->location.pose.pose
```

速度来自：

```cpp
msg->current_velocity
```

如果档位 `gear_location == 7`，内部认为是倒挡，先取负再取绝对值作为冲突预测速度。

### 5. 冲突模块频率调整为 1Hz

配置文件：

```text
src/conflict_prediction_resolution/config/ugv_two_vehicle_conflict.yaml
```

关键参数：

```yaml
decision_rate: 1.0
heading_compensation_degree: 0.0
```

由于冲突判断变成 1 秒一次，planner 侧的约束超时时间设置为 1.5 秒，避免 1Hz 下每两次判断之间误判超时。

### 6. planner 接入冲突约束

关键文件：

```text
src/planning/path_planner/include/planning_node.h
src/planning/path_planner/src/planning_node.cpp
```

planner 新增：

```text
发布 trajectory_candidate
订阅 conflict_constraint
缓存最近一次 ConflictConstraint
在速度规划之后应用冲突约束
```

当前顺序大致是：

```text
planner_ptr->implement(...)
planner_ptr->getTrajectory(trajectory)
velocityPlanning(trajectory)
publish trajectory_candidate
applyConflictConstraint(trajectory)
publish final trajectory
```

说明：虽然最初讨论过“路径规划后立刻发布 candidate”，但冲突模块需要轨迹点中的 `v` 和 `relative_time` 做时空预测，所以当前实际做法是先做基础速度规划，再发布 candidate，然后下一轮用最近的冲突约束修正最终速度。

### 7. 两车分布式 launch

关键文件：

```text
src/launch_node/launch/two-vehicle-distributed-simulate.launch
```

当前会启动：

```text
/rviz
/vehicle_1/conflict_resolver_node
/vehicle_1/route
/vehicle_1/simulate
/vehicle_1/reference_line_node
/vehicle_1/planner
/vehicle_1/trajectory_follower
/vehicle_1/reference_marker_color_filter
/vehicle_1/planning_marker_color_filter

/vehicle_2/conflict_resolver_node
/vehicle_2/route
/vehicle_2/simulate
/vehicle_2/reference_line_node
/vehicle_2/planner
/vehicle_2/trajectory_follower
/vehicle_2/reference_marker_color_filter
/vehicle_2/planning_marker_color_filter
```

不再启动编队 `cooperation` 节点。

### 8. RViz 设置

关键文件：

```text
src/launch_node/param/rviz/param.two_vehicle.rviz
```

已做过：

```text
1车起点
1车终点
2车起点
2车终点
```

两车 marker 用不同颜色区分：

```text
vehicle_1: 蓝色
vehicle_2: 红色
```

已添加冲突显示：

```text
/trajectory_conflict/markers
```

### 9. RViz 运动开始面板

关键文件：

```text
src/launch_node/include/launch_node/two_vehicle_motion_panel.h
src/launch_node/src/two_vehicle_motion_panel.cpp
src/launch_node/rviz_plugin_description.xml
```

面板发布：

```text
/vehicle_1/chassis_motion_start_cmd
/vehicle_2/chassis_motion_start_cmd
```

用于分别或同时让两车开始/停止运动。

## 重要配置

冲突模块配置：

```text
src/conflict_prediction_resolution/config/ugv_two_vehicle_conflict.yaml
```

当前核心内容：

```yaml
conflict_resolver_node:
  decision_rate: 1.0
  frame_id: map
  conflict_markers_topic: /trajectory_conflict/markers
  enable_longitudinal_retiming: false
  heading_compensation_degree: 0.0
```

车辆输入：

```yaml
vehicles:
  - id: vehicle_1
    localization_topic: /vehicle_1/odomData
    chassis_topic: /vehicle_1/chassis
    candidate_topic: /vehicle_1/trajectory_candidate
    constraint_topic: /vehicle_1/conflict_constraint

  - id: vehicle_2
    localization_topic: /vehicle_2/odomData
    chassis_topic: /vehicle_2/chassis
    candidate_topic: /vehicle_2/trajectory_candidate
    constraint_topic: /vehicle_2/conflict_constraint
```

planner 侧参数在 launch 中设置：

```xml
<param name="enable_conflict_constraint" value="true"/>
<param name="conflict_constraint_timeout" value="1.5"/>
<param name="conflict_timeout_max_speed" value="1.0"/>
<param name="conflict_deceleration_limit" value="1.5"/>
<param name="conflict_stop_buffer" value="0.0"/>
```

## 已验证命令

编译验证通过：

```bash
source /opt/ros/noetic/setup.bash && catkin_make --pkg conflict_prediction_resolution launch_node planner
```

启动两车仿真建议命令：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch launch_node two-vehicle-distributed-simulate.launch
```

## 常用检查命令

检查约束输出：

```bash
rostopic echo /vehicle_1/conflict_constraint
rostopic echo /vehicle_2/conflict_constraint
```

检查候选轨迹：

```bash
rostopic echo -n 1 /vehicle_1/trajectory_candidate
rostopic echo -n 1 /vehicle_2/trajectory_candidate
```

检查冲突 marker：

```bash
rostopic echo -n 1 /trajectory_conflict/markers
```

检查话题连接：

```bash
rostopic info /vehicle_1/conflict_constraint
rostopic info /vehicle_2/conflict_constraint
rostopic info /vehicle_1/trajectory_candidate
rostopic info /vehicle_2/trajectory_candidate
```

## 可用冲突测试场景

文件：

```text
src/conflict_prediction_resolution/launch/se.txt
```

当前文件中记录了可手动发布的起点/终点。

一个已使用的冲突测试场景：

```bash
rostopic pub -1 /vehicle_1/move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -14.20, y: -9.83, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.956804, w: 0.290732}}}"

rostopic pub -1 /vehicle_2/move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -108.43, y: -72.42, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.286475, w: 0.958088}}}"

rostopic pub -1 /vehicle_1/clicked_point geometry_msgs/PointStamped "{header: {frame_id: 'map'}, point: {x: -53.76, y: -70.63, z: 0.0}}"

rostopic pub -1 /vehicle_2/clicked_point geometry_msgs/PointStamped "{header: {frame_id: 'map'}, point: {x: -40.49, y: -89.96, z: 0.0}}"
```

## 当前 git 状态要点

当前工作区有未提交改动，包括：

```text
src/launch_node/CMakeLists.txt
src/launch_node/launch/two-vehicle-distributed-simulate.launch
src/launch_node/param/rviz/param.two_vehicle.rviz
src/msg/planning_msgs/CMakeLists.txt
src/msg/planning_msgs/msg/ConflictConstraint.msg
src/planning/path_planner/include/planning_node.h
src/planning/path_planner/src/planning_node.cpp
src/conflict_prediction_resolution/
```

注意：`src/conflict_prediction_resolution/` 是新增包，整体可能仍处于 untracked 状态。

## 后续建议检查点

1. 启动两车仿真，确认两个 `conflict_resolver_node` 都正常运行。
2. 发布两车起点终点后，确认 `/vehicle_x/trajectory_candidate` 有输出。
3. 确认 `/vehicle_x/conflict_constraint` 每秒更新一次。
4. 在 RViz 中确认 `/trajectory_conflict/markers` 能显示冲突区域或让行提示。
5. 观察车辆接近冲突区时，`YIELD` 车辆是否明显限速或停车。
6. 如果发现冲突模块方向判断偏差，优先调整 `heading_compensation_degree`。
7. 如果 1Hz 下车辆反应太慢，可以考虑提高 `decision_rate` 或增加 planner 侧保守距离。

## 关键设计注意

超时不能理解为“没有冲突”。

当前 planner 侧如果 `conflict_constraint` 超过 `conflict_constraint_timeout` 未更新，会把速度限制到 `conflict_timeout_max_speed`，这是保守处理，避免实车或仿真通信延迟时恢复高速。

当前冲突模块 1 秒判断一次，因此：

```text
decision_rate = 1.0
conflict_constraint_timeout = 1.5
```

两者需要配套。如果后续把 `decision_rate` 改成更高频，可以同步缩短 timeout。

## 2026-06-12：场景 2 纯规则速度规划验证

本轮验证目的：

```text
关闭冲突速度 QP，只使用规则速度规划，判断是否可以完整完成场景 2 的冲突规避。
```

当前配置确认：

```yaml
# src/conflict_prediction_resolution/config/conflict_resolution.yaml
enable_conflict_velocity_qp: false

# src/launch_node/param/planning/planning.yaml
enable_conflict_constraint: true
```

使用场景：

```text
场景 2：一车主路下行，一车侧路汇入
车 1：76606 -> 76368
车 2：76435 -> 76370
```

启动方式：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11334
export ROS_HOME=/tmp/ugv_ros_home_11334_rule
export ROS_LOG_DIR=/tmp/ugv_ros_log_11334_rule
roslaunch launch_node two-vehicle-distributed-simulate.launch rviz:=false
```

场景输入：

```bash
rostopic pub -1 /vehicle_1/move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -14.20, y: -9.83, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.956804, w: 0.290732}}}"

rostopic pub -1 /vehicle_2/move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -108.43, y: -72.42, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.286475, w: 0.958088}}}"

rostopic pub -1 /vehicle_1/clicked_point geometry_msgs/PointStamped "{header: {frame_id: 'map'}, point: {x: -53.76, y: -70.63, z: 0.0}}"

rostopic pub -1 /vehicle_2/clicked_point geometry_msgs/PointStamped "{header: {frame_id: 'map'}, point: {x: -40.49, y: -89.96, z: 0.0}}"

rostopic pub -1 /vehicle_1/chassis_motion_start_cmd driver_msgs/MotionStartCmd "{motion_start: 1}"
rostopic pub -1 /vehicle_2/chassis_motion_start_cmd driver_msgs/MotionStartCmd "{motion_start: 1}"
```

实测现象摘要：

```text
T≈1s ~ T≈3s：
  vehicle_1: PROCEED
  vehicle_2: YIELD
  vehicle_2 最终轨迹被规则速度规划限制，未出现重复几何点。

T≈4s ~ T≈8s：
  conflict_resolver 曾短暂输出 ROLE_NONE。
  由于 conflict_constraint_processor 中加入了最近 YIELD 决策保持，
  vehicle_2 最终轨迹仍保持受限，没有立即释放到原始高速。

T≈9s ~ T≈17s：
  vehicle_2 再次进入 YIELD。
  规则速度规划先进行降速，随后切到停车让行。

T≈18s 之后：
  vehicle_2 实际速度降为 0。
  vehicle_2 final trajectory 变为零速停车轨迹，未发现重复几何点导致控制器崩溃。
  但 vehicle_1 实际速度也变为 0，冲突决策仍持续：
      vehicle_1: PROCEED
      vehicle_2: YIELD
  到 T≈66s 仍未自动释放，vehicle_2 一直等待。
```

本轮结论：

```text
纯规则速度规划可以做到“安全保守停车”，目前没有观察到控制器崩溃或重复几何点问题。

但它还不能称为“完美完成冲突规避”，因为场景 2 中出现了先行车 vehicle_1 也停住，
而冲突决策仍保持 vehicle_1 PROCEED / vehicle_2 YIELD，导致避让车长期等待，无法恢复通行。
```

因此当前主要问题不在 QP，而在冲突决策/释放逻辑：

```text
1. conflict_resolver 的 decision lock 只保持先行/让行关系，但缺少“先行车是否真正通过冲突区”的可靠释放判断。
2. 当先行车也停住时，YIELD 车会继续等待，系统容易进入互相等待。
3. 需要增加以下判断之一：
   - 先行车已经通过 peer/ego 冲突出口，释放该 conflict_id；
   - 先行车长期未移动，重新仲裁或切换为安全重规划；
   - 对 LOCK 状态增加 stale/blocked 判断，避免永久锁定。
```

下一步建议：

```text
优先检查 conflict_prediction_resolution/src/coordination_core.cpp 中 decision_locks_ 的释放条件。
当前锁定释放主要依赖 minimum_lock_hold_time 和 decision_unlock_distance，
但场景 2 中先行车停在冲突相关区域时，该条件可能长期不满足。

建议新增“通过冲突区”或“先行车停滞超时”的释放/重仲裁机制。
在这之前，不建议把问题归因到 QP，也不建议急着恢复 QP 参数。
```
