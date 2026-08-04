# UGV 开放空间泊车开发交接说明

> 生成时间：2026-07-30  
> 用途：将本文件与 UGV 工程一起复制到新电脑。在新的 Codex 会话开始时，请先让 Codex 完整阅读本文件，再检查实际代码和 `git status`，然后继续讨论或修改。  
> 建议给新 Codex 的第一句话：  
> **“请先完整阅读 UGV 根目录的 `OPEN_SPACE_PARKING_HANDOFF.md`，再检查当前分支、git 状态和其中列出的关键代码；不要立即修改，先向我概括你对当前泊车系统的理解。”**

## 1. 当前目标

当前工作的核心目标是：在不破坏原参考线规划和控制功能的前提下，形成一套可用于单车开放空间泊车的规划、控制和仿真闭环，最终逐步接入比赛 Lite 仿真平台、CARLA 和实车。

当前阶段优先保证：

1. 静态障碍物环境下能够完成一般开放空间泊车，不先区分垂直、平行等泊车类型。
2. 输入可以是点云生成的局部栅格，只有明确观测到的自由区允许通行，未知区不可通行。
3. Hybrid A* 生成包含多次前进/倒车切换的完整路径，但控制器一次只执行一个同档位轨迹段。
4. 正常到达换挡点时优先使用初次搜索缓存的下一段，不重复搜索；缓存段与最新地图冲突时才重新规划。
5. 车辆在收到每个新轨迹段后，必须依次完成停车、档位握手、前轮转角准备，之后才能开放纵向运动。
6. 泊车规划、纵向控制和横向控制尽量与原参考线功能隔离。
7. 后续能够把输出适配成比赛 Lite/CARLA 的油门、制动、转向和档位控制。

暂不作为当前主要范围的功能：

- 基于车位类型的 ROI 模板；
- 动态障碍物预测与冲突消解；
- 完整 Apollo Open Space Trajectory Optimizer；
- 自动识别垂直或平行车位；
- 未知区强行拟合为实体障碍物 polygon。

## 2. 工程与版本基线

工程目录：

```text
/home/zhq/UGV
```

当前分支：

```text
openspaceplanner
```

生成本文件时的最新提交：

```text
d715e86 将横向规划器单独独立出来，并且适配简化版比武仿真软件
```

该提交已经包含泊车专用横向 MPC、泊车纵向控制器、Lite 桥接节点、Lite launch、车辆参数、开放空间参数和一键启动脚本，也包含脚本中用于寻找用户级 `rg` 的 `/home/zhq/.local/bin` 路径。

生成本文件时，工作区相对该提交只有本交接文件尚未纳入 Git。迁移或继续工作前仍必须重新执行：

```bash
cd /home/zhq/UGV
git status --short
git log -1 --oneline
```

不要假设工作区始终与本文生成时完全相同。

## 3. 用户对后续代码修改的要求

后续 Codex 修改代码时需要遵守以下约定：

1. 新增或修改的重要代码使用中文注释。
2. 状态迁移、主要数据处理、坐标变换、控制符号和安全逻辑需要有较详细说明。
3. 保证代码精简，避免为同一功能建立多套重复流程。
4. 泊车专用功能必须由 `open_space_execution_mode` 等开关隔离。
5. 原参考线规划器、参考线纵向控制器和原参考线 MPC 的行为应保持不变。
6. 修改前先读取实际代码，不能只按照历史讨论臆测。
7. 每次修改后至少进行编译；控制或状态机修改应尽量做可重复的仿真验证。
8. 不要在未经讨论的情况下大幅重写横向 MPC；目前希望保留 MPC 框架。
9. 终端默认只保留必要状态迁移、故障和任务切换日志，连续诊断量用 DEBUG 或限频日志。

## 4. 当前系统总链路

### 4.1 工程内置泊车仿真

```text
RViz /initialpose
        │
        ▼
simulate：车辆状态、档位反馈、转向执行器、射线式局部栅格
        │
        ├── odomData
        ├── chassis
        └── /free_space_map
                │
                ▼
planning_node + OpenSpacePlanner
        │
        └── trajectory（一次只发布一个同档位段）
                │
                ▼
trajectory_follower
        ├── 泊车执行状态机
        ├── OpenSpaceMpcLateralController
        └── OpenSpaceLongitudinalController
                │
                ├── auto_chassis_steeringwheel_cmd
                ├── auto_chassis_drive_cmd
                └── auto_chassis_gear_cmd
                        │
                        └── simulate
```

### 4.2 比赛 Lite 平台

```text
Lite: /chatter + /goal_pose + /segmenter/points_freeGridMap
                │
                ▼
lite_parking_bridge
        ├── odomData（后轴中心）
        ├── chassis
        ├── /move_base_simple/goal（后轴中心）
        └── /free_space_map（以后轴中心为原点的局部地图）
                │
                ▼
planner -> trajectory -> trajectory_follower
                │
                ▼
auto_chassis_*_cmd
                │
                ▼
lite_parking_bridge -> /vehicle_control_cmd
```

桥接节点同时把本工程 `trajectory` 转成比赛界面显示需要的：

```text
/SIM_trajectory
```

## 5. PlanningNode 的开放空间模式

关键文件：

```text
src/planning/path_planner/src/planning_node.cpp
src/planning/path_planner/include/planning_node.h
src/planning/planner_common/include/common/planning_config.h
src/launch_node/param/planning/planning.yaml
```

关键开关：

```yaml
enable_open_space_planner: true
enable_conflict_constraint: false
```

当开放空间模式启用时：

- `plannerTypeDecision()` 强制返回 `OPEN_SPACE`；
- 不创建道路速度规划器；
- 不创建冲突约束处理链路；
- 不订阅普通道路障碍物话题；
- 不发布道路重规划消息；
- 订阅 `/free_space_map`；
- 目标可由 `/move_base_simple/goal` 或编队任务提供；
- 只有位姿、目标和开放空间地图均有效时才调用 OpenSpacePlanner。

新目标和新起点都会调用 `resetOpenSpaceTask()`：

- 清理上一条规划轨迹；
- 清理 OpenSpacePlanner 的缓存段和状态；
- 清除旧目标；
- 发布 `/open_space_task_reset`，让控制器和仿真节点同步重置；
- 新起点重置时还会丢弃旧地图，等待时间戳更新后的新局部地图。

仿真中：

- `/initialpose` 用于重新设置车辆起点；
- `/move_base_simple/goal` 用于设置泊车目标；
- 起点重置后旧目标不会继续有效；
- 目标重置后上一任务状态也会清空。

## 6. 栅格地图语义与坐标约定

### 6.1 规划器统一语义

当前 OpenSpacePlanner 的安全规则是：

```text
OccupancyGrid 数值 25：明确可通行
其他数值，包括 0、100、-1：不可通行
```

对应代码在：

```text
OpenSpacePlanner::initializeCurrentMap()
```

因此真实点云地图或桥接地图必须在进入规划器前完成语义转换。不能直接把普通 ROS OccupancyGrid 的 `0` 当作规划器自由格。

### 6.2 工程内置仿真地图

地图生成已经从 planning_node 移到：

```text
src/plug/simulate/src/simulate.cpp
Simulate::parkingGridTimer()
```

该地图模拟激光射线观测：

- 从车辆向外发射多条射线；
- 障碍物之前的已观测空间写成 25；
- 障碍物命中格写成 100；
- 障碍物后方遮挡区域保持未知；
- 规划器只有在值 25 的区域中搜索。

这符合“只有确认自由区可以规划”的保守策略。未知区不会被误当成一个实体 polygon，但会限制当前可执行路径。

相关默认参数位于：

```text
src/launch_node/param/planning/planning.yaml
```

主要值：

```yaml
mapParams_pointNum: 360
mapParams_resolution: 0.2
mapParams_length: 40
mapParams_width: 40
parking_obstacle_length: 5.0
parking_obstacle_width: 2.5
```

### 6.3 Lite/CARLA 局部地图参考点转换

当前 Lite/CARLA 直接发布 `/segmenter/points_freeGridMap` 车体局部栅格，
不再由桥接节点进行全局地图裁切。CARLA 状态和目标以 actor 中心为参考点，
规划器、控制器及 Hybrid A* 车辆包络以后轴中心为参考点，因此参考点转换统一
放在桥接节点完成。

代码：

```text
src/plug/lite_parking_bridge/src/lite_parking_bridge_node.cpp
LiteParkingBridge::chatterCallback()
LiteParkingBridge::goalCallback()
LiteParkingBridge::localGridCallback()
LiteParkingBridge::trajectoryCallback()
```

处理流程：

1. Cybertruck actor 中心到后轴中心的纵向距离为 `1.9069015357 m`。
2. 当前位姿和目标位姿沿各自车头反方向平移该距离。
3. 输入局部栅格坐标由 actor 中心改写为后轴中心，`origin.x` 增加该距离，
   栅格数据不重采样，并发布为 `rear_axle` frame。
4. 规划器轨迹保持后轴中心语义；仅在发布 `/SIM_trajectory` 给 Lite 展示时
   沿轨迹航向加回该距离，恢复 actor 中心轨迹。
5. 输入栅格必须与车辆局部轴对齐；旋转栅格会被桥接节点拒绝，因为当前
   OpenSpacePlanner 不处理 `OccupancyGrid.info.origin.orientation`。

具体偏移参数位于：

```text
src/launch_node/param/vehcile/carla_cybertruck/bridge_param.yaml
```

## 7. Hybrid A* 当前实现

关键文件：

```text
src/planning/open_space_planner/src/hybrid_a_star.cpp
src/planning/open_space_planner/include/hybrid_a_star/hybrid_a_star.h
src/planning/open_space_planner/src/rs_path.cpp
```

当前特点：

- 搜索起点固定为车辆局部坐标 `(0, 0, 0)`；
- 目标由全局目标位姿变换到当前车辆局部坐标；
- 车辆位姿参考点按后轴中心处理；
- 碰撞车身后边界使用 `rear_overhang`，不再错误使用轴距；
- 搜索接近目标时尝试 Reeds-Shepp 解析连接；
- RViz 可显示 Hybrid A* 扩展树和最终 RS 连接段；
- 每次 `Init()` 会先按旧尺寸释放搜索数组，避免第二次目标规划时内存越界崩溃；
- 搜索完成后按前进/倒车方向分成多个轨迹段；
- 轨迹段全部缓存，但只向控制器提交第一段。

需要特别注意：

```cpp
kinodynamic_astar_searcher_ptr_->Init(..., 1.0, map_resolution_, ...);
```

这里的 `1.0 m` 是 Hybrid A* 状态去重栅格的 XY 分辨率，不是碰撞地图分辨率。碰撞检查仍使用输入地图的 `map_resolution_`，例如 0.2 m；运动原语内部也有更细的离散点。但 1 m 状态去重确实偏粗，可能影响狭窄车位搜索质量。后续可以参数化为 0.5 m 或更细，但必须评估三维状态数组内存和搜索耗时，不能直接无条件改成 0.2 m。

当前 Lite/CARLA Cybertruck 主要几何参数：

```yaml
car_length: 6.2735533714
car_width: 2.3895740509
rear_overhang: 1.2298751500
wheel_base: 4.0752487613
steering_angle: 35.0
```

## 8. 轨迹几何检查与速度规划

代码主要位于：

```text
src/planning/open_space_planner/src/openSpacePlanner.cc
```

搜索结果会进行：

- RS 连接连续性检查；
- 分段有效性检查；
- 按弧长重采样；
- 航向推导曲率；
- `dkappa` 计算；
- 点间距、曲率和 `dkappa` 的最小几何质量检查；
- 两点短段补中间点，保证 MPC 至少有三个参考点。

速度规划当前规则：

1. 前进和倒车使用独立最高速度。
2. 根据横向加速度约束限制曲率较大位置的速度。
3. 使用相邻点实际前轮角变化：

   ```text
   delta_i = atan(wheel_base * kappa_i)
   ```

   再按相邻弧长限制速度，保证转角变化率不超过执行器上限。
4. 每个同档位段从零速起步，并在段末降为零速。
5. 前向扫描施加最大加速度限制。
6. 后向扫描保证能够按最大减速度在段末停车。
7. 根据相邻点平均速度生成 `relative_time`，再生成一致的有符号 `v` 和 `a`。

Lite 当前规划参数：

```yaml
trajectory_resample_resolution: 0.2
forward_speed: 1.0
reverse_speed: 0.5
max_acceleration: 0.5
max_deceleration: 1.0
max_lateral_acceleration: 0.5
max_front_tire_steering_rate: 0.23
```

不要直接把最高车速提高到 2 m/s。泊车速度必须同时满足转角变化率、曲率横向加速度、制动距离和控制误差要求。

## 9. 规划器泊车状态机

代码：

```text
src/planning/open_space_planner/include/hybrid_a_star/openSpacePlanner.h
src/planning/open_space_planner/src/openSpacePlanner.cc
```

状态：

```text
WAITING_FOR_INPUT
PLANNING
TRACKING_COMMITTED_SEGMENT
STOP_AT_CUSP
EMERGENCY_BRAKING
REPLAN_AFTER_SEGMENT_END_MISS
GOAL_REACHED
```

正常流程：

```text
PLANNING
  -> 搜索完整路径并切分全部同档位段
  -> 提交第一段，其他段进入 cached_segments_
  -> TRACKING_COMMITTED_SEGMENT
  -> 满足当前段换挡点位置、航向和停稳要求
  -> STOP_AT_CUSP
  -> 用最新栅格检查缓存下一段
       无冲突：直接提交缓存下一段
       有冲突或无缓存：重新搜索
```

当前段执行期间，每个规划周期都会用最新地图检查当前段剩余部分的车身扫掠：

- 地图无效；
- 路径进入未知区域；
- 路径与占用区域冲突；

任何一种情况都会进入 `EMERGENCY_BRAKING`，发布原地停车轨迹；实际停稳后再重新规划。

若控制器已经把轨迹执行到末端并连续停车超过 1 秒，但车辆实际位姿仍不满足换挡点容差：

```text
控制器发布 OpenSpaceExecutionStatus::SEGMENT_END_HOLD
  -> 规划器进入 REPLAN_AFTER_SEGMENT_END_MISS
  -> 清除原缓存段
  -> 从实际位姿无档位方向限制地重新搜索
```

该反馈不再依赖轨迹段编号。当前防旧反馈方式是：

- 只在 `TRACKING_COMMITTED_SEGMENT` 接收反馈；
- 消费后立即清除；
- 新轨迹提交时清除；
- 紧急制动时清除；
- 新任务重置时清除。

关键判定参数：

```yaml
segment_end_position_tolerance: 0.20
segment_end_heading_tolerance: 0.15
stop_speed_tolerance: 0.05
goal_position_tolerance: 0.25
goal_heading_tolerance: 0.0873
```

## 10. 控制节点泊车执行状态机

代码：

```text
src/controller/trajectory_follower/include/trajectory_follower_nodes/controller_node.hpp
src/controller/trajectory_follower/src/controller_node.cpp
```

状态：

```text
IDLE
HOLD_STOP
SHIFT_GEAR
PREPARE_STEERING
EXECUTING
SEGMENT_END_HOLD
STRAIGHTEN_STEERING
PARKING_COMPLETE
```

每个运动轨迹段的执行顺序：

```text
收到新段
  -> 先写入 pending_open_space_trajectory_
  -> HOLD_STOP：持续发布制动，等待实际速度小于阈值
  -> SHIFT_GEAR：周期发布档位命令，等待底盘反馈连续稳定
  -> PREPARE_STEERING：保持停车，原地调整前轮转角
  -> 前轮角连续多帧落入容差
  -> 激活 pending 轨迹
  -> 用当前实际前轮角重置泊车 MPC 的本段控制历史
  -> EXECUTING：MPC 横向控制与泊车纵向控制同时运行
```

`has_pending_open_space_trajectory_` 的意义：

- `true`：新轨迹已经收到，但档位和前轮准备还没有完成，不能让 MPC 和纵控执行；
- `false`：没有等待接管的新段，或者该段已经正式激活。

纯停车轨迹不需要换挡和转角准备，会立即激活，以便覆盖失效的运动轨迹。

前轮准备目标当前根据轨迹第一点曲率计算：

```text
delta_target = atan(wheel_base * direction_sign * kappa_first)
```

倒车时 `direction_sign = -1`，用于修正轨迹排列方向与车头方向的符号关系。当前还没有根据一段预瞄区间曲率自适应选择准备角，后续若首点曲率不能代表起步弯道，可再讨论改为短距离曲率统计。

泊车终点达到后：

```text
GOAL_REACHED
  -> 控制器 STRAIGHTEN_STEERING
  -> 持续制动并下发前轮 0 rad
  -> 连续满足回正容差
  -> PARKING_COMPLETE
```

HOLD_STOP、换挡、转角准备、段末等待和回正阶段均应保持制动，不能只下发零速度而没有制动。

## 11. 档位命令与反馈握手

控制器发布：

```text
auto_chassis_gear_cmd
```

档位约定：

```text
0：空挡
1：前进挡
7：倒挡
```

控制器要求：

- 先停稳；
- 周期发送目标档位；
- 底盘反馈档位等于目标档位；
- 连续满足 `open_space_gear_confirm_stable_cycles`；
- 然后进入 `PREPARE_STEERING`。

工程内置仿真中：

- simulate 接收档位命令；
- 默认模拟 0.30 秒换挡延迟；
- 延迟完成后才通过 `chassis.gear_location` 反馈。

Lite 中：

- bridge 把档位命令转换成 `/vehicle_control_cmd` 的 reverse 标志；
- Lite 静止时原始 gear 可能回到 0，bridge 会结合最近一次已应用档位命令恢复前进挡反馈；
- 倒挡仍以 Lite reverse 状态为准。

## 12. 泊车专用横向 MPC

关键文件：

```text
src/controller/trajectory_follower/include/trajectory_follower/open_space_mpc_lateral_controller.hpp
src/controller/trajectory_follower/src/open_space_mpc_lateral_controller.cpp
```

当前设计：

- `open_space_execution_mode=true` 时创建 `OpenSpaceMpcLateralController`；
- 道路模式继续创建原 `MpcLateralController`；
- 泊车 MPC 拥有独立轨迹缓存、控制历史和 MPC 实例；
- 底层仍复用通用 `MPC`、运动学自行车模型和 QP 求解器；
- 新段激活前使用实际前轮角调用 `resetForNewTrajectory()`；
- 清理上一段控制命令、延迟缓冲、滤波和轨迹形状缓存；
- 不会把车辆真实前轮角清零。

当前 MPC 已有前进/倒车方向符号处理，并使用运动学模型，但倒车控制效果仍需专门回归和参数标定。目前不能认为已经达到实车泊车质量。

Cybertruck 转向执行器映射放在 `lite_parking_bridge`，不放进 MPC：

- MPC 指令和反馈统一使用等效自行车前轮角，并限制在 ±35°；
- 桥接使用 2026-08-03 低速定圆实测奇次多项式完成双向转换；
- 35°对应 CARLA 归一化转向约 0.62257，归一化边界为 ±0.623；
- `/chatter` 归一化反馈通过正向实测模型转换为 `chassis.front_wheel_angle`；
- 70°名义内侧轮 Ackermann 模型仅作为配置回退。

标定报告位于：

```text
calibration_results/CARLA_CYBERTRUCK_STEERING_20260803.md
```

OSQP 判定已经修正：

- 主求解状态为 `SOLVED` 或 `SOLVED_INACCURATE` 时使用结果；
- polish 失败不会丢弃已经成功的主解；
- 非法长度和非有限结果仍返回失败。

Lite 当前 MPC 参数：

```yaml
traj_resample_dist: 0.05
mpc_prediction_horizon: 50
mpc_prediction_dt: 0.1
mpc_weight_lat_error: 4.5
mpc_weight_heading_error: 0.3
mpc_weight_steering_input: 1.0
mpc_weight_terminal_lat_error: 2.0
mpc_weight_terminal_heading_error: 0.5
input_delay: 0.0
vehicle_model_steer_tau: 0.005
steering_lpf_cutoff_hz: 2.0
```

控制诊断话题：

```text
control/open_space_mpc/yaw_error
control/open_space_mpc/desired_tire_angle
control/open_space_mpc/actual_tire_angle
control/open_space_mpc/signed_speed
control/open_space_mpc/reference_pose
lat_error
```

## 13. 泊车专用纵向控制器

关键文件：

```text
src/controller/trajectory_follower/include/trajectory_follower/open_space_longitudinal_controller.hpp
src/controller/trajectory_follower/src/open_space_longitudinal_controller.cpp
```

控制节点只决定是否处于泊车场景；具体输出后端由纵向控制器内部参数选择：

```text
ideal_acceleration
carla_pedal
vehicle
```

### ideal_acceleration

用于工程内置 simulate：

- 按轨迹线段投影得到单调进度；
- 按弧长预瞄目标速度；
- 计算剩余弧长；
- PI 速度闭环输出期望加速度；
- 有最大加速度、最大减速度和 jerk 限制；
- simulate 对加速度积分得到实际速度和位姿。

### carla_pedal

当前用于比赛 Lite：

- 先得到有符号期望加速度 `a_cmd`；
- 再乘档位方向，得到沿实际行驶方向的加减速度；
- 驱动需求映射为油门；
- 减速需求映射为制动；
- 油门、制动互斥；
- 停稳后保持配置的制动踏板；
- `DriveCmd` 内部踏板单位为 0～100%，bridge 转成 Lite 的 0～1。

默认近似关系：

```text
throttle = (沿行驶方向加速度需求 + 滚阻补偿) / 3.0
brake    = (沿行驶方向减速度需求 - 滚阻作用) / 6.0
```

结果会限幅到 `[0, 1]`。

### vehicle

目前只是安全预留接口，尚未实现具体车型油门、制动或扭矩映射。进入该模式会返回失败，由控制节点保持制动。

纵控 `Reset()` 会在新轨迹、换挡或任务重置时清除：

- 轨迹匹配进度；
- 速度误差积分；
- 上一周期加速度；
- 控制周期时间。

## 14. 工程内置仿真功能

关键文件：

```text
src/plug/simulate/src/simulate.cpp
src/plug/simulate/include/simulate.h
src/launch_node/launch/open-space-parking-simulate.launch
src/launch_node/param/rviz/open-space-parking.rviz
```

泊车仿真专用功能：

- `/initialpose` 重置车辆位置；
- `/clicked_point` 可选添加静态矩形障碍物；
- 周期生成射线式 `/free_space_map`；
- 模拟档位切换延迟和实际档位反馈；
- 模拟前轮实际转角变化速度上限；
- 使用加速度积分实现泊车纵向运动；
- RViz 显示车辆、前轮实际角、障碍物、栅格、规划轨迹、A* 扩展和 RS 连接；
- RViz 插件提供“开始运动”按钮。

当前前轮变化上限：

```text
0.23 rad/s
```

启动：

```bash
cd /home/zhq/UGV
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch launch_node open-space-parking-simulate.launch
```

操作：

1. `2D Pose Estimate` 设置或重置车辆。
2. 可选用 `Publish Point` 添加障碍物。
3. `2D Nav Goal` 设置泊车目标。
4. 点击 RViz 面板“开始运动”。

## 15. 比赛 Lite 平台接入

比赛平台不在 UGV 仓库内，当前路径：

```text
/home/zhq/SIM_CODE_LITE_ubuntu20.04_ros1_20260730/
  SIM_CODE_LITE_ubuntu20.04_ros1_20260729
```

参考资料：

```text
/home/zhq/下载/自动泊车比赛仿真接口.pptx
/home/zhq/SIM_CODE_LITE_ubuntu20.04_ros1_20260730/
  SIM_CODE_LITE_ubuntu20.04_ros1_20260729/
  Ubuntu20.04_ROS1_部署与启动说明.txt
```

UGV 侧文件：

```text
src/plug/lite_parking_bridge/
src/launch_node/launch/lite-open-space-parking-planning.launch
src/launch_node/param/control/lite_parking/open_space_controller.param.yaml
src/launch_node/param/vehcile/lite_parking/vehicle_param.yaml
src/planning/open_space_planner/param/openspace_lite.yaml
start-lite-parking.sh
```

一键启动：

```bash
cd /home/zhq/UGV
./start-lite-parking.sh
```

脚本负责：

- 启动或复用 ROS Master；
- 启动 Lite bridge、planner、trajectory_follower；
- 打开比赛 Lite GUI；
- 检查同名遗留节点；
- GUI 关闭或按 `Ctrl+C` 时清理本次进程；
- 日志写入 `/tmp/ugv_lite_parking.*`。

界面打开后：

1. 保持 `ROS control` 勾选。
2. 点击 `Start`，由 GUI 启动 Lite 后端。
3. 点击 `Add Vehicle` 添加 0 号车辆。
4. 使用 `Set goal pose` 设置目标位姿。

后端不由一键脚本直接启动。这是为了让 GUI 正确维护自己的后端运行状态，避免 GUI 与脚本重复启动 `/zheda_lite_backend`。

新电脑上需要修改 `start-lite-parking.sh` 中的 Lite 目录，或设置：

```bash
export LITE_SIM_ROOT=/新电脑上的实际Lite目录
```

脚本当前依赖 `rg`。新电脑推荐安装：

```bash
sudo apt-get update
sudo apt-get install -y ripgrep
```

当前电脑因无法在 Codex 会话输入 sudo 密码，使用的是用户级：

```text
/home/zhq/.local/bin/rg
```

## 16. 已做过的重要验证与已解决问题

已经处理过：

1. 开放空间规划与道路冲突、道路速度规划逻辑隔离。
2. 栅格地图生成从 planning_node 移到 simulate。
3. 自由格数值统一为 25，未知区不可规划。
4. 第一段/多段轨迹缓存和正常换挡点直接提交下一段。
5. 轨迹失效时停车并重新规划。
6. 控制器轨迹耗尽但未满足换挡点时无方向约束重规划。
7. 新起点、新目标时规划、控制、仿真同步清理旧任务。
8. HOLD_STOP 持续制动。
9. 档位命令与实际反馈握手。
10. 每一段执行前先进行前轮准备。
11. 倒车前轮准备角符号修正。
12. 新段激活时用实际前轮角初始化 MPC 历史。
13. 最终到达后前轮回正。
14. 仿真增加前轮角速度上限。
15. 速度规划增加曲率、转角速率、加减速度限制。
16. OSQP 主解与 polish 状态判定修正。
17. Hybrid A* 连续多目标规划时释放旧尺寸数组，解决第二次任务崩溃。
18. Lite 全局地图在 bridge 中裁切并转换为车辆局部地图。
19. Lite 状态、目标、地图、轨迹显示、踏板控制与档位反馈闭环接通。

此前进行过连续多个不同目标的无障碍规划验证，Hybrid A* 能在不同裁切地图尺寸下重复规划而不再因旧数组尺寸崩溃。控制效果仍需继续做系统性回归，不能把“规划成功”视为“控制已经适合实车”。

## 17. 当前最重要的遗留问题

### 17.1 第二段或长距离大角度段的跟踪误差

历史仿真中经常出现第二个同档位段，特别是倒车转弯段，横向误差逐渐增大。

已经解决的相关问题：

- 换挡前没有前轮准备；
- 倒车准备角符号相反；
- MPC 第一帧继承上一段控制历史；
- 规划速度超过实际前轮角速度能力。

仍需继续确认：

- 倒车时 MPC 的误差定义、参考方向和曲率前馈是否完全一致；
- 当前 `kinematics` 模型在低速倒车下的符号是否在所有路径方向上正确；
- 第二段开始的实际位姿与缓存轨迹起点误差；
- 转角准备只使用第一点曲率是否足够；
- Lite 转向归一化、前轮角、方向盘角和转向比是否全链路一致；
- MPC 权重、预测时间和滤波参数是否适配长距离大曲率切换；
- 控制频率、定位频率和执行器延迟是否与参数一致。

推荐固定记录：

```text
轨迹方向
当前档位/目标档位
目标前轮角/实际前轮角
MPC输出前轮角
横向误差
航向误差
车辆实际速度/轨迹目标速度
最近参考点或投影点
剩余弧长
油门/制动
```

### 17.2 实车纵向控制接口未实现

`OpenSpaceLongitudinalController::computeVehicleCommand()` 仍为空接口。后续接实车时应：

- 保留现有轨迹投影和 `a_cmd` 计算；
- 根据车型标定加速度到油门、制动或扭矩的映射；
- 接入真实档位、制动和执行器反馈；
- 增加命令超时、安全制动和故障状态；
- 不把车型映射重新塞回控制节点。

### 17.3 Lite/CARLA 踏板参数仍需标定

当前 3.0/6.0 m/s² 和 0.15 m/s² 滚阻是简化模型参数。应先做：

- 不同目标速度的前进直线；
- 不同目标速度的倒车直线；
- 每种速度的停车距离；
- 前进切倒车、倒车切前进。

如果主要目标只是纵控踏板标定，不需要一开始就做大量复杂泊车场景；但最终完整侧方/垂直泊车仍需要验证横纵向耦合。

### 17.4 Hybrid A* 状态分辨率仍为 1 m

这可能使狭窄环境中的状态合并过于粗糙。建议后续：

1. 把状态分辨率改为 YAML 参数，而不是硬编码。
2. 对 1.0、0.5、0.25 m 分别测搜索成功率、耗时和内存。
3. 必要时把当前三维裸数组改为稀疏哈希结构，再进一步降低分辨率。

### 17.5 真实点云地图接入仍未完成

当前验证的是：

- simulate 生成射线式局部栅格；
- Lite 全局静态图转换成局部栅格。

真实车辆若只提供点云或点云生成的原始栅格，仍需要一个独立感知/桥接模块完成：

- 地面过滤；
- 点云坐标变换和时间同步；
- 障碍物膨胀；
- 射线清空与遮挡未知区；
- 多帧概率或时间衰减融合；
- 车辆自体点去除；
- 输出值 25 的已确认自由区。

规划器不应直接承担点云建图职责。

## 18. 推荐的后续升级顺序

建议按以下顺序继续：

1. **建立 Lite 固定场景数据记录**  
   先能稳定记录第二段开始前后 3～5 秒的位姿、速度、档位、转角、MPC 误差、踏板和参考点。

2. **定位第二段横向误差的唯一主因**  
   分清是初始位姿偏差、倒车符号、转角执行滞后、MPC模型、参考匹配还是速度过高，避免同时改多个参数。

3. **完成泊车 MPC 的前进/倒车回归**  
   固定测试前进左/右弯和倒车左/右弯，再测试前倒切换。

4. **参数化 Hybrid A* 状态分辨率**  
   在不改变碰撞地图语义的情况下比较搜索质量和性能。

5. **标定 Lite/CARLA 纵向踏板映射**  
   先直线前进/倒车不同速度，再做完整泊车。

6. **实现真实点云到安全局部栅格的桥接节点**。

7. **实现 `vehicle` 实车纵控后端和真实底盘安全握手**。

8. **最后再考虑 ROI、车位识别、轨迹优化器和动态障碍物**。

## 19. 常用检查命令

编译：

```bash
cd /home/zhq/UGV
source /opt/ros/noetic/setup.bash
bash compile.sh
```

仅查看关键节点：

```bash
rosnode list | rg 'planner|trajectory_follower|simulate|lite_parking_bridge|zheda_lite_backend'
```

查看关键话题：

```bash
rostopic hz /chatter
rostopic echo -n 1 /map
rostopic echo -n 1 /free_space_map
rostopic echo -n 1 /trajectory
rostopic echo -n 1 /vehicle_control_cmd
rostopic echo -n 1 /open_space_execution_status
```

观察控制诊断：

```bash
rostopic echo /control/open_space_mpc/desired_tire_angle
rostopic echo /control/open_space_mpc/actual_tire_angle
rostopic echo /control/open_space_mpc/yaw_error
rostopic echo /lat_error
```

## 20. 新会话开始时的核对清单

新的 Codex 会话不要直接按本文修改代码，应先执行：

```bash
cd /实际路径/UGV
git status --short
git branch --show-current
git log -3 --oneline
rg --files | rg 'openSpacePlanner|open_space_mpc|open_space_longitudinal|lite_parking_bridge'
```

然后重点读取：

```text
src/planning/path_planner/src/planning_node.cpp
src/planning/open_space_planner/src/openSpacePlanner.cc
src/planning/open_space_planner/src/hybrid_a_star.cpp
src/controller/trajectory_follower/src/controller_node.cpp
src/controller/trajectory_follower/src/open_space_mpc_lateral_controller.cpp
src/controller/trajectory_follower/src/open_space_longitudinal_controller.cpp
src/plug/lite_parking_bridge/src/lite_parking_bridge_node.cpp
src/launch_node/launch/lite-open-space-parking-planning.launch
src/launch_node/param/control/lite_parking/open_space_controller.param.yaml
src/planning/open_space_planner/param/openspace_lite.yaml
```

如果实际代码与本文不同，以实际 Git 状态和代码为准，并先向用户说明差异。
