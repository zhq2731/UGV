# LIOM 第五阶段：正常规划接入、时间预算与闭环仿真（2026-10-09）

本项基于第四阶段已验证的矩形走廊与逐轮重建，接入原 `OpenSpacePlanner` 状态机，
不新增消息类型或换挡状态。第2项已完成原 ROS 车辆跟踪闭环，结果见文末。
下面第1项的记录保留当时验证边界；UGV_real 未修改，未进行实车测试。

## 启用方式

编译 `OPEN_SPACE_BUILD_LIOM=ON`，并在 `openspace.yaml` 设置 `liom_enabled: true`。
默认编译与运行开关均关闭；不安装新依赖。仅新增两项 YAML 参数：

- `liom_enabled: false`：启用优化；同名 ROS 私有参数可由 launch 覆盖。
- `liom_time_budget_ms: 1000.0`：一次完整多段优化的墙钟预算；同名私有参数可覆盖。

编译关闭时不会链接 IPOPT/ADOL-C；若误请求启用，明确警告并保持原搜索模式。
修改开关后需重启 planner，运行参数不是每帧热更新。

实际使用前同时重编优化库和外层 `planner`（头文件新增成员，不能只替换旧节点所加载的库）：

```bash
cd /home/zhq/UGV
source /opt/ros/noetic/setup.bash
catkin_make open_space_planner planner simulate trajectory_follower -DOPEN_SPACE_BUILD_LIOM=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
source devel/setup.bash
```

然后将 `src/planning/open_space_planner/param/openspace.yaml` 的 `liom_enabled` 改为 `true`，
沿用 `roslaunch launch_node open-space-parking-simulate.launch`。
第1项当时未覆盖 UGV 原有 build/devel，也未启动闭环；第2项已重编并验证闭环。
默认运行开关仍为 false，闭环检查入口通过私有参数启用，不修改全局 YAML。

## 实际运行路径

搜索 → 原分段/曲率/速度初值 → LIOM 多段优化 → 原缓存/单段提交 → 原控制器。

启用 LIOM 后，重新搜索/优化只在车辆停稳时进行；未停稳先使用现有紧急制动流程。
无已提交段时，停车消息按实际有符号速度保留运动方向，不使用空缓存的默认倒车标志。
执行已提交轨迹时不重复优化，正常到达换挡点仍从缓存提交下一段。
同一次调用使用搜索已有的局部地图快照和起点世界位姿，未增加地图坐标系。
该适配沿用已有地图/定位接口，不宣称解决它们的时间戳对齐问题。

优化结果必须求解收敛、模型残差达标、完整车身及点间扫掠安全，且没有超预算，才替换搜索轨迹。
优化失败/超时后，所有原搜索分段须用相同地图和 **0.4 m 安全余量** 通过完整扫掠才能回退。
回退不安全则清空缓存并提交现有停车轨迹，不能发布优化的部分结果或未验收结果。
运行开关关闭时，保留原搜索规划行为，不偷偷引入这条更严格的回退路径。
外层 planner 发布泊车轨迹时使用实际交付时间戳，不把搜索/优化耗时当作已执行时间。

## 预算边界

`LiomOptions::max_runtime_ms=0` 保留离线不限时；正常规划总是传入正预算。
多段共享从接口入口计算的一条截止时间，涵盖地图预处理、等待求解锁、建框、求解、输出复核。
IPOPT 在迭代回调中协作终止，各阶段及交付输出前也检查截止时间；超时保持 output 原值。
无法强制打断一次正在进行的导数计算/矩阵分解，所以实测返回时间可能略超设定值。
这是结果采用预算，不是 100 ms 规划周期的硬实时保证；同步求解仅在停车时运行。
安全回退的扫掠检查仍需要额外时间，没有因优化超时省略它。

## 验证入口

原 segment/grid/search 检查保留；新 `liom_planner_check` 调用真实 `implement()`，
需运行在隔离的 ROS master，仅记录显示回调，不发布控制话题。它检查：
运行关闭、优化采用、安全超时回退、不安全回退停车、建框失败回退、缓存换挡段、运动中先制动。
缓存测试只注入停车端点，不模拟控制器跟踪，不能替代闭环仿真。

第1项结束时下一小步是闭环仿真及等时间节点适配检查；现已完成，详见文末第2项。

本机首次真实 ROS 检查在布尔 YAML 解析处崩溃：头文件来自 `/usr/local` 的 yaml-cpp 0.6.3，
裸 `-lyaml-cpp` 却链接系统 0.6.2。构建改用 `find_package(yaml-cpp)` 的配套导入目标，
统一头文件与库后真实状态机检查全部通过，未安装或更换系统依赖。

## 本项验证结果

- 独立 Release 构建：优化库、四个检查程序通过；外层 `planner` 可执行程序编译链接通过。
- 原 segment/grid/search 三项 CTest：3/3 通过，约 3.34 s；六个真实搜索场景全部接受且输出复核通过。
- 5 ms 单段预算测试：约 5.93 ms 返回，IPOPT 状态 5（协作停止），超时输出保持原值。
- 极小多段预算测试：拒绝且保持整个输出原值，未交付已完成的部分轨迹。
- 隔离 ROS master `127.0.0.1:11473` 上真实 `implement()` 七项检查全部通过（`failures=0`）。
  直行优化被采用；极小预算下安全初值原样回退；原扫掠不安全的绕障初值改为停车输出；
  紧贴墙导致建框失败但原扫掠安全时正确回退；缓存换挡正常；前进/倒车运动中先制动且方向正确。
- `OPEN_SPACE_BUILD_LIOM=OFF` 的原规划库构建通过，不链接 IPOPT/ADOL-C。
- 主 planner 构建仍有原 `reference_line_info.cc:145` 的缺少返回值警告，本项未修改道路规划代码。

完整源码副本、构建、ROS 日志：`/tmp/ugv-liom-runtime-bLLiRf`；其中 `planner-check.log`
记录初次正常状态机检查；方向修正后的完整复测见 `final-planner-check.log`。
`build/Testing/Temporary/LastTest.log` 记录三项离线回归。
独立主框架构建通过临时父 CMake 先加载 catkin，未改原 planner 构建脚本。
以上不是控制器跟踪仿真，更不构成可直接上车结论；未提交 git。

## 部署收尾

本项 14 个文件已同步到 `/home/zhq/UGV` 的 `openspaceplanner` 工作区；
内容与测试副本一致（原 C++ 文件混合 CR/LF 换行，核对时忽略行尾差异），`git diff --check` 通过。
从部署后的 UGV 源码重新构建四个检查程序，三项 CTest 仍为 3/3 通过，约 3.42 s；
隔离 ROS 上部署版七项状态机检查全部通过，日志为 `deployed-planner-check.log`。
停车方向修正后，默认关闭构建和主 `planner` 链接检查也再次通过。
隔离 ROS master 已正常关闭，没有残留测试控制节点；未启动闭环控制节点或发布控制话题。
第1项收尾时默认运行开关为 false，UGV 原 build/devel 未覆盖；
第2项已在原工作空间重编并检查闭环。UGV_real 未改，未提交 git。

## 第2项：真实 ROS 控制闭环（2026-10-09）

本轮仅验证闭环、修正影响验证的两处泊车仿真执行器行为并保存复现入口，
没有修改 LIOM 目标函数、搜索权重、0.4 m 余量、控制器参数或重规划容差。
本项完成后停下；UGV_real 未修改，未提交 Git。

### 本轮输入与运行环境

在独立 master `http://127.0.0.1:11483` 上，沿用原
`planner + trajectory_follower + simulate`，不启动底盘驱动或车辆接口节点。
脚本只发布初始化、目标、障碍和运动开始指令；定位和底盘反馈来自车辆模型，
没有把停车端点直接注入规划器。测试不自动加入 CTest。

真实固定世界图由 simulate 生成，并随车辆运动发布自车局部窗口。
保留原 launch 的半宽 40 m（局部窗口实际为 **80x80 m**），仅本轮覆盖分辨率为 0.1 m；
这与旧离线检查使用的 40x40 m 地图不同，未改正常 launch 的地图默认值。
起点为世界坐标 (400000, 4400000, 0)，下表目标为相对该起点的位置。
绕障场景添加中心 (6,0)、长 2 m、宽 4 m 的矩形障碍。

本轮已在 UGV 原 build/devel 重编 open_space_planner、planner、simulate、
trajectory_follower 及三个回归检查；编译缓存 OPEN_SPACE_BUILD_LIOM=ON。
openspace.yaml 的 liom_enabled 仍为 false，只有检查 launch 私有参数启用优化。
未安装或下载新依赖。

### 发现并修正的两个仿真问题

- 原转向在命令回调中积分；换挡约 0.4 s 没有横向命令后，新的目标角吃掉了整段等待时间。
  实测 100 ms 观测窗口出现约 0.94～1.03 rad/s 的跳变，并非优化器主动放宽角速度。
  泊车模式现在只在命令回调中更新目标角，由已有 100 Hz 定时器连续限速积分实际角及航向。
  仅增加一个目标角状态，仍使用 0.23 rad/s 限值，不新增配置参数；道路模式保持原回调积分。
- 零速保持命令的 acc_target=0、brake_pedal>0，理想加速度模型没有踏板动力学，
  原来仍以停车容差内的残速漂移（直线测试约 0.042 m/s）。
  现在目标零速且保持制动时清除 0.05 m/s 容差内的残速；不是在整个泊车过程中直接赋目标速度。

修改文件为 simulate.h、simulate.cpp；优化器和控制器未因这些问题修改。

### 修正后结果

| 场景 | 相对目标 (x,y,航向) | 最终位置误差 | 最大横向误差 | 重规划 |
| --- | --- | --- | --- | --- |
| 前进直线 | (8,0,0) | 0.0073 m | 0 m | 0 |
| 前进→倒车→前进 | (4,4,pi/2) | 0.0579 m | 0.0569 m | 0 |
| 绕过矩形障碍 | (12,0,0) | 0.1155 m | 0.6511 m | 1 |

三个场景都收到 GOAL_REACHED 和控制器 PARKING_COMPLETE，实际速度最终为 0，
前轮按原容差完成回正，所有实际换挡发生在零速，已提交运动段都是正间隔的等时间节点。
用规划参数的 5x3 m 车身与实际占据栅格外包矩形独立 SAT 复核，
10 Hz 观测点未发现车身碰撞或 0.4 m 余量侵入。
该执行轨迹检查是采样检查，不宣称证明观测点间扫掠、实车跟踪和定位误差下的安全。

首次优化耗时约 86/96/393 ms，绕障重规划的优化约 139 ms，均未触发 1000 ms 预算回退。
泊车总用时约 14.6/31.7/79.9 s；这是仿真行驶、换挡和回正总时间，不是搜索耗时。
原三个离线回归 CTest 3/3 通过，约 2.49 s。

转向执行器公式逐周期限制为 0.23 rad/s。10 Hz 接收端有限差分观测峰值约
0.253/0.253 rad/s，反馈和观测不是同一采样时刻，不能把该值当作精确执行器角速度。
检查允许两个 100 Hz 反馈周期的采样量化误差，没有修改执行器或优化器限值。
这不是硬实时或实车转向响应认证。

### 仍然存在的问题

绕障初始搜索有五段，但第4段停车后横向偏差较大：
控制器上报 remaining=0、speed=0，车辆未达到原换挡位姿容差，
规划器按原规则触发 REPLAN_AFTER_SEGMENT_END_MISS，放弃原剩余缓存并重新规划，
随后两段执行完成。这不是地图碰撞、优化失败或预算回退，不能说已消除重规划。

现有控制器接收优化后的几何点和速度，但不是直接执行优化器的所有时域控制量：
纵控仍使用空间预瞄速度及原 PI/停车包络；横控转换几何/速度后，
内部重采样并重新计算曲率，没有直接使用优化输出 a、转角及转角变化率作完整前馈。
本轮证明接口和基本闭环可以运行，不代表优化模型与控制执行严格等价。

下一小步应先定位绕障第4段的跟踪偏差，检查 MPC 内部重采样/曲率处理、
空间速度预瞄与优化轨迹的配合；不先放宽换挡容差，也不立即增加新的优化目标。
本轮不继续实施下一项，不构成直接上车的结论。

### 手动复现

启动终端（先按本文启用方式完成编译）：

```bash
cd /home/zhq/UGV
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11483
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
roslaunch -p 11483 open_space_planner liom_closedloop_check.launch rviz:=true
```

此入口本身不自动发送运动指令；也可以 rviz:=false 无界面运行。
另一个终端使用相同的 source 和 ROS 环境，然后执行：

```bash
liom_closedloop_log="$(mktemp /tmp/ugv-liom-closedloop-XXXXXX.log)"
/usr/bin/python3 "$(rospack find open_space_planner)/test/liom_closedloop_check.py" > "$liom_closedloop_log"
rg '^(RESULT|SUMMARY)' "$liom_closedloop_log"
```

脚本会依次重置起点、设置三组目标、开始运动并记录结果，
因此仅在该隔离仿真中运行；脚本拒绝非本地 11483 master。
RESULT 表示到位/停车/采样安全等检查，允许原安全重规划，replans 必须单独看。
退出 roslaunch 后该独立 master 和三个仿真节点同时停止。

本轮日志、修正前后数据和构建记录：`/tmp/ugv-liom-closedloop-vJYuZc`。
`closedloop.log` 是修正前运行，`fixed-closedloop.log` 是首次修正后运行，
`deployed-closedloop.log` 是最终仓库入口完整复测，SUMMARY 为 [True, True, True]。
`regression.log` 是三个回归检查；SAMPLES 行保留实际轨迹和控制量。
上述表格来自 deployed-closedloop.log，不用修正前的到位结果代替真正停车检查。
脚本语法、package.xml、launch 参数展开及非隔离 master 拒绝检查均通过。
独立 roslaunch/master 已关闭，无残留测试控制节点。

## 停车消息导致控制节点崩溃的修复（2026-10-09）

用户反馈：完成泊车后更新目标，`trajectory_follower` 抛出 `Points is empty` 并退出。
实际日志显示新目标已触发任务重置；崩溃前激活的是规划失败后的原地停车段。
该段有三个位置相同的点（时间 0、0.1、1.0 s），并非空消息，但几何弧长为零。
横向 MPC 仍对其按距离重采样，产生空查询点集，插值校验抛出异常。
原三场景闭环检查每次重置起点，未覆盖连续目标和这种停车消息进入控制节点的情况。

本次仅在 `Controller::latControl()` 中对泊车停车段绕过 MPC，清除旧横向输出，
保持实际前轮角；纵向制动保持原逻辑。补充注释说明首点可保留实际非零车速，
停车判定应看后续点。没有追加虚拟行驶距离、吞掉异常、放宽安全余量或增大优化预算，
没有修改道路模式、正常运动段的 MPC、优化器、launch 或 UGV_real。

原节点在独立 master `127.0.0.1:11493` 收到同形态三点停车消息，
可复现完全相同的异常；修复版重放后仍保持轨迹订阅连接。原工作空间已重新编译
`trajectory_follower`，构建成功，`git diff --check` 通过，未提交 Git。

修复版在原 `planner + trajectory_follower + simulate` 中完成两项真实闭环复测：

- 使用 1000 ms 预算启用 LIOM，起点 (400000,4400000,0)，连续下发相对目标
  (5,0,0)、(10,0,0)，任务之间不发布 `/initialpose`；两次均到位停车并完成前轮回正，
  位置误差约 0.0076 m、0.0064 m，实际速度均为零。
- 只在临时 launch 中覆盖 0.5 ms 预算，放置中心 (6,0)、长 2 m、宽 4 m 障碍，
  目标 (12,0,0)。实际规划器判定搜索回退不安全并连续发布零弧长停车段；
  控制节点未崩溃，观测窗口内 146 条纵向命令的目标速度均为零，车辆保持零速。
  不重置位姿、不清除障碍，继续下发 (-5,0,0)、(-10,0,0)，安全搜索回退均完成泊车，
  位置误差约 0.1516 m、0.1401 m，符合原 0.25 m 到位门限。

本次只修复停车消息的控制入口，LIOM 超时或拒绝的原因仍须与节点崩溃分开分析。
未注入定位或底盘运动反馈，未进行实车测试；独立 master 及仿真节点已关闭。
日志及手动复现脚本位于 `/tmp/ugv-parking-stop-fix-Lyk3FM`：
`pre-launch.log` / `pre-stop.log` 为修复前复现，`fixed-stop.log` 为修复后重放，
`consecutive-goals.log` / `unsafe-goals.log` 为两项闭环结果，`build.log` 为构建记录。
