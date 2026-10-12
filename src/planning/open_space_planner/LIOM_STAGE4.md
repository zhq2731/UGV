# LIOM 第四阶段进度：真实搜索验证、可视化与走廊修正

本阶段四小项已完成；最新结果见本文“第四项”。前两项的五成功、一拒绝
是当时的历史结果，不代表当前版本。2026-10-09 后续正常规划接入与预算/回退
见 [LIOM_STAGE5.md](LIOM_STAGE5.md)，其第2项已完成原仿真控制闭环，尚未进行实车验证。

2026-10-08 完成第一项：直接调用 UGV 当前 Hybrid A*，使用同一份栅格快照，
经正常路径分段、世界坐标转换、重采样、曲率和原速度规划生成优化初值，
然后调用 `OptimizeLiomParkingSegments()`。本项完成后停下，等待用户安排下一项。

## 代码变更

- 增加 `OpenSpacePlanner::GenerateSearchSegments()` 无状态函数，提取原规划已有逻辑。
  正常规划和本检查程序共用该函数；未向正常规划接入优化调用。
  原有各类路径预处理拒绝日志合并为 `search trajectory preparation failed`。
- 增加 `test/liom_search_check.cpp` 和可选构建/CTest 目标。
- 测试读取现有 YAML 的搜索参数，不另设搜索权重或优化权重。
- 结果独立检查分段档位、首尾位姿和零速、有限值、时间、车辆限值、
  离散运动学与完整车身扫掠。失败时检查输出仍为调用前的值。
- 可选导出 `search_paths.csv`，保存搜索处理后的初值与被接受的优化输出。

## 第一项历史场景与结果

共同条件：openspace.yaml，车长 5 m、车宽 3 m，安全余量 0.4 m，
0.1 m 分辨率、40x40 m 地图，世界位姿 (400000, 3000000, 0.6 rad)。
正常场景地图原点 (-20,-20)；长距离树展开场景原点 (-8,-20)，仍为同尺寸局部图，
目的为在默认 shot_distance=25 m 下确保发生搜索树展开。均使用固定静态地图快照。

| 场景 | 局部目标 (x,y,航向) | 分段 | 搜索耗时 | 优化耗时 | 结果 |
| --- | --- | --- | --- | --- | --- |
| 前进直行 | (8,0,0) | 1 | 18 ms | 95 ms | 成功，输出检查通过 |
| 倒车曲线 | (-6,-1,-0.3) | 1 | 17 ms | 234 ms | 成功，输出检查通过 |
| 停车换挡 | (4,4,π/2) | 3 | 17 ms | 92 ms | 成功，换挡位姿和零速保留 |
| 绕矩形障碍 | (12,0,0) | 5 | 158 ms | 0.2 ms | 求解前拒绝，输出不变 |
| 约 4.3 m 净宽直通道 | (8,0,0) | 1 | 18 ms | 97 ms | 成功，输出检查通过 |
| 搜索树展开后掉头 | (26,0,π) | 2 | 19 ms | 409 ms | 成功，输出检查通过 |

数字为同机单次测量，未设实时周期预算。直行、倒车、三段换挡及窄通道这四例由
搜索器起点的 Reeds-Shepp 解析连接完成；长距离掉头例确实展开搜索树（26 个带父节点状态），
绕障例也展开搜索树（984 个带父节点状态），不是人为构造曲线。

## 第一项发现的限制（第三项已修复本例）

障碍占据局部 x∈[5,7)、y∈[-2,2) 的栅格矩形。搜索成功并产生五段初值，
但当前独立优化入口没有接受该初值，不能计入优化成功场景：

1. 第一段真实矩形车身加 0.4 m 余量能通过扫掠检查，但四圆外接方框无法建走廊。
   起点真实车身前边界约 4.05 m，最后一圆的方框前边界约 5.41 m，
   已伸入 x=5 m 起的障碍；这是覆盖模型的保守拒绝。
2. 第二段的原重采样节点单独检查均通过 0.4 m 余量，但加入节点间插值和额外包络后
   扫掠复核拒绝；零余量的完整车身扫掠通过。搜索的离散检测与优化模块的
   加密/外扩复核尚不能保证同一接受标准。此结果不等同于确认真实车身已经碰撞。
   后续拆开诊断确认：不加额外扫掠包络、只使用 0.4 m 余量，中间插值位姿也会相交。
   首次位于第二段第 15→16 个节点区间约 5/6 处，后轴局部位姿约
   (0.64604,-1.00683,-0.66209 rad)。因此该例并非只有附加包络造成的保守拒绝。
3. 当前优化在第一段构造走廊时即返回失败，IPOPT 未执行；不能靠增加求解迭代数解决。

本检查程序将该场景显式标记 `expected_rejection=1`，检查拒绝原因及输出保持不变。
最后 `accepted=5 rejected=1 check_failures=0` 表示五例优化成功、一个已知限制例
按预期拒绝，不能表述为“六例全部优化成功”或“已解决绕障适配问题”。

## 复现

不需要 ROS master，不启动控制节点，复用已有 ROS 消息和求解依赖。

```bash
source /opt/ros/noetic/setup.bash
source /home/zhq/UGV/devel/setup.bash
liom_search_dir=$(mktemp -d /tmp/ugv-liom-search-check-XXXXXX)
cmake -S /home/zhq/UGV/src/planning/open_space_planner -B "$liom_search_dir/build" \
  -DOPEN_SPACE_BUILD_LIOM=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
cmake --build "$liom_search_dir/build" --target liom_search_check liom_grid_check liom_segment_check -j2
export LD_LIBRARY_PATH="$liom_search_dir/build/devel/lib:$LD_LIBRARY_PATH"
"$liom_search_dir/build/devel/lib/open_space_planner/liom_search_check" \
  /home/zhq/UGV/src/planning/open_space_planner/param/openspace.yaml "$liom_search_dir"
(cd "$liom_search_dir/build" && ctest -R '^liom_(search|grid|segment)_check$' --output-on-failure)
```

## 第二项：有障碍、多段轨迹可视化（2026-10-08）

已增加 `scripts/run_liom_search_visualization.sh` 一键构建/导出/绘图入口，
以及 `scripts/plot_liom_search.py`。检查程序的可选导出增加原始搜索点、实际占据格、
本次车辆参数、地图位姿与求解结果。优化器、安全余量、搜索代价和运行入口均未改动。

在桌面终端运行以下命令，默认打开绕障场景、安全几何放大图和六场景总览：

```bash
bash /home/zhq/UGV/src/planning/open_space_planner/scripts/run_liom_search_visualization.sh
```

可以选择三段换挡等成功场景，加 `--no-show` 只保存图片：

```bash
bash /home/zhq/UGV/src/planning/open_space_planner/scripts/run_liom_search_visualization.sh --scene parking_cusp
bash /home/zhq/UGV/src/planning/open_space_planner/scripts/run_liom_search_visualization.sh --no-show
```

可选场景：`forward_straight`、`reverse_curve`、`parking_cusp`、`obstacle_detour`、
`narrow_straight`、`search_tree_turn`。每次生成六场景的全部图片，`--scene` 仅控制打开哪个。
结果位于终端打印的 `/tmp/ugv-liom-search-view-XXXXXX`，不需要 ROS master：

- `search_overview.png`：实际栅格地图、原始搜索点、搜索初值、被接受的优化轨迹。
- `<scene>_comparison.png`：分段轨迹、速度、加速度、前轮转角、段内转角变化率。
- `obstacle_detour_safety.png`：旧四圆方框问题、当前矩形初始包络，以及第二段初值/优化后的角点运动。
  第二项当时文件名为 `obstacle_detour_rejection.png`，保留在旧结果目录中。
- `search_paths.csv`：世界坐标下的分段初值与成功输出；失败案例没有伪造优化结果。
- `search_raw.csv` / `search_obstacles.csv`：实际搜索点和占据格局部坐标及格子尺寸。
- `search_metadata.json` / `search_diagnostics.json`：参数、求解结果和绘图侧几何复查位置。

图中采用地图局部坐标，按导出的地图位姿转换，原轨迹消息坐标不变。
灰线为原始搜索点，橙线为原速度规划初值，蓝线为被接受的优化输出。
D/R 分段、换挡端点和各自时间轴均保留。转角曲线分段绘制，转角速率只在段内差分，
不把停车换挡的角度突变算成运动中的转角速率；总时间不含换挡或停车调轮等待。
绘图显示实际结果，当前目标函数同时惩罚加速度和调轮速率，不保证总时间比初值短。
六个场景均由检查程序的地图和搜索器产生，部分场景无障碍；没有改造地图伪造优化成功。

已有本版本导出的完整数据可以直接重画，不必重新编译：

```bash
python3 /home/zhq/UGV/src/planning/open_space_planner/scripts/plot_liom_search.py /tmp/你的结果目录 --scene parking_cusp
```

## 第二项收尾记录

第二项收尾记录：segment/grid/search 三项 CTest 全部通过；部署至 UGV 后，
实际执行 `run_liom_search_visualization.sh --no-show` 成功，六场景为 5 个接受、
1 个预期拒绝、0 个检查失败，生成 8 张 PNG 及 CSV/JSON。
本次结果目录为 `/tmp/ugv-liom-search-view-lxSafb`，可直接打开图片或重新绘图。
已检查图片内容；桌面交互窗口未在本次无窗口检查中测试。

本项收尾记录：独立构建成功，CTest 的 segment/grid/search 三项检查全部通过，
总测试时间约 2.14 s。默认关闭 LIOM 的原规划库也单独构建通过。
临时构建和完整日志位于 `/tmp/ugv-liom-search-vWqbDx`；测试 CSV 可用可选参数重新生成。
关键结果已写入本文，临时目录清理不影响交接。

## 第三项：修正走廊保守性与初值点间余量不足（2026-10-08）

问题不只是四圆外接方框太大：圆本身也伸出矩形车身。因此不保留旧圆覆盖模型，
改用车身方向的矩形走廊。这是面向 UGV 栅格地图的工程适配，不是论文圆模型原样复现。

改动仅涉及独立 LIOM 接口、离线检查和已有绘图/记录：

1. 每个优化节点用四个外扩车身角点，共同约束在同一个无障碍凸矩形中。
   走廊整块区域检查占据格，因此不是四个角点各自无碰撞就认为车内安全。
   辅助变量仍为每节点 8 个，总规模仍为每节点 15 个状态/几何量加段时间。
2. 起始矩形沿该节点初值航向，保持 `search_collision_margin=0.4 m`，另加
   `0.25*resolution` 的数值缓冲，在 0.1 m 图中为 0.025 m；不二次膨胀地图。
   每侧最多扩展 1 m。少数贴边内部节点向八邻域一格找无障碍矩形，
   只移动约束框、不伪造安全初值；起终点固定，找不到安全框则拒绝。
3. 不再因输入节点之间的余量不足直接拒绝整条初值；能建安全走廊时允许优化修正。
   输出仍必须满足求解收敛、模型残差和完整车身/点间扫掠三道检查。
   几何辅助量的上下界是硬约束，辅助量与真实角点的一致性仍为残差惩罚。
   所以最终完整车身复核不可省略。任何后段失败仍保持整个 output 原值。
4. 未改变目标函数、搜索权重、0.4 m 余量、残差门限、默认 200 次内层/3 次外层上限。
   试验中单纯将内层上限改为 400 次，保留半格缓冲，仍无法通过第四段的模型残差。
   最终采用上述矩形模型及四分之一格缓冲，恢复默认求解设置后通过。

当前相同六个真实搜索场景全部优化成功，独立输出检查全部通过。
绕障仍为原来的 x∈[5,7)、y∈[-2,2) 障碍、五段、984 个搜索树状态，未更改地图。
第二段原节点间不满足 0.4 m 余量的问题，经优化后通过保留该余量的扫掠复核。
检查程序现在要求该绕障初值确实未通过扫掠，而最终输出通过；不再用预期拒绝充当通过。

| 场景 | 当前优化耗时（同机单次） | 输出复核 |
| --- | --- | --- |
| 前进直行 | 118 ms | 通过 |
| 倒车曲线 | 312 ms | 通过 |
| 三段换挡 | 127 ms | 通过 |
| 五段绕矩形障碍 | 504 ms | 通过，初值余量不足已修正 |
| 约 4.3 m 净宽直通道 | 126 ms | 通过 |
| 搜索树展开后掉头 | 572 ms | 通过 |

并不保证耗时比旧四圆模型短，也不是实时周期上限。三项 CTest 全通过（3/3，约 3.20 s）；
新增旧圆端部过包络回归与走廊内部小障碍拒绝检查，原多段原子输出、旋转/平移扫掠等检查保留。
独立源码/构建及完整结果位于 `/tmp/ugv-liom-corridor-2G38r3`。
绘图更新为动态统计成功数量，并明确旧圆仅用于问题解释；蓝虚线展示修正后的角点运动。

截至本小项仍只建立一次走廊；后续第四项补充了围绕每轮新解重建走廊的外层迭代。
当时尚需完成走廊重建、正常规划接入、运行时预算/超时与失败回退，再做闭环仿真。
`OPEN_SPACE_BUILD_LIOM` 默认仍为 OFF，UGV_real 未改，未提交 git；本项完成后停下汇报。

部署收尾：本项 8 个文件已同步至 `/home/zhq/UGV`，与上述测试副本逐文件一致，
`git diff --check` 通过。实际运行 UGV 的 `run_liom_search_visualization.sh --no-show`
成功，结果为 `accepted=6 rejected=0 check_failures=0`，生成 8 张 PNG 及 CSV/JSON。
部署后重新构建并运行三项 CTest 全通过（3/3，约 3.19 s）；
默认关闭 LIOM 的 `open_space_planner` 原规划库也单独构建成功。
本次实际入口结果目录为 `/tmp/ugv-liom-search-view-zR4j7h`，关键对比图已检查；
桌面交互窗口未在本次无窗口运行中验证。未启动 ROS 节点或发布控制指令。

## 第四项：围绕新解重建走廊与分项耗时（2026-10-09）

本项仅推进独立优化器，不接入正常规划与执行，不改 UGV_real，不提交 git。

1. 将建走廊移到现有惩罚外层循环中。首轮使用等时间初值，后续使用上一轮求得的
   位置、航向和段时间，不再重新采样原搜索轨迹。沿用同一份地图快照与占据前缀和。
   每轮重新计算角点的走廊局部坐标，避免把旧框下的辅助坐标当作新框下的坐标。
2. 残差达标且完整车身/点间扫掠通过即结束，不为了重建强制跑满外层上限。
   建框失败、IPOPT 不收敛、残差超限或最终扫掠失败，仍拒绝且保持 output 原值。
   这不是失败后随意更换拓扑或无限重试；默认仍为每段内层 200 次、外层最多 3 轮。
3. 走廊扩展改为四边轮流各扩一格，遇障碍停下该边；每边仍最多扩展 1 m。
   初次尝试保留旧“单边一次扩满”的方式时，五段绕障第四段在三轮后仍失败：
   最大残差约 0.00169，主要为末端相邻节点的航向运动学残差，而非求解器未收敛。
   四边轮换后，同段第二轮最大残差约 0.000508，恢复通过原门限。
   该对照说明扩展顺序会影响本场景的可行域和收敛，不代表所有场景均更快。
4. `LiomReport` 新增 `corridor_builds/corridor_ms/solver_ms/collision_ms`；多段成功或
   中途失败都累计已发生的工作。`penalty_iterations` 同样为各段外层求解轮数之和。
   建框次数包含失败尝试，未必与求解轮数相同；成功时每轮建框一次，两者一致。
   求解计时包含问题构造、导数准备和 IPOPT 求解；总耗时另含地图初始化、输入输出转换等。
   真实搜索检查同时打印并导出这些字段到 `search_metadata.json`，已有绘图入口保留。

未改变目标函数、0.4 m 余量、四分之一格建框缓冲、运动限值或残差门限。
新增低初始惩罚回归：前三轮逐轮重建，最终通过完整扫掠；只允许一轮则拒绝且保持输出。
同时检查后段失败不会抹掉前段求解耗时。当前仍是矩形模型的工程适配，不是论文完整复现。

下一小步是正常规划链路接入及运行时预算/失败回退，之后再做闭环仿真。
`OPEN_SPACE_BUILD_LIOM` 默认仍为 OFF，尚不能直接宣称在线运行或实车使用就绪。

本项验证：独立 Release 构建成功，segment/grid/search 三项 CTest 全通过（3/3，约 3.34 s）；
低惩罚测试为 3 轮求解、3 次建框，最终安全；单轮耗尽和后段失败统计回归均通过。
原六个真实 Hybrid A* 场景均为 `output_valid=1`，绕障仍是五段、984 个搜索树状态，
仍要求原初值扫掠失败、优化输出在 0.4 m 余量下通过；没有改地图或降低检查要求。
默认关闭 LIOM 的原 `open_space_planner` 库独立构建通过。

同机原版/本版单次对照如下，仅供排查开销，不是性能基准或实时周期保证。
“外层合计”跨段累加，例如五段绕障的 6 轮表示其中一段多求解一轮，不是每段 6 轮。

| 场景 | 原版总优化耗时 | 本版总优化耗时 | 本版外层合计/建框次数 |
| --- | --- | --- | --- |
| 前进直行 | 118 ms | 119 ms | 1/1 |
| 倒车曲线 | 313 ms | 291 ms | 2/2 |
| 三段换挡 | 127 ms | 127 ms | 3/3 |
| 五段绕障 | 507 ms | 525 ms | 6/6 |
| 窄直通道 | 126 ms | 126 ms | 1/1 |
| 搜索树展开后掉头 | 576 ms | 563 ms | 2/2 |

绕障本版约 16.3 ms 建框、503.3 ms 求解、3.2 ms 最终碰撞复核；其他五场景建框均小于 0.3 ms。
在这些用例中主要耗时位于求解阶段，不能把矩形适配后的全部耗时增长归因于地图碰撞计算。
完整源码副本、构建和日志：`/tmp/ugv-liom-rebuild-bYpyjC`。历史日志保留首次回归及修复前后对照。

部署收尾：本项 7 个文件已同步至 `/home/zhq/UGV`，与测试副本逐文件一致，
`git diff --check` 通过。实际运行 UGV 的 `run_liom_search_visualization.sh --no-show`
成功：`accepted=6 rejected=0 check_failures=0`，生成 8 张 PNG 及 CSV/JSON；
绕障优化约 530 ms。结果目录 `/tmp/ugv-liom-search-view-v9GNDM`，已查看绕障曲线和安全几何图。
随后从部署后的 UGV 源码编译并运行三项 CTest，仍为 3/3 全通过，约 3.34 s。
本项完成后停下；未启动 ROS 节点、未发布控制指令、未验证桌面交互窗口。
