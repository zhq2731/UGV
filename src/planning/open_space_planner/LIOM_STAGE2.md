# LIOM 第二阶段：无障碍单段核心

本文记录第二阶段及无障碍可视化；新增的栅格约束、多段接口和检查见 [LIOM_STAGE3.md](LIOM_STAGE3.md)。

本阶段仅建立优化核心与 UGV 输入输出适配。没有接入正常规划下发流程，
没有地图、安全走廊或碰撞约束，不能直接用于有障碍场景或实车。
默认 `OPEN_SPACE_BUILD_LIOM=OFF`，原工程不新增 IPOPT/ADOL-C 构建依赖。

## 接口与适配

入口：`open_space::OptimizeLiomFreeSpaceSegment()`。
输入是现有 `TrajectoryPointArray` 和 `OpenSpace_config`，输出仍为同一消息类型。
输入时间严格递增、首尾零速；速度、加速度和时间初值使用原速度规划
`OpenSpacePlanner::GenerateVelocityProfile()` 生成，不另写速度规划算法。
原 `Velocity_Profile_output_os()` 保留，只转调上述同一实现。

- 等时间重采样、连续展开航向、位置减去起点，适配大世界坐标及 ±π。
- 优化变量为 x、y、航向、速度、实际前轮角、加速度、转角速率及总时间。
- 固定首尾位姿、首尾零速和单档位速度符号；端点转角自由但受车辆限角约束。
- UGV 曲率按行驶方向的正弧长定义：`phi=atan(L*direction*kappa)`，
  输出 `kappa=direction*tan(phi)/L`。倒车不能漏掉 direction=-1。
- 遵循前进/倒车不同限速，以及有符号加速度与加减速上限的对应关系。
- 通过最高速度的保守限角满足横向加速度上限；未实现随速度变化的非线性限角。
- 按优化结果重建 s、dkappa、relative_time，保留 header、档位和其余消息元数据。
- 求解不收敛或残差不达标时返回 false，output 不变，不在内部悄悄下发初值。

这是 LIOM 思路的精简适配，不是原始仓库整包复制：
保留了时间/控制代价、运动学残差惩罚、IPOPT/ADOL-C 求解方式。
去掉 ROS navigation、前端搜索、可视化、尚未接入的圆覆盖/走廊变量。
离散模型改为梯形积分，时间步长为 T/(N-1)，避免零速起步时前向欧拉强制首个重复位置。
增加有界的惩罚外循环；每轮权重乘 10，以前轮结果重新初始化。
只有 IPOPT 正常/可接受收敛且残差平方和 ≤1e-4、最大分量残差 ≤1e-3 才接受。
迭代上限返回失败，不再沿用原始代码的宽松返回策略。
ADOL-C 使用串行锁保护本模块的 tape；模块目前不支持并发求解。

## 编译和简单检查

需要已有可用的 UGV 消息与依赖构建，以及 IPOPT（含 MUMPS）、ADOL-C 开发库。
这些是可选模块的额外依赖，没有自动下载或安装。

为了不覆盖正常仿真的 build/devel，建议独立构建：

```bash
source /opt/ros/noetic/setup.bash
source /home/zhq/UGV/devel/setup.bash
liom_build_dir=$(mktemp -d /tmp/ugv-liom-check-XXXXXX)
cmake -S /home/zhq/UGV/src/planning/open_space_planner -B "$liom_build_dir" \
  -DOPEN_SPACE_BUILD_LIOM=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
cmake --build "$liom_build_dir" --target liom_segment_check -j2
# 必须优先加载本次构建的库，避免混入旧 UGV 动态库。
export LD_LIBRARY_PATH="$liom_build_dir/devel/lib:$LD_LIBRARY_PATH"
"$liom_build_dir/devel/lib/open_space_planner/liom_segment_check" \
  /home/zhq/UGV/src/planning/open_space_planner/param/openspace.yaml
(cd "$liom_build_dir" && ctest -R '^liom_segment_check$' --output-on-failure)
```

检查程序直接读取指定车辆 YAML，覆盖前进/倒车直线及圆弧、大世界坐标、
跨 ±π 航向、短段、迭代上限拒绝、无效时间输入拒绝、弱惩罚外循环收敛。
它独立复算输出消息的运动学、限速、加减速度、曲率/转角速率及首尾衔接。
不需要启动 ROS master，不会发布控制或规划话题。

本阶段已使用 openspace.yaml、openspace_lite.yaml、openspace_carla_cybertruck.yaml
分别执行上述检查，均为 failures=0。弱惩罚测试经 3 轮递增达标，迭代上限和
重复时间输入均被拒绝。默认关闭 LIOM 时另行构建原规划库，验证依赖隔离。

## 来源与后续边界

### 优化前后可视化

在桌面终端执行，脚本会独立编译并打开一个含轨迹和四组曲线的窗口：

```bash
bash /home/zhq/UGV/src/planning/open_space_planner/scripts/run_liom_visualization.sh
```

加 `--reverse` 查看倒车，加 `--no-show` 仅生成图片，不打开窗口。
脚本使用 `openspace.yaml`、系统 Python3、NumPy 和 Matplotlib；不安装软件。
输出目录为终端打印的 `/tmp/ugv-liom-view-XXXXXX`，其中保存：

- `liom_comparison.png`：轨迹、速度、加速度、前轮转角、前轮转角变化率对比。
- `before.csv` / `after.csv`：优化前后的时间与状态数据。
- `before_rate.csv` / `after_rate.csv`：按相邻点差分计算的区间平均转角变化率。
- `summary.json`：方向、车辆物理限制、求解耗时与残差。

橙色为优化前，蓝色为优化后，灰虚线为物理限制。角度采用前轮角（deg），
不是方向盘角；变化率采用 deg/s，并非直接导出优化器内部节点 omega。
速度和加速度沿车辆前进方向取正，倒车时保留符号。两条曲线使用各自实际时间轴。
初始路径是人为构造的分段曲率路径，用来观察原语衔接的影响，不能称为实际搜索结果。
没有障碍物、没有发布 ROS/控制话题，也不会启动仿真车辆；图片不构成避障验证。
结果目录在 /tmp，需长期保存时请自行复制；正常工作空间 build/devel 不被覆盖。

已有数据可单独重画，不必重新编译：

```bash
python3 /home/zhq/UGV/src/planning/open_space_planner/scripts/plot_liom.py /tmp/你的结果目录
```

### 算法来源

参考仓库 https://github.com/yakunouyang/liom_local_planner
版本 385cbae86293ee9437ba539167ce5403fd387365，许可证见 `LICENSE.LIOM`。

第三阶段才会接入占据栅格、完整车身安全走廊/碰撞验证、多个换挡段。
第一阶段发现的走廊种子坐标错误，所属环境模块此次没有引入；第三阶段不能照搬该错误。
第四阶段再接入正常规划、明确超时预算与失败回退。当前测试耗时不等于周期保证。
