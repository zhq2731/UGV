#pragma once

#include <planning_msgs/TrajectoryPointArray.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>

struct OpenSpace_config;

namespace open_space {

// 可选优化模块由原规划状态机决定是否采用，不能绕过状态机直接下发。
struct LiomOptions {
  int max_iterations = 200;
  int max_penalty_iterations = 3;
  double initial_penalty = 1.0e4;
  double max_runtime_ms = 0.0;  // 0 表示离线不限时；多段共用一个墙钟预算。
};

struct LiomReport {
  int solver_status = -999;
  int penalty_iterations = 0;  // 已执行的外层求解轮数，多段累计，不是 IPOPT 内层次数。
  int corridor_builds = 0;  // 建走廊次数，包含首轮及失败的构建尝试；多段累计。
  bool timed_out = false;
  double residual_squared = 0.0;
  double max_residual = 0.0;
  double elapsed_ms = 0.0;
  // 各阶段累计耗时；总耗时还包含地图预处理、输入转换等开销。
  double corridor_ms = 0.0;
  double solver_ms = 0.0;
  double collision_ms = 0.0;
  std::string message;
  int failed_segment = -1;  // 多段调用失败的位置；成功为 -1。
};

// 输入为现有速度规划生成的同档位轨迹，时间必须严格递增，首尾必须停车。
// 保留首尾位置/航向、档位与消息元数据；失败时不修改 output。
// UGV 曲率沿递增弧长定义，倒车时与车辆模型曲率符号相反。
bool OptimizeLiomFreeSpaceSegment(
    const planning_msgs::TrajectoryPointArray &seed,
    const OpenSpace_config &vehicle,
    planning_msgs::TrajectoryPointArray &output,
    LiomReport &report,
    const LiomOptions &options = LiomOptions{});

// 输入地图沿用 UGV：仅 25 可通行。grid_pose 为地图发布时的局部系世界位姿。
// 保留搜索分段、换挡位姿、各段独立时间轴；走廊余量使用 search_collision_margin。
// 交付复核使用 output_collision_margin 加扫掠补偿；调用方按同一余量复核搜索回退轨迹。
// 全部段完成车身扫掠复核才修改 output；这里只验证一个地图快照，不负责动态避障。
bool OptimizeLiomParkingSegments(
    const std::vector<planning_msgs::TrajectoryPointArray> &seeds,
    const nav_msgs::OccupancyGrid &map,
    const geometry_msgs::Pose2D &grid_pose,
    const OpenSpace_config &vehicle,
    std::vector<planning_msgs::TrajectoryPointArray> &output,
    LiomReport &report,
    const LiomOptions &options = LiomOptions{});

}  // namespace open_space
