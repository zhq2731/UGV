#pragma once

#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/time.h>

#include "structured_road_conflict_sim/common.hpp"

namespace structured_road_conflict_sim
{

// 将离散参考点转换为 planning_msgs 轨迹，填充 s、relative_time、速度等字段。
planning_msgs::TrajectoryPointArray makeTrajectory(const std::string& frame_id,
                                                   const std::string& task_area,
                                                   const std::vector<Pose2d>& points,
                                                   double speed,
                                                   const ros::Time& stamp);

// 仅用于 RViz 显示路径形状，不携带速度/加速度信息。
nav_msgs::Path makePath(const planning_msgs::TrajectoryPointArray& trajectory);

// 按轨迹弧长 s 插值得到连续位姿和目标速度。
bool sampleTrajectoryByS(const planning_msgs::TrajectoryPointArray& trajectory,
                         double s,
                         Pose2d& pose,
                         double& target_speed);

// 当前轨迹长度约定为最后一个轨迹点的累计 s。
double trajectoryLength(const planning_msgs::TrajectoryPointArray& trajectory);

}  // namespace structured_road_conflict_sim
