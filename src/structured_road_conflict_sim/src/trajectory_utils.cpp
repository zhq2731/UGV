#include "structured_road_conflict_sim/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>

namespace structured_road_conflict_sim
{

planning_msgs::TrajectoryPointArray makeTrajectory(const std::string& frame_id,
                                                   const std::string& task_area,
                                                   const std::vector<Pose2d>& points,
                                                   const double speed,
                                                   const ros::Time& stamp)
{
  planning_msgs::TrajectoryPointArray trajectory;
  trajectory.header.frame_id = frame_id;
  trajectory.header.stamp = stamp;
  trajectory.is_forward_shift = true;
  trajectory.task_area = task_area;
  trajectory.close_to_end = false;
  trajectory.shape = 0;
  trajectory.type = 0;

  // 使用最低 0.1m/s 避免 relative_time 计算时除零。
  const double safe_speed = std::max(0.1, speed);
  double accumulated_s = 0.0;
  trajectory.points.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i)
  {
    if (i > 0)
    {
      accumulated_s += distance2d(points[i - 1].x, points[i - 1].y, points[i].x, points[i].y);
    }

    planning_msgs::TrajectoryPoint point;
    point.relative_time = accumulated_s / safe_speed;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = 0.0;
    point.theta = points[i].yaw;
    point.s = accumulated_s;
    point.kappa = 0.0;
    point.dkappa = 0.0;
    point.v = safe_speed;
    point.a = 0.0;
    trajectory.points.push_back(point);
  }

  if (!trajectory.points.empty())
  {
    // 最后一个点作为终点，速度置零方便仿真节点在终点停车。
    trajectory.points.back().v = 0.0;
    trajectory.close_to_end = true;
  }
  return trajectory;
}

nav_msgs::Path makePath(const planning_msgs::TrajectoryPointArray& trajectory)
{
  nav_msgs::Path path;
  path.header = trajectory.header;
  path.poses.reserve(trajectory.points.size());
  // Path 只保留 pose 序列，速度等规划字段留在 TrajectoryPointArray 中。
  for (const auto& point : trajectory.points)
  {
    path.poses.push_back(
        makePoseStamped(path.header.frame_id, Pose2d{point.x, point.y, point.theta}, path.header.stamp));
  }
  return path;
}

bool sampleTrajectoryByS(const planning_msgs::TrajectoryPointArray& trajectory,
                         const double s,
                         Pose2d& pose,
                         double& target_speed)
{
  if (trajectory.points.empty())
  {
    return false;
  }

  if (s <= trajectory.points.front().s)
  {
    const auto& point = trajectory.points.front();
    pose = Pose2d{point.x, point.y, point.theta};
    target_speed = std::max(0.0, point.v);
    return true;
  }

  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    const auto& prev = trajectory.points[i - 1];
    const auto& next = trajectory.points[i];
    if (s > next.s)
    {
      continue;
    }

    const double ds = std::max(1.0e-6, next.s - prev.s);
    const double ratio = clamp((s - prev.s) / ds, 0.0, 1.0);
    // x/y/v 线性插值，yaw 使用归一化角度差避免跨越 +/-pi 时绕远路。
    pose.x = prev.x + (next.x - prev.x) * ratio;
    pose.y = prev.y + (next.y - prev.y) * ratio;
    pose.yaw = normalizeAngle(prev.theta + normalizeAngle(next.theta - prev.theta) * ratio);
    target_speed = std::max(0.0, prev.v + (next.v - prev.v) * ratio);
    return true;
  }

  const auto& point = trajectory.points.back();
  pose = Pose2d{point.x, point.y, point.theta};
  target_speed = 0.0;
  return true;
}

double trajectoryLength(const planning_msgs::TrajectoryPointArray& trajectory)
{
  if (trajectory.points.empty())
  {
    return 0.0;
  }
  return trajectory.points.back().s;
}

}  // namespace structured_road_conflict_sim
