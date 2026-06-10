#include "conflict_prediction_resolution/conflict_constraint_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include "common/pnc_point.h"
#include "math/linear_interpolation.h"
#include "math/path_matcher.h"

namespace conflict_prediction_resolution
{
namespace
{

using ugv::common::math::PathMatcher;
using ugv::common::math::lerp;
using ugv::planning::PathPoint;

std::vector<PathPoint> makePathPoints(const planning_msgs::TrajectoryPointArray& trajectory)
{
  // PathMatcher 只接受 planner_common 的 PathPoint。这里把本轮轨迹临时转换成
  // PathPoint 序列，用于将冲突模块给出的 map 坐标入口/出口点投影回当前轨迹 s。
  // 注意：这里不改变原 trajectory，只提供投影查询用的几何路径。
  std::vector<PathPoint> path_points;
  path_points.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points)
  {
    PathPoint path_point;
    path_point.set_x(point.x);
    path_point.set_y(point.y);
    path_point.set_z(point.z);
    path_point.set_theta(point.theta);
    path_point.set_s(point.s);
    path_point.set_kappa(point.kappa);
    path_point.set_dkappa(point.dkappa);
    path_points.push_back(path_point);
  }
  return path_points;
}

double relativeTimeAtS(const planning_msgs::TrajectoryPointArray& trajectory, const double query_s)
{
  // 查询当前轨迹在指定 s 处的预计相对时间。工程内已有的 DiscretizedTrajectory::Evaluate
  // 是按 relative_time 查轨迹点，方向相反；这里保留按 s 查找，并复用通用 lerp 插值。
  if (trajectory.points.empty())
  {
    return std::numeric_limits<double>::infinity();
  }

  if (query_s <= trajectory.points.front().s)
  {
    return trajectory.points.front().relative_time;
  }

  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    const auto& prev = trajectory.points[i - 1];
    const auto& cur = trajectory.points[i];
    if (query_s > cur.s)
    {
      continue;
    }

    return lerp(prev.relative_time, prev.s, cur.relative_time, cur.s, query_s);
  }

  return trajectory.points.back().relative_time;
}

bool projectPointToCurrentTrajectory(const planning_msgs::TrajectoryPointArray& trajectory,
                                     const std::vector<PathPoint>& path_points,
                                     const geometry_msgs::Point& point,
                                     double& projected_s,
                                     double& lateral_error)
{
  // 将冲突模块输出的 map 坐标点投影到当前帧轨迹：
  // 1. 返回 projected_s，供当前 planner 重新生成 ego_s_in/ego_s_out/stop_s；
  // 2. 返回 lateral_error，判断当前轨迹是否仍经过原冲突路段；
  // 3. 检查 projected_s 是否落在当前轨迹范围附近，避免把轨迹外延长线上的投影误当有效结果。
  if (trajectory.points.empty() || path_points.size() < 2)
  {
    return false;
  }

  const auto sl = PathMatcher::GetPathFrenetCoordinate(path_points, point.x, point.y);
  projected_s = sl.first;
  lateral_error = std::fabs(sl.second);
  if (!std::isfinite(projected_s) || !std::isfinite(lateral_error))
  {
    return false;
  }

  const double s_min = trajectory.points.front().s;
  const double s_max = trajectory.points.back().s;
  const double s_tolerance = 1.0;
  return projected_s >= s_min - s_tolerance && projected_s <= s_max + s_tolerance;
}

void recomputeTiming(planning_msgs::TrajectoryPointArray& trajectory)
{
  // 修改速度后，原 relative_time/a 已不再匹配当前轨迹速度，需要重新递推一遍。
  if (trajectory.points.empty())
  {
    return;
  }

  trajectory.points.front().relative_time = 0.0;
  trajectory.points.front().a = 0.0;
  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    auto& prev = trajectory.points[i - 1];
    auto& cur = trajectory.points[i];
    const double ds = std::max(0.0, cur.s - prev.s);
    const double avg_v = 0.5 * (std::max(0.0, prev.v) + std::max(0.0, cur.v));
    const double dt = ds > 1e-3 ? ds / std::max(0.1, avg_v) : 0.1;
    cur.relative_time = prev.relative_time + dt;
    cur.a = (cur.v - prev.v) / std::max(0.1, dt);
  }
}

}  // namespace

void ConflictConstraintProcessor::loadParam(ros::NodeHandle& private_nh)
{
  private_nh.param<bool>("enable_conflict_constraint", enabled_, false);
  private_nh.param<double>("conflict_constraint_timeout", constraint_timeout_, 0.5);
  private_nh.param<double>("conflict_timeout_max_speed", timeout_max_speed_, 1.0);
  private_nh.param<double>("conflict_deceleration_limit", deceleration_limit_, 1.5);
  private_nh.param<double>("conflict_stop_margin", stop_margin_, 1.0);
  private_nh.param<double>("conflict_stop_buffer", stop_buffer_, 0.0);
  private_nh.param<double>("conflict_min_smooth_yield_speed", min_smooth_yield_speed_, 0.3);
  private_nh.param<double>("conflict_projection_max_lateral_error", projection_max_lateral_error_, 2.0);
  private_nh.param<double>("conflict_projection_min_s_gap", projection_min_s_gap_, 0.2);
}

void ConflictConstraintProcessor::updateConstraint(const planning_msgs::ConflictConstraint& constraint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_constraint_ = constraint;
  have_constraint_ = true;
}

void ConflictConstraintProcessor::apply(planning_msgs::TrajectoryPointArray& trajectory)
{
  if (!enabled_ || trajectory.points.empty())
  {
    return;
  }

  planning_msgs::ConflictConstraint constraint;
  bool have_constraint = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_constraint_)
    {
      constraint = latest_constraint_;
      have_constraint = true;
    }
  }

  if (!have_constraint)
  {
    return;
  }

  const ros::Time now = ros::Time::now();
  const bool stamp_valid = !constraint.header.stamp.isZero();
  const double age = stamp_valid ? (now - constraint.header.stamp).toSec()
                                 : std::numeric_limits<double>::infinity();
  const bool constraint_timeout = !stamp_valid || age < 0.0 || age > constraint_timeout_;

  bool changed = false;
  bool stop_by_constraint = false;
  bool yield_by_time = false;
  double speed_cap = std::numeric_limits<double>::infinity();
  double yield_speed_cap = std::numeric_limits<double>::infinity();
  double yield_entry_s = std::numeric_limits<double>::infinity();
  double stop_s = std::numeric_limits<double>::infinity();

  const std::vector<PathPoint> path_points = makePathPoints(trajectory);

  if (constraint_timeout)
  {
    // 冲突模块长时间未更新时，不能认为“无冲突”。这里采用保守限速，
    // 让车辆继续保持低速等待下一次有效约束。
    speed_cap = std::max(0.0, timeout_max_speed_);
    changed = true;
    ROS_WARN_THROTTLE(1.0,
                      "conflict_constraint timeout, applying conservative speed cap %.2f m/s",
                      speed_cap);
  }
  else if (constraint.role == planning_msgs::ConflictConstraint::ROLE_YIELD)
  {
    if (!constraint.has_spatial_constraint)
    {
      // 消息中不再携带旧轨迹 s。若没有 map 坐标冲突入口/出口，planner 无法可靠判断
      // 当前轨迹是否仍经过该冲突路段，因此直接丢弃该约束。
      ROS_WARN_THROTTLE(1.0, "conflict yield constraint has no spatial section, skip constraint");
      return;
    }

    double ego_s_in = std::numeric_limits<double>::infinity();
    double ego_s_out = std::numeric_limits<double>::infinity();
    double entry_lateral_error = std::numeric_limits<double>::infinity();
    double exit_lateral_error = std::numeric_limits<double>::infinity();
    const bool entry_projected = projectPointToCurrentTrajectory(
        trajectory, path_points, constraint.ego_entry_point, ego_s_in, entry_lateral_error);
    const bool exit_projected = projectPointToCurrentTrajectory(
        trajectory, path_points, constraint.ego_exit_point, ego_s_out, exit_lateral_error);

    // 投影有效性判断：
    // - 入口/出口都能投到当前轨迹；
    // - 两个点到当前轨迹的横向距离不过大，说明当前轨迹仍经过原冲突路段；
    // - 出口 s 必须在入口 s 之后，且间隔不能退化得过短。
    const bool projection_valid =
        entry_projected &&
        exit_projected &&
        entry_lateral_error <= projection_max_lateral_error_ &&
        exit_lateral_error <= projection_max_lateral_error_ &&
        ego_s_out >= ego_s_in + std::max(0.0, projection_min_s_gap_);

    if (!projection_valid)
    {
      // 当前轨迹与冲突模块上一轮判断使用的轨迹已不一致。
      // 此时没有旧 s 可退回使用，也不应该在错误位置限速，所以直接跳过该约束。
      ROS_WARN_THROTTLE(1.0,
                        "conflict spatial constraint does not match current trajectory, "
                        "skip constraint. entry_projected=%d exit_projected=%d "
                        "entry_l=%.2f exit_l=%.2f projected_s_in=%.2f projected_s_out=%.2f",
                        entry_projected, exit_projected,
                        entry_lateral_error, exit_lateral_error,
                        ego_s_in, ego_s_out);
      return;
    }

    // 使用投影后的 ego_s_in 重新计算本轮轨迹上的停车点和冲突入口。
    // yield_entry_s 用于约束“不能早于 target_entry_time 进入冲突区”。
    stop_s = std::max(0.0, ego_s_in - std::max(0.0, stop_margin_) - std::max(0.0, stop_buffer_));
    yield_entry_s = std::max(stop_s, ego_s_in);

    // 冲突预判模块不再计算建议速度上限，planner 侧只根据当前轨迹剩余距离和
    // target_entry_time 反推本轮速度上限，避免同一件事在两个模块重复计算。
    double smooth_speed_cap = std::numeric_limits<double>::infinity();

    // target_entry_time 是冲突消息生成时的相对目标时间。消息可能已经有 age 秒历史，
    // 因此 planner 当前帧执行时要减去消息年龄，避免用过期时间直接规划。
    const double target_entry_time_from_now =
        std::max(0.0, constraint.target_entry_time - std::max(0.0, age));
    const double current_entry_time = relativeTimeAtS(trajectory, yield_entry_s);
    const bool need_delay =
        std::isfinite(target_entry_time_from_now) &&
        std::isfinite(current_entry_time) &&
        current_entry_time + 1.0e-3 < target_entry_time_from_now;

    if (need_delay)
    {
      const double remain_s = std::max(0.0, yield_entry_s - trajectory.points.front().s);
      if (remain_s > 1.0e-3 && target_entry_time_from_now > 0.1)
      {
        // 轻量平滑让行：用“剩余距离 / 剩余时间”反推进入冲突区前的速度上限。
        // 这样车辆会提前慢下来等快车通过，而不是靠近 stop_s 后再急刹。
        smooth_speed_cap = std::min(smooth_speed_cap, remain_s / target_entry_time_from_now);
      }

      if (std::isfinite(smooth_speed_cap) &&
          smooth_speed_cap > std::max(0.0, min_smooth_yield_speed_))
      {
        // 计算出的让行速度仍可行，采用低速滑行避让。
        yield_speed_cap = smooth_speed_cap;
        yield_by_time = true;
        changed = true;
      }
      else
      {
        // 若为了满足目标进入时间需要极低速度爬行，则改为在 stop_s 前停车等待。
        stop_by_constraint = std::isfinite(stop_s);
        changed = true;
      }
    }
  }
  else
  {
    return;
  }

  const double decel = std::max(0.1, deceleration_limit_);
  for (auto& point : trajectory.points)
  {
    double target_v = std::max(0.0, point.v);
    if (std::isfinite(speed_cap))
    {
      target_v = std::min(target_v, speed_cap);
    }

    if (yield_by_time && point.s < yield_entry_s)
    {
      target_v = std::min(target_v, yield_speed_cap);
    }

    if (stop_by_constraint)
    {
      const double remain_s = stop_s - point.s;
      if (remain_s <= 0.0)
      {
        target_v = 0.0;
      }
      else
      {
        // 停车兜底使用 v^2 = 2as 的制动包络，保证越接近 stop_s 速度越低。
        const double stop_limit_v = std::sqrt(std::max(0.0, 2.0 * decel * remain_s));
        target_v = std::min(target_v, stop_limit_v);
      }
    }

    if (std::fabs(point.v - target_v) > 1e-3)
    {
      point.v = target_v;
      changed = true;
    }
  }

  if (changed)
  {
    recomputeTiming(trajectory);
  }
}

}  // namespace conflict_prediction_resolution
