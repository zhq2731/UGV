#include "conflict_prediction_resolution/conflict_constraint_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include "common/pnc_point.h"
#include "common/points_convert.h"
#include "math/linear_interpolation.h"
#include "math/path_matcher.h"
#include "math/vec2d.h"
#include "trajectory/discretized_trajectory.h"

namespace conflict_prediction_resolution
{
namespace
{

using ugv::common::math::PathMatcher;
using ugv::common::math::Vec2d;
using ugv::common::math::lerp;
using ugv::planning::DiscretizedTrajectory;
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

size_t findStitchingEndIndex(const planning_msgs::TrajectoryPointArray& trajectory,
                             const geometry_msgs::Point& planning_start_point,
                             const double max_match_distance)
{
  // planning_start_point 在 RefLinePlanner 中等于 traj_stitcher.back()。
  // 这里复用工程已有的 DiscretizedTrajectory::QueryNearestPointWithBuffer，
  // 把它匹配回 TrajectoryPointArray，从而恢复“拼接段结束索引”。
  // 若匹配距离过大，说明该轨迹可能不是 RefLinePlanner 拼接轨迹，退化为从第 0 点开始处理。
  if (trajectory.points.empty())
  {
    return 0;
  }

  planning_msgs::TrajectoryPointArray trajectory_copy = trajectory;
  DiscretizedTrajectory discretized_trajectory;
  trajMsg2DiscretTraj(trajectory_copy, discretized_trajectory);

  const Vec2d start_position(planning_start_point.x, planning_start_point.y);
  const size_t best_index = discretized_trajectory.QueryNearestPointWithBuffer(start_position, 1.0e-6);
  const auto& matched_point = trajectory.points[best_index];
  const double matched_distance = std::hypot(matched_point.x - planning_start_point.x,
                                             matched_point.y - planning_start_point.y);

  if (!std::isfinite(matched_distance) || matched_distance > std::max(0.0, max_match_distance))
  {
    ROS_WARN_THROTTLE(1.0,
                      "planning_start_point does not match current trajectory, "
                      "treat trajectory as non-stitched. nearest_distance=%.2f",
                      matched_distance);
    return 0;
  }

  return best_index;
}

double estimateTimeDelta(const planning_msgs::TrajectoryPoint& prev,
                         const planning_msgs::TrajectoryPoint& cur,
                         const double old_prev_time,
                         const double old_cur_time)
{
  // 优先复用速度规划已经给出的相邻点时间差；若时间差异常，再根据空间距离和平均速度估算。
  const double old_dt = old_cur_time - old_prev_time;
  if (std::isfinite(old_dt) && old_dt > 1.0e-4)
  {
    return old_dt;
  }

  const double ds = std::max(0.0, cur.s - prev.s);
  const double avg_v = 0.5 * (std::max(0.0, prev.v) + std::max(0.0, cur.v));
  return ds > 1.0e-3 ? ds / std::max(0.1, avg_v) : 0.0;
}

void normalizeStitchedPrefixTiming(planning_msgs::TrajectoryPointArray& trajectory,
                                   const size_t stitching_end_index,
                                   const double planning_cycle_time)
{
  // 当前工程的 RefLinePlanner 中，planning_start_point 等于 traj_stitcher.back()。
  // 为了让冲突速度粗解的时间零点更清晰，这里只规范拼接前缀的 relative_time：
  // 1. 拼接段末点，也就是本轮路径规划起点，固定为单次规划循环时间；
  // 2. 末点之前的拼接点按原相邻时间间隔向前递推；
  // 3. 不在这里修改非拼接后缀，后缀由规则速度修正和固定时间重采样负责。
  if (trajectory.points.empty())
  {
    return;
  }

  const size_t end_index = std::min(stitching_end_index, trajectory.points.size() - 1);
  const double normalized_start_time = std::max(0.0, planning_cycle_time);

  std::vector<double> old_relative_times;
  old_relative_times.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points)
  {
    old_relative_times.push_back(point.relative_time);
  }

  trajectory.points[end_index].relative_time = normalized_start_time;

  for (size_t i = end_index; i > 0; --i)
  {
    const double dt = estimateTimeDelta(trajectory.points[i - 1],
                                        trajectory.points[i],
                                        old_relative_times[i - 1],
                                        old_relative_times[i]);
    trajectory.points[i - 1].relative_time =
        std::max(0.0, trajectory.points[i].relative_time - dt);
  }

  for (size_t i = 1; i <= end_index; ++i)
  {
    const double dt = trajectory.points[i].relative_time - trajectory.points[i - 1].relative_time;
    if (dt > 1.0e-4)
    {
      trajectory.points[i].a = (trajectory.points[i].v - trajectory.points[i - 1].v) / dt;
    }
  }
}

void recomputeTimingFrom(planning_msgs::TrajectoryPointArray& trajectory, const size_t fixed_prefix_end_index)
{
  // 拼接前缀 [0, fixed_prefix_end_index] 已经由上游规划/控制承接，冲突速度规则不改它。
  // 因此时间递推从拼接末点开始，只更新后缀的 relative_time 和 a。
  if (trajectory.points.empty() || fixed_prefix_end_index + 1 >= trajectory.points.size())
  {
    return;
  }

  for (size_t i = fixed_prefix_end_index + 1; i < trajectory.points.size(); ++i)
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

planning_msgs::TrajectoryPoint interpolatePointByTime(
    const planning_msgs::TrajectoryPointArray& trajectory,
    const double query_time,
    size_t& search_index)
{
  // 在已递增的 relative_time 序列上插值，生成固定时间间隔粗轨迹点。
  // 插值只用于形成后续定时间 QP 的粗解，因此对 x/y/s/v/a/theta/kappa 做轻量线性插值。
  if (trajectory.points.empty())
  {
    return planning_msgs::TrajectoryPoint();
  }

  while (search_index + 1 < trajectory.points.size() &&
         trajectory.points[search_index + 1].relative_time < query_time)
  {
    ++search_index;
  }

  if (search_index + 1 >= trajectory.points.size())
  {
    return trajectory.points.back();
  }

  const auto& prev = trajectory.points[search_index];
  const auto& next = trajectory.points[search_index + 1];
  const double dt = next.relative_time - prev.relative_time;
  const double ratio = dt > 1.0e-6 ? (query_time - prev.relative_time) / dt : 0.0;
  const double clamped_ratio = std::max(0.0, std::min(1.0, ratio));

  planning_msgs::TrajectoryPoint out;
  out.relative_time = query_time;
  out.x = lerp(prev.x, 0.0, next.x, 1.0, clamped_ratio);
  out.y = lerp(prev.y, 0.0, next.y, 1.0, clamped_ratio);
  out.z = lerp(prev.z, 0.0, next.z, 1.0, clamped_ratio);
  out.theta = lerp(prev.theta, 0.0, next.theta, 1.0, clamped_ratio);
  out.s = lerp(prev.s, 0.0, next.s, 1.0, clamped_ratio);
  out.kappa = lerp(prev.kappa, 0.0, next.kappa, 1.0, clamped_ratio);
  out.dkappa = lerp(prev.dkappa, 0.0, next.dkappa, 1.0, clamped_ratio);
  out.v = lerp(prev.v, 0.0, next.v, 1.0, clamped_ratio);
  out.a = lerp(prev.a, 0.0, next.a, 1.0, clamped_ratio);
  return out;
}

void resampleNonStitchedSuffixToFixedTime(planning_msgs::TrajectoryPointArray& trajectory,
                                          const size_t fixed_prefix_end_index,
                                          const double fixed_dt)
{
  // 将非拼接段转为固定时间间隔粗轨迹：
  // - 拼接前缀保持原样，避免破坏上一帧承接段；
  // - 后缀按固定 dt 在 relative_time 上插值；
  // - 若后缀太短，不强行重采样，避免生成空轨迹。
  if (trajectory.points.empty() || fixed_prefix_end_index + 1 >= trajectory.points.size())
  {
    return;
  }

  const double dt = std::max(0.02, fixed_dt);
  const double start_time = trajectory.points[fixed_prefix_end_index].relative_time;
  const double end_time = trajectory.points.back().relative_time;
  if (end_time <= start_time + dt)
  {
    return;
  }

  planning_msgs::TrajectoryPointArray resampled;
  resampled.header = trajectory.header;
  resampled.is_forward_shift = trajectory.is_forward_shift;
  resampled.task_area = trajectory.task_area;
  resampled.close_to_end = trajectory.close_to_end;
  resampled.shape = trajectory.shape;
  resampled.type = trajectory.type;

  resampled.points.insert(resampled.points.end(),
                          trajectory.points.begin(),
                          trajectory.points.begin() + fixed_prefix_end_index + 1);

  size_t search_index = fixed_prefix_end_index;
  for (double t = start_time + dt; t <= end_time + 1.0e-6; t += dt)
  {
    resampled.points.push_back(interpolatePointByTime(trajectory, t, search_index));
  }

  if (resampled.points.size() > fixed_prefix_end_index + 1)
  {
    trajectory = resampled;
  }
}

}  // namespace

void ConflictConstraintProcessor::loadParam(ros::NodeHandle& private_nh)
{
  private_nh.param<bool>("enable_conflict_constraint", enabled_, false);
  private_nh.param<double>("conflict_constraint_timeout", constraint_timeout_, 2.1);
  private_nh.param<double>("conflict_timeout_max_speed", timeout_max_speed_, 1.0);
  private_nh.param<double>("conflict_deceleration_limit", deceleration_limit_, 1.5);
  private_nh.param<double>("conflict_stop_margin", stop_margin_, 1.0);
  private_nh.param<double>("conflict_stop_buffer", stop_buffer_, 0.0);
  private_nh.param<double>("conflict_min_smooth_yield_speed", min_smooth_yield_speed_, 0.3);
  private_nh.param<double>("conflict_projection_max_lateral_error", projection_max_lateral_error_, 2.0);
  private_nh.param<double>("conflict_projection_min_s_gap", projection_min_s_gap_, 0.2);
  private_nh.param<double>("conflict_stitching_start_match_max_distance",
                           stitching_start_match_max_distance_, 1.0);
  private_nh.param<double>("conflict_fixed_time_coarse_dt", fixed_time_coarse_dt_, 0.1);
  private_nh.param<double>("conflict_planning_cycle_time", planning_cycle_time_, 0.1);
}

void ConflictConstraintProcessor::updateConstraint(const planning_msgs::ConflictConstraint& constraint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_constraint_ = constraint;
  have_constraint_ = true;
}

void ConflictConstraintProcessor::apply(planning_msgs::TrajectoryPointArray& trajectory,
                                        const geometry_msgs::Point& planning_start_point)
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
  const bool decision_valid = !constraint.decision_source.empty();

  bool changed = false;
  bool stop_by_constraint = false;
  bool yield_by_time = false;
  double speed_cap = std::numeric_limits<double>::infinity();
  double yield_speed_cap = std::numeric_limits<double>::infinity();
  double yield_entry_s = std::numeric_limits<double>::infinity();
  double stop_s = std::numeric_limits<double>::infinity();

  if (constraint_timeout)
  {
    // 决策消息超时后，不再使用旧的让行关系、冲突入口和目标进入时间。
    // 但出于安全考虑，需要立刻进入保守降速，只对非拼接段施加低速上限。
    speed_cap = std::max(0.0, timeout_max_speed_);
    changed = true;
    ROS_WARN_THROTTLE(1.0,
                      "conflict_constraint timeout, applying conservative speed cap %.2f m/s",
                      speed_cap);
  }
  else if (constraint.role == planning_msgs::ConflictConstraint::ROLE_YIELD)
  {
    if (!decision_valid)
    {
      // role=YIELD 但没有 decision_source，说明冲突预判没有输出完整决策溯源。
      // 此时不执行速度消解，避免 planner 根据半成品消息自行生成让行速度。
      ROS_WARN_THROTTLE(1.0, "conflict yield constraint has no decision source, skip constraint");
      return;
    }

    if (!constraint.has_spatial_constraint)
    {
      // 消息中不再携带旧轨迹 s。若没有 map 坐标冲突入口/出口，planner 无法可靠判断
      // 当前轨迹是否仍经过该冲突路段，因此直接丢弃该约束。
      ROS_WARN_THROTTLE(1.0, "conflict yield constraint has no spatial section, skip constraint");
      return;
    }

    const std::vector<PathPoint> path_points = makePathPoints(trajectory);

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
  }
  else
  {
    // ROLE_NONE 表示冲突预判未作出让行决策；ROLE_PROCEED 表示本车被授权先行。
    // 两者都不需要冲突消解速度规划，保持原规划轨迹。
    return;
  }

  // 只有在确实需要执行冲突速度处理时，才识别拼接段并规范拼接前缀时间。
  // 这样无约束、直行、空间约束失效等正常场景不会被冲突模块额外改写轨迹。
  const size_t stitching_end_index = findStitchingEndIndex(
      trajectory, planning_start_point, stitching_start_match_max_distance_);
  planning_msgs::TrajectoryPointArray normalized_trajectory = trajectory;
  normalizeStitchedPrefixTiming(normalized_trajectory, stitching_end_index, planning_cycle_time_);
  const size_t mutable_start_index =
      std::min(stitching_end_index + 1, normalized_trajectory.points.size() - 1);

  if (std::isfinite(yield_entry_s))
  {
    const double target_entry_time_from_now =
        std::max(0.0, constraint.target_entry_time - std::max(0.0, age));
    const double current_entry_time = relativeTimeAtS(normalized_trajectory, yield_entry_s);
    const bool need_delay =
        std::isfinite(target_entry_time_from_now) &&
        std::isfinite(current_entry_time) &&
        current_entry_time + 1.0e-3 < target_entry_time_from_now;

    if (need_delay)
    {
      const double remain_s =
          std::max(0.0, yield_entry_s - normalized_trajectory.points[stitching_end_index].s);
      if (remain_s > 1.0e-3 && target_entry_time_from_now > 0.1)
      {
        // 冲突预判模块不再计算建议速度上限，planner 侧根据当前轨迹剩余距离和
        // target_entry_time 反推本轮速度上限，避免同一件事在两个模块重复计算。
        // 轻量平滑让行：用“剩余距离 / 剩余时间”反推进入冲突区前的速度上限。
        // 这样车辆会提前慢下来等快车通过，而不是靠近 stop_s 后再急刹。
        const double smooth_speed_cap = remain_s / target_entry_time_from_now;
        if (smooth_speed_cap > std::max(0.0, min_smooth_yield_speed_))
        {
          yield_speed_cap = smooth_speed_cap;
          yield_by_time = true;
          stop_by_constraint = false;
          changed = true;
        }
      }

      if (!yield_by_time)
      {
        // 若为了满足目标进入时间需要极低速度爬行，则改为在 stop_s 前停车等待。
        stop_by_constraint = std::isfinite(stop_s);
        changed = changed || stop_by_constraint;
      }
    }
  }

  if (!changed)
  {
    return;
  }

  trajectory = normalized_trajectory;

  const double decel = std::max(0.1, deceleration_limit_);
  for (size_t i = mutable_start_index; i < trajectory.points.size(); ++i)
  {
    auto& point = trajectory.points[i];
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
    recomputeTimingFrom(trajectory, stitching_end_index);
    resampleNonStitchedSuffixToFixedTime(trajectory, stitching_end_index, fixed_time_coarse_dt_);
  }
}

}  // namespace conflict_prediction_resolution
