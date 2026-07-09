#include "conflict_prediction_resolution/conflict_constraint_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <boost/bind.hpp>
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

// 以下常量属于控制器输入保护和拼接稳定性的内部阈值，不作为外部参数开放：
// 1. 它们不是冲突消解策略本身的调参项；
// 2. 数值只用于避免重复几何点、过短停车视野等工程异常；
// 3. 频繁改动反而会让实验参数含义变乱，因此固定在代码中并在这里集中说明。
constexpr double kMinStopTimeHorizon = 3.0;                // s，停车点至少保留的时间视野。
constexpr double kFinalTimeDtFloor = 0.02;                 // s，最终输出轨迹最小时间间隔保护。
constexpr double kStoppedTailLength = 6.0;                 // m，停车后保留给控制器横向跟踪的零速几何尾迹。
constexpr double kStoppedTailTimeStep = 0.1;               // s，零速几何尾迹相邻点的固定时间间隔。

std::string vehicleTopic(const std::string& vehicle_id, const std::string& suffix)
{
  if (vehicle_id.empty())
  {
    return {};
  }
  if (!vehicle_id.empty() && vehicle_id.front() == '/')
  {
    return vehicle_id + "/" + suffix;
  }
  return "/" + vehicle_id + "/" + suffix;
}

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

const char* strategyName(const uint8_t strategy)
{
  switch (strategy)
  {
    case planning_msgs::ConflictConstraint::STRATEGY_PROCEED:
      return "PROCEED";
    case planning_msgs::ConflictConstraint::STRATEGY_STOP_AND_WAIT:
      return "STOP_AND_WAIT";
    case planning_msgs::ConflictConstraint::STRATEGY_SLOW_DOWN:
      return "SLOW_DOWN";
    case planning_msgs::ConflictConstraint::STRATEGY_FOLLOW:
      return "FOLLOW";
    case planning_msgs::ConflictConstraint::STRATEGY_EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case planning_msgs::ConflictConstraint::STRATEGY_NONE:
    default:
      return "NONE";
  }
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

struct ProjectedConflictSection
{
  bool entry_projected = false;
  bool exit_projected = false;
  bool valid = false;
  double s_in = std::numeric_limits<double>::infinity();
  double s_out = std::numeric_limits<double>::infinity();
  double entry_lateral_error = std::numeric_limits<double>::infinity();
  double exit_lateral_error = std::numeric_limits<double>::infinity();
};

ProjectedConflictSection projectConflictSection(
    const planning_msgs::TrajectoryPointArray& trajectory,
    const std::vector<PathPoint>& path_points,
    const planning_msgs::ConflictConstraint& constraint,
    const double max_lateral_error,
    const double min_s_gap)
{
  // 将一条 ConflictConstraint 中的入口/出口 map 点投影到当前轨迹，并集中完成
  // 有效性判断。速度层只执行冲突判定层当前给出的空间约束，
  // 不在这里额外合并历史冲突区，避免判断层和速度层出现两套释放逻辑。
  ProjectedConflictSection section;
  section.entry_projected = projectPointToCurrentTrajectory(
      trajectory, path_points, constraint.ego_entry_point,
      section.s_in, section.entry_lateral_error);
  section.exit_projected = projectPointToCurrentTrajectory(
      trajectory, path_points, constraint.ego_exit_point,
      section.s_out, section.exit_lateral_error);

  section.valid =
      section.entry_projected &&
      section.exit_projected &&
      section.entry_lateral_error <= max_lateral_error &&
      section.exit_lateral_error <= max_lateral_error &&
      section.s_out >= section.s_in + std::max(0.0, min_s_gap);
  return section;
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
  // 这里参考 Apollo 的拼接时间表达原则：
  // 1. 拼接段末点，也就是本轮路径规划起点，表示“当前时刻向前一个规划周期”的点，
  //    因此固定为单次规划循环时间；
  // 2. 末点之前的拼接点按原相邻时间间隔向前递推，可以小于规划周期，也允许为负；
  // 3. 不在这里修改非拼接后缀，后缀由规则速度修正和固定时间重采样负责。
  //
  // 注意：前缀点 relative_time 不强制夹到 0 以上，这样才符合 Apollo 的拼接表达。
  // Apollo 中已经落在当前时刻之前的拼接点会表现为负 relative_time。
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
    trajectory.points[i - 1].relative_time = trajectory.points[i].relative_time - dt;
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

void normalizeNonStitchedSuffixS(planning_msgs::TrajectoryPointArray& trajectory,
                                 const size_t stitching_end_index)
{
  // 当前 path planner 的拼接轨迹里，拼接前缀来自上一帧轨迹，新规划后缀来自本帧路径。
  // 这里参考 Apollo 的拼接 s 表达原则：
  // 1. 以拼接段末点作为 s=0；
  // 2. 拼接前缀点用 old_s - stitching_end_old_s 表达，因此通常为负；
  // 3. 非拼接后缀从拼接末点开始，按 x/y 弧长继续向前累加，因此为正。
  //
  // 这样冲突消解内部看到的纵向坐标和 Apollo 一样，以“规划连接点”为零点，
  // 可以避免拼接前缀 s 已经走到 2~3m，而新规划后缀又从 0.5m 开始造成 s 回退。
  // 注意：该归一化只作用于冲突模块内部传递的 trajectory，不改全局路径规划器的拼接实现。
  if (trajectory.points.empty())
  {
    return;
  }

  const size_t end_index = std::min(stitching_end_index, trajectory.points.size() - 1);
  const double zero_s = trajectory.points[end_index].s;

  for (size_t i = 0; i <= end_index; ++i)
  {
    trajectory.points[i].s -= zero_s;
  }

  if (end_index + 1 >= trajectory.points.size())
  {
    return;
  }

  for (size_t i = end_index + 1; i < trajectory.points.size(); ++i)
  {
    const auto& prev = trajectory.points[i - 1];
    auto& cur = trajectory.points[i];
    const double ds_xy = std::hypot(cur.x - prev.x, cur.y - prev.y);
    cur.s = prev.s + std::max(0.0, ds_xy);
  }
}

struct PreparedConflictTrajectory
{
  // original 保留 path planner 原始输出。若冲突速度处理最终失败，需要用它恢复现场。
  planning_msgs::TrajectoryPointArray original;

  // normalized 是冲突消解内部使用的轨迹：
  // - 拼接前缀的时间按 Apollo 风格递推；
  // - 拼接末点作为 s=0；
  // - 非拼接后缀重新按几何弧长累计 s。
  planning_msgs::TrajectoryPointArray normalized;

  // stitching_end_index 是拼接前缀的最后一个点，也是本轮规划真正可以修改的起点前一位。
  size_t stitching_end_index = 0;

  // mutable_start_index 是冲突速度规划允许修改的第一个点，避免破坏拼接前缀。
  size_t mutable_start_index = 0;
};

PreparedConflictTrajectory prepareConflictTrajectory(
    const planning_msgs::TrajectoryPointArray& trajectory,
    const geometry_msgs::Point& planning_start_point,
    const double stitching_start_match_max_distance,
    const double planning_cycle_time)
{
  // 将 apply() 中“识别拼接段 + 规范内部 s/t”的固定流程收口到这里。
  // 这样主流程只需要关心“拿到一条可用于冲突消解的 normalized 轨迹”，
  // 不再把拼接细节散落在决策逻辑中。
  PreparedConflictTrajectory prepared;
  prepared.original = trajectory;
  prepared.normalized = trajectory;

  if (trajectory.points.empty())
  {
    return prepared;
  }

  prepared.stitching_end_index = findStitchingEndIndex(
      trajectory, planning_start_point, stitching_start_match_max_distance);
  normalizeStitchedPrefixTiming(prepared.normalized,
                                prepared.stitching_end_index,
                                planning_cycle_time);
  normalizeNonStitchedSuffixS(prepared.normalized, prepared.stitching_end_index);
  prepared.mutable_start_index =
      std::min(prepared.stitching_end_index + 1, prepared.normalized.points.size() - 1);
  return prepared;
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
    const bool stopped_tail = prev.v < 1.0e-3 && cur.v < 1.0e-3 && ds > 1.0e-3;
    // 停车点后的几何尾迹只用于给横向 MPC 提供足够长的前向路径，不表示车辆继续运动。
    // 若按极低“虚拟爬行速度”展开时间，6m 尾迹会生成数百秒时间轴，
    // 不仅破坏下一轮拼接，还可能使控制器追加的末点发生时间倒退。
    // 现在统一使用短固定时间步长：几何仍严格向前、速度保持 0、时间严格递增，
    // 同时整条停车轨迹保持在正常的秒级时间范围内。
    const double dt = stopped_tail
                          ? kStoppedTailTimeStep
                          : (ds > 1e-3 ? ds / std::max(0.1, avg_v) : 0.1);
    cur.relative_time = prev.relative_time + dt;
    cur.a = (cur.v - prev.v) / std::max(0.1, dt);
  }
}

bool truncateTrajectoryAtStopS(planning_msgs::TrajectoryPointArray& trajectory,
                               size_t fixed_prefix_end_index,
                               double stop_s);

void enforceStrictlyIncreasingRelativeTime(planning_msgs::TrajectoryPointArray& trajectory,
                                           const double min_dt)
{
  // 控制器按 relative_time 做轨迹插值。若相邻点时间相等或倒退，
  // lower_bound/插值比例都可能出现异常，因此在冲突模块最终输出前做一次兜底修正。
  if (trajectory.points.empty())
  {
    return;
  }

  const double dt_floor = std::max(1.0e-3, min_dt);
  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    const double min_time = trajectory.points[i - 1].relative_time + dt_floor;
    if (!std::isfinite(trajectory.points[i].relative_time) ||
        trajectory.points[i].relative_time < min_time)
    {
      trajectory.points[i].relative_time = min_time;
    }
  }

  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    const double dt = std::max(dt_floor,
                               trajectory.points[i].relative_time -
                                   trajectory.points[i - 1].relative_time);
    trajectory.points[i].a = (trajectory.points[i].v - trajectory.points[i - 1].v) / dt;
  }
}

bool validateTrajectoryForController(const planning_msgs::TrajectoryPointArray& trajectory,
                                     const size_t strict_check_start_index,
                                     std::string* reject_reason = nullptr)
{
  // 冲突速度处理后的最终轨迹要交给 trajectory_follower。
  // 这里做轻量但关键的输入保护：
  // 1. 全轨迹检查非有限数和 relative_time 递增，因为这些一定会破坏插值；
  // 2. 只从 strict_check_start_index 开始严格检查 s 回退/重复几何。
  //
  // 原因是拼接前缀来自上一帧规划，里面偶尔会存在很小的 s 抖动，
  // 但它不是冲突模块本轮新增的问题；如果把前缀也当成冲突输出错误，
  // 会触发“冲突消解失败 -> 恢复原始轨迹 -> 下一帧继续失败”的循环。
  // 真正需要拦截的是冲突模块改写后的非拼接后缀，以及拼接末点到后缀的边界。
  if (trajectory.points.size() < 2)
  {
    if (reject_reason)
    {
      *reject_reason = "too_few_points";
    }
    return false;
  }

  const size_t check_start =
      std::min(strict_check_start_index, trajectory.points.size() - 1);
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    const auto& point = trajectory.points[i];
    const bool finite =
        std::isfinite(point.x) &&
        std::isfinite(point.y) &&
        std::isfinite(point.z) &&
        std::isfinite(point.theta) &&
        std::isfinite(point.s) &&
        std::isfinite(point.v) &&
        std::isfinite(point.a) &&
        std::isfinite(point.relative_time);
    if (!finite || point.v < -1.0e-3)
    {
      if (reject_reason)
      {
        *reject_reason = "non_finite_or_negative_velocity index=" + std::to_string(i);
      }
      return false;
    }

    if (i == 0)
    {
      continue;
    }

    const auto& prev = trajectory.points[i - 1];
    if (point.relative_time <= prev.relative_time + 1.0e-4)
    {
      if (reject_reason)
      {
        *reject_reason = "non_increasing_time index=" + std::to_string(i) +
                         " prev_t=" + std::to_string(prev.relative_time) +
                         " cur_t=" + std::to_string(point.relative_time);
      }
      return false;
    }

    if (i < check_start)
    {
      continue;
    }

    if (point.s < prev.s - 1.0e-4)
    {
      if (reject_reason)
      {
        *reject_reason = "s_regression index=" + std::to_string(i) +
                         " prev_s=" + std::to_string(prev.s) +
                         " cur_s=" + std::to_string(point.s);
      }
      return false;
    }

    const double ds = point.s - prev.s;
    const double dxy = std::hypot(point.x - prev.x, point.y - prev.y);
    if (ds <= 1.0e-4 && dxy <= 1.0e-4)
    {
      // MPC 横向控制会按几何弧长重采样。相邻重复点会让累计弧长出现相等值，
      // 进而触发 “base_keys is not sorted” 异常，所以这里必须严格拦住。
      if (reject_reason)
      {
        *reject_reason = "duplicate_geometry index=" + std::to_string(i) +
                         " ds=" + std::to_string(ds) +
                         " dxy=" + std::to_string(dxy) +
                         " prev_v=" + std::to_string(prev.v) +
                         " cur_v=" + std::to_string(point.v);
      }
      return false;
    }
  }

  return true;
}

planning_msgs::TrajectoryPoint interpolatePointByS(
    const planning_msgs::TrajectoryPointArray& trajectory,
    const size_t begin_index,
    const double query_s)
{
  // 在当前轨迹上按 s 插值得到停车点。
  // 规则停车时不应该继续保留 stop_s 之后的路径点，否则速度为 0 但几何仍向前，
  // 后续固定时间重采样/QP 很容易生成“低速平台”或重复点。
  if (trajectory.points.empty())
  {
    return planning_msgs::TrajectoryPoint();
  }

  const size_t begin = std::min(begin_index, trajectory.points.size() - 1);
  if (query_s <= trajectory.points[begin].s)
  {
    return trajectory.points[begin];
  }

  for (size_t i = begin + 1; i < trajectory.points.size(); ++i)
  {
    const auto& prev = trajectory.points[i - 1];
    const auto& next = trajectory.points[i];
    if (query_s > next.s)
    {
      continue;
    }

    const double ds = next.s - prev.s;
    const double ratio = ds > 1.0e-6 ? std::max(0.0, std::min(1.0, (query_s - prev.s) / ds))
                                     : 0.0;
    planning_msgs::TrajectoryPoint out = prev;
    out.relative_time = lerp(prev.relative_time, 0.0, next.relative_time, 1.0, ratio);
    out.x = lerp(prev.x, 0.0, next.x, 1.0, ratio);
    out.y = lerp(prev.y, 0.0, next.y, 1.0, ratio);
    out.z = lerp(prev.z, 0.0, next.z, 1.0, ratio);
    out.theta = lerp(prev.theta, 0.0, next.theta, 1.0, ratio);
    out.s = query_s;
    out.kappa = lerp(prev.kappa, 0.0, next.kappa, 1.0, ratio);
    out.dkappa = lerp(prev.dkappa, 0.0, next.dkappa, 1.0, ratio);
    out.v = 0.0;
    out.a = 0.0;
    return out;
  }

  auto out = trajectory.points.back();
  out.v = 0.0;
  out.a = 0.0;
  return out;
}

bool truncateTrajectoryAtStopS(planning_msgs::TrajectoryPointArray& trajectory,
                               const size_t fixed_prefix_end_index,
                               const double stop_s)
{
  // 停车等待不能在停车点后复制一串相同位置点，因为 trajectory_follower 会按几何弧长插值，
  // 重复几何点会让弧长 key 不严格递增。
  //
  // 但也不能只保留“当前位置 + 停车点”两个点：本工程 MPC 横向控制需要足够长的几何路径
  // 做预测，过短轨迹会让车辆看起来脱离实际轨迹。因此这里采用折中表达：
  //   1. 在 stop_s 处插入一个零速停车点；
  //   2. stop_s 之后沿原路径保留约 kStoppedTailLength 的几何尾迹；
  //   3. 尾迹点全部置零速，表示车辆到停车点后等待，不允许继续驶入冲突区。
  if (!std::isfinite(stop_s) || trajectory.points.empty() ||
      fixed_prefix_end_index + 1 >= trajectory.points.size())
  {
    return false;
  }

  const size_t begin = std::min(fixed_prefix_end_index + 1, trajectory.points.size() - 1);
  if (stop_s >= trajectory.points.back().s)
  {
    return false;
  }

  size_t stop_index = begin;
  while (stop_index < trajectory.points.size() && trajectory.points[stop_index].s < stop_s)
  {
    ++stop_index;
  }

  if (stop_index >= trajectory.points.size())
  {
    return false;
  }

  const auto original_points = trajectory.points;
  auto append_stopped_tail = [&](const size_t first_tail_index) {
    const double tail_end_s = stop_s + kStoppedTailLength;
    for (size_t i = first_tail_index; i < original_points.size(); ++i)
    {
      if (original_points[i].s > tail_end_s)
      {
        break;
      }

      const auto& previous = trajectory.points.back();
      const double ds = original_points[i].s - previous.s;
      const double dxy = std::hypot(original_points[i].x - previous.x,
                                    original_points[i].y - previous.y);
      if (ds <= 1.0e-4 && dxy <= 1.0e-4)
      {
        continue;
      }

      auto tail_point = original_points[i];
      tail_point.v = 0.0;
      tail_point.a = 0.0;
      trajectory.points.push_back(tail_point);
    }
  };

  planning_msgs::TrajectoryPoint stop_point;
  if (stop_index == begin && stop_s <= trajectory.points[fixed_prefix_end_index].s + 1.0e-4)
  {
    // 停车点已经落在拼接末点之前或几乎重合，说明本轮应立即停在拼接末点附近。
    // 控制器仍需要一个向前的几何目标点，因此保留第一个非拼接点作为零速停止目标，
    // 而不是只剩拼接前缀或复制同一几何点。
    trajectory.points.resize(begin + 1);
    trajectory.points.back().v = 0.0;
    trajectory.points.back().a = 0.0;
    append_stopped_tail(begin + 1);
    return true;
  }

  stop_point = interpolatePointByS(trajectory, fixed_prefix_end_index, stop_s);
  trajectory.points.resize(stop_index);
  trajectory.points.push_back(stop_point);
  append_stopped_tail(stop_index);
  return true;
}

void extendStopPointTimeHorizon(planning_msgs::TrajectoryPointArray& trajectory,
                                const size_t fixed_prefix_end_index,
                                const double min_stop_horizon)
{
  // 停车让行时轨迹会被截断到 stop_s。若 stop_s 离拼接末点很近，
  // 输出轨迹的最后 relative_time 可能只有 0.1~0.2s，下一轮规划就会认为
  // “current time beyond the previous trajectory's last time”，从而频繁重规划。
  //
  // 这里不复制停车点，也不追加重复几何点；只把最后一个停车点的时间戳向后拉长。
  // 这样控制器仍只看到一个合法的停车几何目标，同时 planner 有足够的时间视野
  // 用于下一帧轨迹拼接。
  if (trajectory.points.empty() || fixed_prefix_end_index + 1 >= trajectory.points.size())
  {
    return;
  }

  const double horizon = std::max(0.0, min_stop_horizon);
  if (horizon <= 1.0e-3)
  {
    return;
  }

  const double start_time = trajectory.points[fixed_prefix_end_index].relative_time;
  const double start_s = trajectory.points[fixed_prefix_end_index].s;
  const double start_v = std::max(0.0, trajectory.points[fixed_prefix_end_index].v);
  auto& stop_point = trajectory.points.back();

  if (trajectory.points.size() <= fixed_prefix_end_index + 2 && start_v > 0.2)
  {
    // 近距离停车时，轨迹经常只剩“拼接末点 + 停车点”两个点。
    // 这种情况下不能为了给 planner 留 3s 拼接视野，直接把停车点时间拉长：
    // 例如 0.2m 后停车却给 3s，会让控制器看到“很短几何距离 + 较高初速度 + 很长时间”，
    // 速度、时间、空间三者不一致，车辆容易继续按初速度滑行。
    //
    // 因此两点停车轨迹采用物理一致的刹停时间：
    //   匀减速到 0 时，平均速度约为 v0/2，所以 t_stop = 2 * ds / v0。
    // 这里仍保留一个很小的下限，避免时间戳过密；但不再强行扩展到 min_stop_horizon。
    const double stop_distance = std::max(0.0, stop_point.s - start_s);
    const double physical_stop_time = 2.0 * stop_distance / std::max(0.1, start_v);
    stop_point.relative_time =
        std::max(stop_point.relative_time, start_time + std::max(0.1, physical_stop_time));
  }
  else
  {
    // 已经接近静止，或停车轨迹还有多个前向几何点时，拉长最后停车点时间是安全的：
    // 它表达“到停车点后等待”，同时不会形成重复几何点序列。
    stop_point.relative_time = std::max(stop_point.relative_time, start_time + horizon);
  }
  stop_point.v = 0.0;
  stop_point.a = 0.0;
}

struct VelocityConstraintAction
{
  // 通用速度上限。主要用于决策超时后的保守限速。
  double speed_cap = std::numeric_limits<double>::infinity();

  // 停车让行时使用：stop_s 是“冲突入口前阈值停车线”。
  // 速度层沿当前轨迹逐渐降低速度上限，确保车辆在 stop_s 前刹停。
  bool stop_by_constraint = false;
  double stop_s = std::numeric_limits<double>::infinity();
};

bool applyVelocityConstraintAction(planning_msgs::TrajectoryPointArray& trajectory,
                                   const size_t mutable_start_index,
                                   const VelocityConstraintAction& action,
                                   const double deceleration_limit)
{
  // 将“冲突消解决策”真正写回每个轨迹点速度：
  // - 拼接前缀不改，从 mutable_start_index 开始处理；
  // - 超时保护只夹 v 上限；
  // - 停车让行以 stop_s 为目标，先从拼接承接速度线性降到 0，再叠加
  //   v^2 <= 2*a*remain_s 的物理刹停上限。
  // - 停车分支递推上一点限制后的速度，避免当前轮 planner 把上一帧已经压低的速度抬高。
  //
  // 函数只负责改 v，不负责截断轨迹和重算时间；这些动作仍放在 apply() 后半段统一处理。
  bool velocity_changed = false;
  const double decel = std::max(0.1, deceleration_limit);
  double previous_limited_v = 0.0;
  double stop_start_s = std::numeric_limits<double>::infinity();
  double stop_start_v = 0.0;
  double stop_distance = 0.0;
  if (action.stop_by_constraint && mutable_start_index < trajectory.points.size())
  {
    const size_t anchor_index = mutable_start_index > 0 ? mutable_start_index - 1 : mutable_start_index;
    stop_start_s = trajectory.points[anchor_index].s;
    stop_start_v = std::max(0.0, trajectory.points[anchor_index].v);
    previous_limited_v = stop_start_v;
    stop_distance = std::max(1.0e-3, action.stop_s - stop_start_s);
  }

  for (size_t i = mutable_start_index; i < trajectory.points.size(); ++i)
  {
    auto& point = trajectory.points[i];
    double target_v = std::max(0.0, point.v);

    if (std::isfinite(action.speed_cap))
    {
      target_v = std::min(target_v, action.speed_cap);
    }

    if (action.stop_by_constraint)
    {
      if (point.s >= action.stop_s)
      {
        target_v = 0.0;
      }
      else
      {
        // 1. 线性渐降上限：从拼接承接速度开始，沿剩余距离持续下降到 0。
        //    这避免只在临近“必须按规定减速度刹停”的边界时才突然降速。
        const double travelled_s = std::max(0.0, point.s - stop_start_s);
        const double linear_ratio = std::max(0.0, std::min(1.0, 1.0 - travelled_s / stop_distance));
        const double smooth_stop_v = stop_start_v * linear_ratio;

        // 2. 物理刹停上限：以停车线为终点反推当前点允许的最大速度：
        //   v^2 <= 2 * a * remain_s
        // 3. previous_limited_v 是上一点已经限制后的速度，保证停车让行过程中
        //    不会因为当前轮基础 planner 重新给出较高速度而产生回升。
        // 4. target_v 初值是当前轨迹点原速度，所以最终 min() 只会降速，不会抬速。
        const double remain_s = std::max(0.0, action.stop_s - point.s);
        const double stop_limit_v = std::sqrt(std::max(0.0, 2.0 * decel * remain_s));
        target_v = std::min(target_v, smooth_stop_v);
        target_v = std::min(target_v, stop_limit_v);
        target_v = std::min(target_v, previous_limited_v);
      }
    }

    if (std::fabs(point.v - target_v) > 1.0e-3)
    {
      point.v = target_v;
      velocity_changed = true;
    }
    if (action.stop_by_constraint)
    {
      previous_limited_v = target_v;
    }
  }

  return velocity_changed;
}

bool buildEmergencyStopFallback(planning_msgs::TrajectoryPointArray& trajectory,
                                const size_t stitching_end_index,
                                const size_t mutable_start_index,
                                const double requested_stop_s,
                                const double emergency_deceleration,
                                const double min_stop_horizon,
                                const double time_dt_floor,
                                std::string* reject_reason)
{
  // 当规则速度规划和 QP 都无法形成可信轨迹时，最后的安全冗余不应恢复原始高速轨迹，
  // 而应生成一条“尽快停车”的轨迹。这里使用比舒适让行更大的减速度：
  // - 若有冲突停车点 stop_s，则优先保证不越过该停车点；
  // - 若没有可用 stop_s，则按当前速度和紧急减速度估计一个刹停距离；
  // - 只修改非拼接段，拼接前缀仍交给上游规划/控制承接。
  if (trajectory.points.empty() || mutable_start_index >= trajectory.points.size())
  {
    if (reject_reason)
    {
      *reject_reason = "emergency_stop_too_few_points";
    }
    return false;
  }

  const size_t start_index = std::min(stitching_end_index, trajectory.points.size() - 1);
  const double start_s = trajectory.points[start_index].s;
  const double start_v = std::max(0.0, trajectory.points[start_index].v);
  const double decel = std::max(0.1, emergency_deceleration);
  const double estimated_stop_s = start_s + start_v * start_v / (2.0 * decel);

  double emergency_stop_s = estimated_stop_s;
  if (std::isfinite(requested_stop_s))
  {
    // requested_stop_s 来自冲突入口前的停车点，不能为了“平滑”越过它。
    emergency_stop_s = std::min(requested_stop_s, estimated_stop_s);
  }
  emergency_stop_s = std::max(start_s + 1.0e-3, emergency_stop_s);

  for (size_t i = mutable_start_index; i < trajectory.points.size(); ++i)
  {
    auto& point = trajectory.points[i];
    const double remain_s = emergency_stop_s - point.s;
    if (remain_s <= 0.0)
    {
      point.v = 0.0;
      point.a = 0.0;
      continue;
    }

    // 紧急停车包络：v^2 <= 2*a*s。该速度上限会随停车点临近快速降低，
    // 比舒适让行更保守，作为规则/QP 都失败时的最后保护。
    const double stop_limit_v = std::sqrt(std::max(0.0, 2.0 * decel * remain_s));
    point.v = std::min(std::max(0.0, point.v), stop_limit_v);
  }

  truncateTrajectoryAtStopS(trajectory, stitching_end_index, emergency_stop_s);
  recomputeTimingFrom(trajectory, stitching_end_index);
  extendStopPointTimeHorizon(trajectory, stitching_end_index, min_stop_horizon);
  enforceStrictlyIncreasingRelativeTime(trajectory, time_dt_floor);

  return validateTrajectoryForController(trajectory, mutable_start_index, reject_reason);
}

}  // namespace

void ConflictConstraintProcessor::loadParam(ros::NodeHandle& private_nh)
{
  private_nh.param<bool>("enable_conflict_constraint", enabled_, false);
  private_nh.param<double>("conflict_constraint_timeout", constraint_timeout_, 2.1);
  private_nh.param<double>("conflict_timeout_max_speed", timeout_max_speed_, 1.0);
  private_nh.param<double>("conflict_deceleration_limit", deceleration_limit_, 1.5);
  private_nh.param<double>("conflict_emergency_stop_deceleration",
                           emergency_stop_deceleration_,
                           2.5);
  private_nh.param<double>("conflict_stop_margin", stop_margin_, 1.0);
  private_nh.param<double>("conflict_stop_buffer", stop_buffer_, 0.0);
  private_nh.param<double>("conflict_projection_max_lateral_error", projection_max_lateral_error_, 2.0);
  private_nh.param<double>("conflict_projection_min_s_gap", projection_min_s_gap_, 0.2);
  private_nh.param<double>("conflict_stitching_start_match_max_distance",
                           stitching_start_match_max_distance_, 1.0);
  private_nh.param<double>("conflict_planning_cycle_time", planning_cycle_time_, 0.1);
  private_nh.param<double>("conflict_follow_min_distance", follow_min_distance_, 5.0);
  private_nh.param<double>("conflict_follow_time_headway", follow_time_headway_, 2.0);
  private_nh.param<double>("conflict_follow_gap_gain", follow_gap_gain_, 0.5);
  private_nh.param<double>("conflict_follow_relative_speed_gain",
                           follow_relative_speed_gain_,
                           0.8);
  private_nh.param<double>("conflict_follow_closing_time", follow_closing_time_, 3.0);
  private_nh.param<double>("conflict_follow_brake_deceleration",
                           follow_brake_deceleration_,
                           1.0);
  private_nh.param<double>("conflict_follow_emergency_distance", follow_emergency_distance_, 2.0);
  private_nh.param<double>("conflict_follow_state_timeout", follow_state_timeout_, 0.5);
  private_nh.param<double>("conflict_follow_activation_grace_time",
                           follow_activation_grace_time_,
                           0.5);
  private_nh.param<double>("conflict_follow_bumper_gap_offset", follow_bumper_gap_offset_, 3.7);
  private_nh.param<double>("conflict_follow_projection_max_lateral_error",
                           follow_projection_max_lateral_error_,
                           projection_max_lateral_error_);
}

void ConflictConstraintProcessor::updateConstraint(const planning_msgs::ConflictConstraint& constraint)
{
  const bool follow_constraint =
      constraint.yield_strategy == planning_msgs::ConflictConstraint::STRATEGY_FOLLOW &&
      constraint.has_follow_constraint &&
      !constraint.peer_id.empty();
  if (follow_constraint)
  {
    ensureFollowPeerSubscriptions(constraint.peer_id);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (follow_constraint)
  {
    const bool new_follow =
        !have_active_follow_ ||
        active_follow_peer_id_ != constraint.peer_id;
    if (new_follow)
    {
      have_active_follow_ = true;
      active_follow_peer_id_ = constraint.peer_id;
      active_follow_conflict_id_ = constraint.conflict_id;
      follow_activation_start_ = ros::Time::now();
    }
  }
  else
  {
    have_active_follow_ = false;
    active_follow_peer_id_.clear();
    active_follow_conflict_id_.clear();
    follow_activation_start_ = ros::Time();
  }
  latest_constraint_ = constraint;
  have_constraint_ = true;
}

void ConflictConstraintProcessor::ensureFollowPeerSubscriptions(const std::string& peer_id)
{
  if (peer_id.empty() || peer_id == subscribed_follow_peer_id_)
  {
    return;
  }

  ros::NodeHandle nh;
  follow_peer_pose_sub_ = nh.subscribe<localization_msgs::Localization>(
      vehicleTopic(peer_id, "odomData"),
      1,
      boost::bind(&ConflictConstraintProcessor::onFollowPeerLocalization, this, peer_id, _1));
  follow_peer_chassis_sub_ = nh.subscribe<driver_msgs::ChassisReport>(
      vehicleTopic(peer_id, "chassis"),
      1,
      boost::bind(&ConflictConstraintProcessor::onFollowPeerChassis, this, peer_id, _1));

  {
    std::lock_guard<std::mutex> lock(mutex_);
    subscribed_follow_peer_id_ = peer_id;
    follow_peer_state_ = FollowPeerState();
    follow_peer_state_.peer_id = peer_id;
  }

  ROS_INFO_STREAM("[Conflict Velocity Decision] Subscribe high-rate follow peer state: peer="
                  << peer_id << " pose_topic=" << vehicleTopic(peer_id, "odomData")
                  << " chassis_topic=" << vehicleTopic(peer_id, "chassis"));
}

void ConflictConstraintProcessor::onFollowPeerLocalization(
    const std::string& peer_id,
    const localization_msgs::Localization::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (peer_id != subscribed_follow_peer_id_)
  {
    return;
  }

  follow_peer_state_.peer_id = peer_id;
  follow_peer_state_.position = msg->location.pose.pose.position;
  follow_peer_state_.pose_stamp = ros::Time::now();
  follow_peer_state_.have_pose = true;
}

void ConflictConstraintProcessor::onFollowPeerChassis(
    const std::string& peer_id,
    const driver_msgs::ChassisReport::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (peer_id != subscribed_follow_peer_id_)
  {
    return;
  }

  double speed = msg->current_velocity;
  if (msg->gear_location == 7)
  {
    speed = -speed;
  }
  follow_peer_state_.peer_id = peer_id;
  follow_peer_state_.speed = std::fabs(speed);
  follow_peer_state_.speed_stamp = ros::Time::now();
  follow_peer_state_.have_speed = true;
}

ConflictConstraintProcessor::FollowPeerState
ConflictConstraintProcessor::copyFollowPeerState(const std::string& peer_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (peer_id != follow_peer_state_.peer_id)
  {
    return {};
  }
  return follow_peer_state_;
}

void ConflictConstraintProcessor::apply(planning_msgs::TrajectoryPointArray& trajectory,
                                        const geometry_msgs::Point& planning_start_point)
{
  if (!enabled_)
  {
    ROS_INFO_THROTTLE(2.0,
                      "[Conflict Velocity Decision] Conflict constraint processing is disabled; "
                      "keep the original planned velocity.");
    return;
  }

  if (trajectory.points.empty())
  {
    ROS_WARN_THROTTLE(1.0,
                      "[Conflict Velocity Decision] Input trajectory is empty; "
                      "skip conflict velocity resolution.");
    return;
  }

  planning_msgs::ConflictConstraint constraint;
  bool have_constraint = false;
  bool have_active_follow = false;
  ros::Time follow_activation_start;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_constraint_)
    {
      constraint = latest_constraint_;
      have_constraint = true;
      have_active_follow = have_active_follow_;
      follow_activation_start = follow_activation_start_;
    }
  }

  if (!have_constraint)
  {
    ROS_INFO_THROTTLE(2.0,
                      "[Conflict Velocity Decision] No conflict decision has been received; "
                      "keep the original planned velocity.");
    return;
  }

  const ros::Time now = ros::Time::now();
  const bool stamp_valid = !constraint.header.stamp.isZero();
  const double age = stamp_valid ? (now - constraint.header.stamp).toSec()
                                 : std::numeric_limits<double>::infinity();
  const bool constraint_timeout = !stamp_valid || age < 0.0 || age > constraint_timeout_;
  const bool decision_valid = !constraint.decision_source.empty();
  const uint8_t active_strategy = constraint.yield_strategy;
  const bool stop_strategy =
      active_strategy == planning_msgs::ConflictConstraint::STRATEGY_STOP_AND_WAIT ||
      active_strategy == planning_msgs::ConflictConstraint::STRATEGY_EMERGENCY_STOP;
  const bool follow_strategy =
      active_strategy == planning_msgs::ConflictConstraint::STRATEGY_FOLLOW;

  if (!constraint_timeout)
  {
    if (constraint.role != planning_msgs::ConflictConstraint::ROLE_YIELD)
    {
      // ROLE_NONE 表示冲突预判未作出让行决策；ROLE_PROCEED 表示本车被授权先行。
      // 两者通常不需要冲突消解速度规划，保持原规划轨迹。
      // 决策保持、冲突区扩展和释放时机都应由 conflict_resolver_node 负责；
      // 速度层只执行当前消息，不再额外保存上一帧 YIELD 决策。
      if (constraint.role == planning_msgs::ConflictConstraint::ROLE_PROCEED)
      {
        ROS_INFO_THROTTLE(
            1.0,
            "[Conflict Velocity Decision] ego=%s peer=%s decision=PROCEED; "
            "keep the original planned velocity."
            " source=%s reason=%s",
            constraint.ego_id.c_str(),
            constraint.peer_id.c_str(),
            constraint.decision_source.c_str(),
            constraint.decision_reason.c_str());
      }
      else
      {
        ROS_INFO_THROTTLE(
            1.0,
            "[Conflict Velocity Decision] ego=%s peer=%s decision=NONE; "
            "keep the original planned velocity.",
            constraint.ego_id.c_str(),
            constraint.peer_id.c_str());
      }
      return;
    }

    if (!decision_valid)
    {
      // role=YIELD 但没有 decision_source，说明冲突预判没有输出完整决策溯源。
      // 此时不执行速度消解，避免 planner 根据半成品消息自行生成让行速度。
      ROS_WARN_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s decision=YIELD but "
          "decision_source is missing; reject the constraint and keep the original velocity.",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str());
      return;
    }

    if (!stop_strategy && !follow_strategy)
    {
      // 当前速度执行器只实现停车等待和跟车限速。SLOW_DOWN 等策略应由后续专门模块实现，
      // 不能在这里临时拼规则，否则会重新形成“速度层自行决策”的问题。
      ROS_WARN_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s strategy=%s is not supported; "
          "keep the original planned velocity.",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str(),
          strategyName(active_strategy));
      return;
    }

    if (stop_strategy && !constraint.has_spatial_constraint)
    {
      // 消息中不携带候选轨迹 s。若没有 map 坐标冲突入口/出口，planner 无法可靠判断
      // 当前轨迹是否仍经过该冲突路段，因此直接丢弃该约束。
      ROS_WARN_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s decision=YIELD but the spatial "
          "conflict section is missing; reject the constraint and keep the original velocity.",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str());
      return;
    }

    if (follow_strategy && !constraint.has_follow_constraint)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s decision=YIELD/FOLLOW but "
          "the follow constraint is missing; reject the constraint and keep the original velocity.",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str());
      return;
    }
  }

  // 只有确实需要执行速度处理时，才准备冲突模块内部轨迹：
  // - 超时：只做保守限速；
  // - YIELD + STOP_AND_WAIT/EMERGENCY_STOP：生成停车等待轨迹。
  auto prepared_trajectory = prepareConflictTrajectory(
      trajectory,
      planning_start_point,
      stitching_start_match_max_distance_,
      planning_cycle_time_);
  const auto& original_trajectory = prepared_trajectory.original;
  auto& normalized_trajectory = prepared_trajectory.normalized;
  const size_t stitching_end_index = prepared_trajectory.stitching_end_index;
  const size_t mutable_start_index = prepared_trajectory.mutable_start_index;

  double stop_s = std::numeric_limits<double>::infinity();
  double stop_start_s = normalized_trajectory.points[stitching_end_index].s;
  double stop_initial_speed = std::max(0.0, normalized_trajectory.points[stitching_end_index].v);
  bool stop_by_constraint = false;
  bool changed = false;

  if (constraint_timeout)
  {
    // 决策消息超时后，不再使用上一帧让行关系、冲突入口和目标进入时间。
    // 但出于安全考虑，需要立刻进入保守降速，只对非拼接段施加低速上限。
    planning_msgs::TrajectoryPointArray timeout_trajectory = normalized_trajectory;
    VelocityConstraintAction timeout_action;
    timeout_action.speed_cap = std::max(0.0, timeout_max_speed_);
    changed = applyVelocityConstraintAction(timeout_trajectory,
                                            mutable_start_index,
                                            timeout_action,
                                            deceleration_limit_);
    ROS_WARN_THROTTLE(
        1.0,
        "[Conflict Velocity Decision] ego=%s conflict decision timed out "
        "(age=%.2fs, limit=%.2fs); apply conservative speed cap=%.2fm/s.",
        constraint.ego_id.c_str(),
        age,
        constraint_timeout_,
        timeout_action.speed_cap);

    if (!changed)
    {
      return;
    }

    recomputeTimingFrom(timeout_trajectory, stitching_end_index);
    enforceStrictlyIncreasingRelativeTime(timeout_trajectory, kFinalTimeDtFloor);
    std::string reject_reason;
    if (validateTrajectoryForController(timeout_trajectory, mutable_start_index, &reject_reason))
    {
      trajectory = timeout_trajectory;
      ROS_WARN_THROTTLE(1.0,
                        "[Conflict Velocity Decision] ego=%s output=TIMEOUT_SPEED_CAP_TRAJECTORY. "
                        "speed_cap=%.2fm/s points=%zu.",
                        constraint.ego_id.c_str(),
                        timeout_action.speed_cap,
                        trajectory.points.size());
      return;
    }

    planning_msgs::TrajectoryPointArray emergency_stop_trajectory = normalized_trajectory;
    std::string emergency_reject_reason;
    if (buildEmergencyStopFallback(emergency_stop_trajectory,
                                   stitching_end_index,
                                   mutable_start_index,
                                   std::numeric_limits<double>::infinity(),
                                   emergency_stop_deceleration_,
                                   kMinStopTimeHorizon,
                                   kFinalTimeDtFloor,
                                   &emergency_reject_reason))
    {
      ROS_WARN_THROTTLE(1.0,
                        "[Conflict Velocity Decision] ego=%s output=EMERGENCY_STOP_TRAJECTORY. "
                        "Timeout speed-cap trajectory failed validation. reason=%s",
                        constraint.ego_id.c_str(),
                        reject_reason.c_str());
      trajectory = emergency_stop_trajectory;
      return;
    }

    ROS_WARN_THROTTLE(1.0,
                      "[Conflict Velocity Decision] ego=%s output=ORIGINAL_PLANNER_TRAJECTORY. "
                      "Timeout speed-cap and emergency stop fallback both failed validation. "
                      "reason=%s emergency_reason=%s",
                      constraint.ego_id.c_str(),
                      reject_reason.c_str(),
                      emergency_reject_reason.c_str());
    trajectory = original_trajectory;
    return;
  }

  if (active_strategy == planning_msgs::ConflictConstraint::STRATEGY_FOLLOW)
  {
    trajectory = normalized_trajectory;
    const double current_speed = std::max(0.0, trajectory.points[stitching_end_index].v);
    const double target_gap =
        std::max(std::max(0.0, follow_min_distance_),
                 current_speed * std::max(0.0, follow_time_headway_));
    const FollowPeerState follow_state = copyFollowPeerState(constraint.peer_id);
    const bool pose_fresh =
        follow_state.have_pose &&
        (now - follow_state.pose_stamp).toSec() >= 0.0 &&
        (now - follow_state.pose_stamp).toSec() <= follow_state_timeout_;
    const bool speed_fresh =
        follow_state.have_speed &&
        (now - follow_state.speed_stamp).toSec() >= 0.0 &&
        (now - follow_state.speed_stamp).toSec() <= follow_state_timeout_;
    const bool follow_activation_grace =
        have_active_follow &&
        !follow_activation_start.isZero() &&
        (now - follow_activation_start).toSec() >= 0.0 &&
        (now - follow_activation_start).toSec() <=
            std::max(0.0, follow_activation_grace_time_);
    const bool use_snapshot_pose =
        !pose_fresh &&
        follow_activation_grace &&
        constraint.has_follow_peer_snapshot;
    const bool use_snapshot_speed =
        !speed_fresh &&
        follow_activation_grace &&
        constraint.has_follow_peer_snapshot;

    double actual_gap = 0.0;
    double lead_speed = 0.0;
    bool lead_projected_ahead = false;
    double lead_s = std::numeric_limits<double>::quiet_NaN();
    double lead_lateral_error = std::numeric_limits<double>::infinity();
    const bool have_lead_position = pose_fresh || use_snapshot_pose;
    const geometry_msgs::Point lead_position =
        pose_fresh ? follow_state.position : constraint.follow_peer_position;

    if (have_lead_position)
    {
      const std::vector<PathPoint> follow_path_points = makePathPoints(trajectory);
      lead_projected_ahead =
          projectPointToCurrentTrajectory(trajectory,
                                          follow_path_points,
                                          lead_position,
                                          lead_s,
                                          lead_lateral_error) &&
          lead_lateral_error <= follow_projection_max_lateral_error_ &&
          lead_s >= trajectory.points[stitching_end_index].s;
      if (lead_projected_ahead)
      {
        actual_gap = std::max(0.0,
                              lead_s -
                                  trajectory.points[stitching_end_index].s -
                                  std::max(0.0, follow_bumper_gap_offset_));
      }
      else if (std::isfinite(lead_s) && lead_s < trajectory.points[stitching_end_index].s)
      {
        actual_gap = 0.0;
      }
      else
      {
        const auto& ego_point = trajectory.points[stitching_end_index];
        actual_gap = std::max(0.0,
                              std::hypot(lead_position.x - ego_point.x,
                                         lead_position.y - ego_point.y) -
                                  std::max(0.0, follow_bumper_gap_offset_));
      }
    }

    if (speed_fresh)
    {
      lead_speed = std::max(0.0, follow_state.speed);
    }
    else if (use_snapshot_speed)
    {
      lead_speed = std::max(0.0, constraint.follow_peer_speed);
    }
    else if (have_lead_position)
    {
      lead_speed = 0.0;
    }

    const double gap_error = actual_gap - target_gap;
    const double relative_speed = lead_speed - current_speed;
    const double gap_control_cap =
        lead_speed +
        std::max(0.0, follow_gap_gain_) * gap_error +
        std::max(0.0, follow_relative_speed_gain_) * relative_speed;
    const double closing_time = std::max(1.0e-3, follow_closing_time_);
    const double closing_speed_cap =
        lead_speed + std::max(0.0, gap_error) / closing_time;
    const double brake_distance =
        std::max(0.0, actual_gap - std::max(0.0, follow_emergency_distance_));
    const double brake_speed_cap =
        std::sqrt(std::max(0.0,
                           lead_speed * lead_speed +
                               2.0 * std::max(0.0, follow_brake_deceleration_) *
                                   brake_distance));
    double speed_cap = std::min(gap_control_cap, std::min(closing_speed_cap, brake_speed_cap));
    if (gap_error < 0.0)
    {
      speed_cap = std::min(speed_cap, lead_speed);
    }
    if (!have_lead_position || actual_gap <= std::max(0.0, follow_emergency_distance_))
    {
      speed_cap = 0.0;
    }
    speed_cap = std::max(0.0, speed_cap);

    VelocityConstraintAction follow_action;
    follow_action.speed_cap = speed_cap;
    changed = applyVelocityConstraintAction(trajectory,
                                            mutable_start_index,
                                            follow_action,
                                            deceleration_limit_);
    if (!changed)
    {
      ROS_INFO_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s strategy=FOLLOW. "
          "Current planned velocity already satisfies speed_cap=%.2fm/s. "
          "gap=%.2fm target_gap=%.2fm lead_speed=%.2fm/s current_speed=%.2fm/s "
          "gap_cap=%.2fm/s closing_cap=%.2fm/s brake_cap=%.2fm/s "
          "pose_fresh=%d speed_fresh=%d snapshot_pose=%d snapshot_speed=%d grace=%d.",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str(),
          speed_cap,
          actual_gap,
          target_gap,
          lead_speed,
          current_speed,
          gap_control_cap,
          closing_speed_cap,
          brake_speed_cap,
          pose_fresh,
          speed_fresh,
          use_snapshot_pose,
          use_snapshot_speed,
          follow_activation_grace);
      return;
    }

    recomputeTimingFrom(trajectory, stitching_end_index);
    enforceStrictlyIncreasingRelativeTime(trajectory, kFinalTimeDtFloor);
    std::string follow_reject_reason;
    if (validateTrajectoryForController(trajectory, mutable_start_index, &follow_reject_reason))
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s output=FOLLOW_SPEED_TRAJECTORY. "
          "gap=%.2fm target_gap=%.2fm lead_speed=%.2fm/s current_speed=%.2fm/s "
          "speed_cap=%.2fm/s gap_cap=%.2fm/s closing_cap=%.2fm/s brake_cap=%.2fm/s "
          "lead_projected=%d lead_s=%.2fm lead_l=%.2fm "
          "snapshot_pose=%d snapshot_speed=%d grace=%d points=%zu.",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str(),
          actual_gap,
          target_gap,
          lead_speed,
          current_speed,
          speed_cap,
          gap_control_cap,
          closing_speed_cap,
          brake_speed_cap,
          lead_projected_ahead,
          lead_s,
          lead_lateral_error,
          use_snapshot_pose,
          use_snapshot_speed,
          follow_activation_grace,
          trajectory.points.size());
      return;
    }

    planning_msgs::TrajectoryPointArray emergency_stop_trajectory = normalized_trajectory;
    std::string emergency_reject_reason;
    if (buildEmergencyStopFallback(emergency_stop_trajectory,
                                   stitching_end_index,
                                   mutable_start_index,
                                   std::numeric_limits<double>::infinity(),
                                   emergency_stop_deceleration_,
                                   kMinStopTimeHorizon,
                                   kFinalTimeDtFloor,
                                   &emergency_reject_reason))
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s output=EMERGENCY_STOP_TRAJECTORY. "
          "FOLLOW trajectory failed validation. reason=%s",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str(),
          follow_reject_reason.c_str());
      trajectory = emergency_stop_trajectory;
      return;
    }

    ROS_WARN_THROTTLE(
        1.0,
        "[Conflict Velocity Decision] ego=%s peer=%s output=ORIGINAL_PLANNER_TRAJECTORY. "
        "FOLLOW speed-cap and emergency stop fallback both failed validation. "
        "reason=%s emergency_reason=%s",
        constraint.ego_id.c_str(),
        constraint.peer_id.c_str(),
        follow_reject_reason.c_str(),
        emergency_reject_reason.c_str());
    trajectory = original_trajectory;
    return;
  }

  const std::vector<PathPoint> path_points = makePathPoints(normalized_trajectory);
  const auto projected_section = projectConflictSection(
      normalized_trajectory, path_points, constraint,
      projection_max_lateral_error_, projection_min_s_gap_);
  const bool entry_projection_valid =
      projected_section.entry_projected &&
      projected_section.entry_lateral_error <= projection_max_lateral_error_;

  const double decel = std::max(0.1, deceleration_limit_);
  const double braking_stop_s =
      stop_start_s + stop_initial_speed * stop_initial_speed / (2.0 * decel);

  double requested_stop_s = std::numeric_limits<double>::infinity();
  if (entry_projection_valid)
  {
    // 判定层给出冲突入口绝对坐标；速度层只负责把入口点投影到当前轨迹，
    // 并在入口前生成停车点。停车策略以这条阈值线为目标逐渐降速，
    // 不再因为理论制动距离更短就提前停在规划起点附近。
    requested_stop_s = std::max(0.0,
                                projected_section.s_in -
                                    std::max(0.0, stop_margin_) -
                                    std::max(0.0, stop_buffer_));
  }
  else
  {
    // STOP 是安全策略。即使入口点无法投影到当前轨迹，也不能恢复原始速度；
    // 这通常意味着锁定冲突入口已落在当前轨迹身后，此时直接按制动距离停车。
    requested_stop_s = braking_stop_s;
    ROS_WARN_THROTTLE(
        1.0,
        "[Conflict Velocity Decision] ego=%s peer=%s strategy=%s but the conflict "
        "entry cannot be projected to current trajectory; apply immediate stop fallback. "
        "entry_projected=%d exit_projected=%d entry_l=%.2f exit_l=%.2f "
        "projected_s_in=%.2f projected_s_out=%.2f.",
        constraint.ego_id.c_str(),
        constraint.peer_id.c_str(),
        strategyName(active_strategy),
        projected_section.entry_projected,
        projected_section.exit_projected,
        projected_section.entry_lateral_error,
        projected_section.exit_lateral_error,
        projected_section.s_in,
        projected_section.s_out);
  }

  // 正常情况下，stop_s 就是冲突入口前阈值停车线；速度包络会保证所有点速度
  // 不高于“能在 stop_s 刹停”的上限，且不会把原轨迹速度抬高。
  // 只有入口无法投影时，requested_stop_s 才会退化成 braking_stop_s。
  stop_s = requested_stop_s;
  stop_s = std::max(stop_start_s + 1.0e-3, stop_s);
  stop_by_constraint = std::isfinite(stop_s);
  if (!stop_by_constraint)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[Conflict Velocity Decision] ego=%s peer=%s strategy=%s could not "
                      "compute a finite stop point; keep the original planned velocity.",
                      constraint.ego_id.c_str(),
                      constraint.peer_id.c_str(),
                      strategyName(active_strategy));
    return;
  }

  trajectory = normalized_trajectory;
  VelocityConstraintAction stop_action;
  stop_action.stop_by_constraint = true;
  stop_action.stop_s = stop_s;
  changed = applyVelocityConstraintAction(trajectory,
                                          mutable_start_index,
                                          stop_action,
                                          deceleration_limit_);

  ROS_WARN_THROTTLE(
      1.0,
      "[Conflict Velocity Decision] ego=%s peer=%s strategy=%s. "
      "Execute STOP_AND_WAIT from planning start. entry_projected=%d "
      "entry_s=%.2fm requested_stop_s=%.2fm actual_stop_s=%.2fm.",
      constraint.ego_id.c_str(),
      constraint.peer_id.c_str(),
      strategyName(active_strategy),
      entry_projection_valid,
      projected_section.s_in,
      requested_stop_s,
      stop_s);

  truncateTrajectoryAtStopS(trajectory, stitching_end_index, stop_s);
  recomputeTimingFrom(trajectory, stitching_end_index);
  extendStopPointTimeHorizon(trajectory, stitching_end_index, kMinStopTimeHorizon);
  enforceStrictlyIncreasingRelativeTime(trajectory, kFinalTimeDtFloor);

  std::string stop_reject_reason;
  if (validateTrajectoryForController(trajectory, mutable_start_index, &stop_reject_reason))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[Conflict Velocity Decision] ego=%s peer=%s output=RULE_BASED_STOP_TRAJECTORY. "
        "stop_s=%.2fm points=%zu last_time=%.2fs.",
        constraint.ego_id.c_str(),
        constraint.peer_id.c_str(),
        stop_s,
        trajectory.points.size(),
        trajectory.points.back().relative_time);
    return;
  }

  planning_msgs::TrajectoryPointArray emergency_stop_trajectory = normalized_trajectory;
  std::string emergency_reject_reason;
  if (buildEmergencyStopFallback(emergency_stop_trajectory,
                                 stitching_end_index,
                                 mutable_start_index,
                                 stop_s,
                                 emergency_stop_deceleration_,
                                 kMinStopTimeHorizon,
                                 kFinalTimeDtFloor,
                                 &emergency_reject_reason))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[Conflict Velocity Decision] ego=%s peer=%s output=EMERGENCY_STOP_TRAJECTORY. "
                      "Rule-based stop trajectory validation failed. reason=%s stop_s=%.2f",
                      constraint.ego_id.c_str(),
                      constraint.peer_id.c_str(),
                      stop_reject_reason.c_str(),
                      stop_s);
    trajectory = emergency_stop_trajectory;
    return;
  }

  ROS_WARN_THROTTLE(1.0,
                    "[Conflict Velocity Decision] ego=%s peer=%s output=ORIGINAL_PLANNER_TRAJECTORY. "
                    "Both rule-based and emergency stop trajectories failed validation. "
                    "reason=%s emergency_reason=%s stop_s=%.2f",
                    constraint.ego_id.c_str(),
                    constraint.peer_id.c_str(),
                    stop_reject_reason.c_str(),
                    emergency_reject_reason.c_str(),
                    stop_s);
  trajectory = original_trajectory;
}

}  // namespace conflict_prediction_resolution
