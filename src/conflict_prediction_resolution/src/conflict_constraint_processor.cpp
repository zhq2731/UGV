#include "conflict_prediction_resolution/conflict_constraint_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
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

// 以下常量属于控制器输入保护和拼接稳定性的内部阈值，不作为外部参数开放：
// 1. 它们不是冲突消解策略本身的调参项；
// 2. 数值只用于避免重复几何点、过短停车视野等工程异常；
// 3. 频繁改动反而会让实验参数含义变乱，因此固定在代码中并在这里集中说明。
constexpr double kMinStopTimeHorizon = 3.0;                // s，停车点至少保留的时间视野。
constexpr double kFinalTimeDtFloor = 0.02;                 // s，最终输出轨迹最小时间间隔保护。
constexpr double kStoppedTailLength = 6.0;                 // m，停车后保留给控制器横向跟踪的零速几何尾迹。
constexpr double kStoppedTailTimeStep = 0.1;               // s，零速几何尾迹相邻点的固定时间间隔。
constexpr double kYieldReleaseHoldTime = 3.0;              // s，ROLE_NONE 后继续保持最近让行决策的滞回时间。

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

bool computeComfortYieldCruiseSpeed(const double distance_to_entry,
                                    const double target_entry_time,
                                    const double initial_speed,
                                    const double comfortable_deceleration,
                                    double& cruise_speed)
{
  // 根据“先舒适减速，再低速巡航”的模型，反推为了不早于目标时间进入冲突区，
  // 需要降到的巡航让行速度。
  //
  // 运动模型：
  //   1. 车辆从初速度 v0 开始，以额定舒适减速度 b 降到 v_c；
  //   2. 到达 v_c 后保持 v_c 巡航；
  //   3. 在 target_entry_time 时刻刚好走完 distance_to_entry。
  //
  // 若减速阶段能在目标时间内完成：
  //   D = v_c * T + (v0 - v_c)^2 / (2b)
  // 这里通过一元二次方程求 v_c。若目标时间/距离要求过严，说明靠舒适降速已经
  // 无法满足，需要退化为停车让行。
  const double distance = std::max(0.0, distance_to_entry);
  const double target_time = target_entry_time;
  const double v0 = std::max(0.0, initial_speed);
  const double decel = std::max(1.0e-3, comfortable_deceleration);

  if (!std::isfinite(distance) || !std::isfinite(target_time) ||
      !std::isfinite(v0) || target_time <= 1.0e-3)
  {
    return false;
  }

  if (distance >= v0 * target_time)
  {
    // 按初速度走都不会早到，不需要为了冲突额外降速。
    cruise_speed = v0;
    return true;
  }

  const double stop_time = v0 / decel;
  const double min_distance =
      stop_time <= target_time
          ? v0 * v0 / (2.0 * decel)
          : v0 * target_time - 0.5 * decel * target_time * target_time;
  if (distance < min_distance - 1.0e-3)
  {
    // 在额定舒适减速度下，即使持续减速也会越过目标距离。
    // 这种情况不能再给“巡航让行速度”，应由上层切换为停车让行。
    return false;
  }

  const double discriminant =
      decel * decel * target_time * target_time -
      2.0 * decel * (v0 * target_time - distance);
  if (discriminant < -1.0e-6)
  {
    return false;
  }

  const double delta_v = decel * target_time - std::sqrt(std::max(0.0, discriminant));
  cruise_speed = std::max(0.0, v0 - std::max(0.0, delta_v));
  return std::isfinite(cruise_speed);
}

double comfortDecelSpeedCapAtS(const double query_s,
                               const double start_s,
                               const double initial_speed,
                               const double cruise_speed,
                               const double comfortable_deceleration)
{
  // 生成“从初速度按舒适减速度逐渐降到巡航让行速度”的速度包络。
  // 该包络不会在规划起点直接跳到低速，而是随 s 增长逐步降低：
  //   v(s)^2 = v0^2 - 2*b*(s - s0)
  // 并且最低不低于 cruise_speed。
  const double ds = std::max(0.0, query_s - start_s);
  const double v0 = std::max(0.0, initial_speed);
  const double cruise = std::max(0.0, cruise_speed);
  const double decel = std::max(1.0e-3, comfortable_deceleration);
  const double decel_speed =
      std::sqrt(std::max(0.0, v0 * v0 - 2.0 * decel * ds));
  return std::max(cruise, decel_speed);
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
  // 这里参考 Apollo 的拼接时间表达原则：
  // 1. 拼接段末点，也就是本轮路径规划起点，表示“当前时刻向前一个规划周期”的点，
  //    因此固定为单次规划循环时间；
  // 2. 末点之前的拼接点按原相邻时间间隔向前递推，可以小于规划周期，也允许为负；
  // 3. 不在这里修改非拼接后缀，后缀由规则速度修正和固定时间重采样负责。
  //
  // 注意：旧实现会把前缀点 relative_time 强制夹到 0 以上，这不符合 Apollo。
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
    // 旧实现按 0.02m/s 的“虚拟爬行速度”展开时间，6m 尾迹会生成约 300s 的时间轴，
    // 不仅破坏下一轮拼接，还可能使控制器追加的 100s 末点发生时间倒退。
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
                                          const double fixed_dt,
                                          const int max_suffix_points)
{
  // 将非拼接段转为固定时间间隔粗轨迹：
  // - 拼接前缀保持原样，避免破坏上一帧承接段；
  // - 后缀按固定 dt 在 relative_time 上插值；
  // - 后缀点数做硬限制，避免低速让行时把几十秒轨迹采成数千点，
  //   从而导致后续 QP dense 矩阵和控制器输入同时膨胀。
  // - 若后缀太短，不强行重采样，避免生成空轨迹。
  if (trajectory.points.empty() || fixed_prefix_end_index + 1 >= trajectory.points.size())
  {
    return;
  }

  if (trajectory.points.size() <= fixed_prefix_end_index + 2)
  {
    // 如果截断后只剩“拼接末点 + 停车点”，不要为了凑固定时间间隔强行插出
    // 一串很密的几何点。近距离停车时这些点的 xy/s 间隔会非常小，
    // 反而更容易被控制器识别成重复点。
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
  const size_t max_generated_points =
      max_suffix_points > 0 ? static_cast<size_t>(max_suffix_points)
                            : trajectory.points.size();
  size_t generated_points = 0;
  for (double t = start_time + dt;
       t <= end_time + 1.0e-6 && generated_points < max_generated_points;
       t += dt, ++generated_points)
  {
    resampled.points.push_back(interpolatePointByTime(trajectory, t, search_index));
  }

  if (generated_points >= max_generated_points && end_time > resampled.points.back().relative_time + dt)
  {
    ROS_WARN_THROTTLE(1.0,
                      "conflict fixed-time coarse trajectory truncated: suffix_points=%zu max=%zu",
                      generated_points,
                      max_generated_points);
  }

  if (resampled.points.size() > fixed_prefix_end_index + 1)
  {
    trajectory = resampled;
  }
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
  private_nh.param<double>("conflict_smooth_yield_deceleration",
                           smooth_yield_deceleration_,
                           0.8);
  private_nh.param<double>("conflict_stop_margin", stop_margin_, 1.0);
  private_nh.param<double>("conflict_stop_buffer", stop_buffer_, 0.0);
  private_nh.param<double>("conflict_min_smooth_yield_speed", min_smooth_yield_speed_, 0.3);
  private_nh.param<double>("conflict_projection_max_lateral_error", projection_max_lateral_error_, 2.0);
  private_nh.param<double>("conflict_projection_min_s_gap", projection_min_s_gap_, 0.2);
  private_nh.param<double>("conflict_stitching_start_match_max_distance",
                           stitching_start_match_max_distance_, 1.0);
  private_nh.param<double>("conflict_fixed_time_coarse_dt", fixed_time_coarse_dt_, 0.1);
  private_nh.param<int>("conflict_fixed_time_max_points", fixed_time_max_points_, 160);
  private_nh.param<double>("conflict_planning_cycle_time", planning_cycle_time_, 0.1);
  velocity_optimizer_.loadParam(private_nh);
}

void ConflictConstraintProcessor::updateConstraint(const planning_msgs::ConflictConstraint& constraint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_constraint_ = constraint;
  have_constraint_ = true;

  const bool complete_yield_constraint =
      constraint.role == planning_msgs::ConflictConstraint::ROLE_YIELD &&
      !constraint.decision_source.empty() &&
      constraint.has_spatial_constraint;
  if (complete_yield_constraint)
  {
    // 只缓存“完整的 YIELD 决策”：必须有决策来源和冲突入口/出口空间信息。
    // 这样后续 ROLE_NONE 抖动时可以短时间继续让行，但不会拿半成品消息约束车辆。
    last_yield_constraint_ = constraint;
    last_yield_update_time_ = ros::Time::now();
    have_last_yield_constraint_ = true;
  }
  else if (constraint.role == planning_msgs::ConflictConstraint::ROLE_PROCEED)
  {
    // 如果冲突模块明确授权本车先行，说明上一条让行缓存已经不应继续生效。
    have_last_yield_constraint_ = false;
  }
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
  planning_msgs::ConflictConstraint held_yield_constraint;
  bool have_constraint = false;
  bool have_held_yield_constraint = false;
  ros::Time held_yield_update_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_constraint_)
    {
      constraint = latest_constraint_;
      have_constraint = true;
    }
    if (have_last_yield_constraint_)
    {
      held_yield_constraint = last_yield_constraint_;
      held_yield_update_time = last_yield_update_time_;
      have_held_yield_constraint = true;
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
  bool using_held_yield_constraint = false;
  if (constraint.role == planning_msgs::ConflictConstraint::ROLE_NONE &&
      have_held_yield_constraint &&
      !held_yield_update_time.isZero())
  {
    const double hold_age = (now - held_yield_update_time).toSec();
    if (hold_age >= 0.0 && hold_age <= kYieldReleaseHoldTime)
    {
      // 释放滞回：避让车减速后，冲突预判可能因为时间窗暂时错开而输出 ROLE_NONE。
      // 如果 planner 立即恢复原始速度，车辆会再次进入冲突预测区，形成“让行-释放-再让行”
      // 的闭环振荡。这里短时间继续使用最近一次完整 YIELD 决策，并把目标进入时间
      // 按已经过去的时间扣减，避免把旧决策无条件刷新成新的长等待。
      constraint = held_yield_constraint;
      constraint.header.stamp = now;
      constraint.target_entry_time = std::max(0.0, constraint.target_entry_time - hold_age);
      using_held_yield_constraint = true;
      ROS_WARN_THROTTLE(1.0,
                        "hold last yield constraint after ROLE_NONE, hold_age=%.2f "
                        "target_entry_time=%.2f",
                        hold_age,
                        constraint.target_entry_time);
    }
  }

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
  double yield_start_s = std::numeric_limits<double>::infinity();
  double yield_initial_speed = 0.0;
  double yield_entry_s = std::numeric_limits<double>::infinity();
  double stop_s = std::numeric_limits<double>::infinity();
  double target_entry_time_from_now = std::numeric_limits<double>::infinity();

  if (!constraint_timeout)
  {
    if (constraint.role != planning_msgs::ConflictConstraint::ROLE_YIELD)
    {
      // ROLE_NONE 表示冲突预判未作出让行决策；ROLE_PROCEED 表示本车被授权先行。
      // 两者通常不需要冲突消解速度规划，保持原规划轨迹。
      // 但 ROLE_NONE 的短时抖动已经在上方通过 using_held_yield_constraint 被转回 YIELD，
      // 能走到这里说明没有可用保持约束，可以安全释放。
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

    if (!constraint.has_spatial_constraint)
    {
      // 消息中不再携带旧轨迹 s。若没有 map 坐标冲突入口/出口，planner 无法可靠判断
      // 当前轨迹是否仍经过该冲突路段，因此直接丢弃该约束。
      ROS_WARN_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s decision=YIELD but the spatial "
          "conflict section is missing; reject the constraint and keep the original velocity.",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str());
      return;
    }
  }

  // 只有在确实需要执行冲突速度处理时，才识别拼接段并规范内部轨迹。
  // 注意这里必须早于冲突点投影：拼接轨迹的前缀/后缀 s 可能来自不同坐标系，
  // 若直接拿原始 s 去投影和做 QP，会把正常拼接边界误判成 s 回退。
  const planning_msgs::TrajectoryPointArray original_trajectory = trajectory;
  const size_t stitching_end_index = findStitchingEndIndex(
      trajectory, planning_start_point, stitching_start_match_max_distance_);
  planning_msgs::TrajectoryPointArray normalized_trajectory = trajectory;
  normalizeStitchedPrefixTiming(normalized_trajectory, stitching_end_index, planning_cycle_time_);
  normalizeNonStitchedSuffixS(normalized_trajectory, stitching_end_index);
  const size_t mutable_start_index =
      std::min(stitching_end_index + 1, normalized_trajectory.points.size() - 1);

  if (constraint_timeout)
  {
    // 决策消息超时后，不再使用旧的让行关系、冲突入口和目标进入时间。
    // 但出于安全考虑，需要立刻进入保守降速，只对非拼接段施加低速上限。
    speed_cap = std::max(0.0, timeout_max_speed_);
    changed = true;
    ROS_WARN_THROTTLE(
        1.0,
        "[Conflict Velocity Decision] ego=%s conflict decision timed out "
        "(age=%.2fs, limit=%.2fs); apply conservative speed cap=%.2fm/s.",
        constraint.ego_id.c_str(),
        age,
        constraint_timeout_,
        speed_cap);
  }
  else if (constraint.role == planning_msgs::ConflictConstraint::ROLE_YIELD)
  {
    if (using_held_yield_constraint)
    {
      ROS_INFO_THROTTLE(1.0,
                        "[Conflict Velocity Decision] ego=%s peer=%s decision briefly changed "
                        "to NONE; hold the previous YIELD decision to prevent speed oscillation.",
                        constraint.ego_id.c_str(),
                        constraint.peer_id.c_str());
    }

    const std::vector<PathPoint> path_points = makePathPoints(normalized_trajectory);

    double ego_s_in = std::numeric_limits<double>::infinity();
    double ego_s_out = std::numeric_limits<double>::infinity();
    double entry_lateral_error = std::numeric_limits<double>::infinity();
    double exit_lateral_error = std::numeric_limits<double>::infinity();
    const bool entry_projected = projectPointToCurrentTrajectory(
        normalized_trajectory, path_points, constraint.ego_entry_point, ego_s_in, entry_lateral_error);
    const bool exit_projected = projectPointToCurrentTrajectory(
        normalized_trajectory, path_points, constraint.ego_exit_point, ego_s_out, exit_lateral_error);

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
                        "[Conflict Velocity Decision] ego=%s peer=%s decision=YIELD but the "
                        "conflict section does not match the current trajectory; keep the "
                        "original velocity. entry_projected=%d "
                        "exit_projected=%d entry_l=%.2f exit_l=%.2f "
                        "projected_s_in=%.2f projected_s_out=%.2f",
                        constraint.ego_id.c_str(),
                        constraint.peer_id.c_str(),
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
  if (std::isfinite(yield_entry_s))
  {
    target_entry_time_from_now =
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
        // 冲突预判模块不再计算建议速度上限，planner 侧根据当前轨迹剩余距离、
        // 初始速度和 target_entry_time 反推本轮规则粗解。
        //
        // 旧做法是直接把冲突入口前的所有点夹到 remain_s / time，速度会突然掉到
        // 一个常数。这里改为更贴近车辆实际的“舒适减速度包络”：
        //   先按额定舒适减速度从规划起点初速度逐渐降速；
        //   降到合适巡航让行速度后保持该速度；
        //   理论上刚好在 target_entry_time 之后进入冲突区域。
        yield_start_s = normalized_trajectory.points[stitching_end_index].s;
        yield_initial_speed = std::max(0.0, normalized_trajectory.points[stitching_end_index].v);
        double smooth_speed_cap = std::numeric_limits<double>::infinity();
        const bool smooth_speed_valid = computeComfortYieldCruiseSpeed(
            remain_s,
            target_entry_time_from_now,
            yield_initial_speed,
            smooth_yield_deceleration_,
            smooth_speed_cap);
        if (smooth_speed_valid &&
            smooth_speed_cap > std::max(0.0, min_smooth_yield_speed_))
        {
          yield_speed_cap = smooth_speed_cap;
          yield_by_time = true;
          stop_by_constraint = false;
          changed = true;
          ROS_INFO_THROTTLE(
              1.0,
              "[Conflict Velocity Decision] ego=%s peer=%s decision=SMOOTH_YIELD."
              " current_entry_time=%.2fs target_entry_time=%.2fs "
              "entry_s=%.2fm remain_s=%.2fm initial_v=%.2fm/s "
              "cruise_v=%.2fm/s comfortable_decel=%.2fm/s^2.",
              constraint.ego_id.c_str(),
              constraint.peer_id.c_str(),
              current_entry_time,
              target_entry_time_from_now,
              yield_entry_s,
              remain_s,
              yield_initial_speed,
              yield_speed_cap,
              smooth_yield_deceleration_);
        }
      }

      if (!yield_by_time)
      {
        // 若为了满足目标进入时间需要极低速度爬行，则改为在 stop_s 前停车等待。
        stop_by_constraint = std::isfinite(stop_s);
        changed = changed || stop_by_constraint;
        if (stop_by_constraint)
        {
          ROS_WARN_THROTTLE(
              1.0,
              "[Conflict Velocity Decision] ego=%s peer=%s decision=STOP_AND_YIELD. "
              "Comfortable deceleration cannot satisfy the target time or the required "
              "cruise speed is too low. "
              "current_entry_time=%.2fs target_entry_time=%.2fs "
              "entry_s=%.2fm stop_s=%.2fm.",
              constraint.ego_id.c_str(),
              constraint.peer_id.c_str(),
              current_entry_time,
              target_entry_time_from_now,
              yield_entry_s,
              stop_s);
        }
      }
    }
  }

  if (!changed)
  {
    ROS_INFO_THROTTLE(
        1.0,
        "[Conflict Velocity Decision] ego=%s peer=%s decision=YIELD, but the current "
        "candidate trajectory already satisfies the entry-time constraint; no velocity "
        "change is required. target_entry_time=%.2fs entry_s=%.2fm.",
        constraint.ego_id.c_str(),
        constraint.peer_id.c_str(),
        target_entry_time_from_now,
        yield_entry_s);
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
      const double smooth_yield_cap = comfortDecelSpeedCapAtS(
          point.s,
          yield_start_s,
          yield_initial_speed,
          yield_speed_cap,
          smooth_yield_deceleration_);
      target_v = std::min(target_v, smooth_yield_cap);
    }

    if (stop_by_constraint)
    {
      // 一旦进入停车让行，不能再让原候选轨迹首段速度反向抬升。
      // 场景 2 中曾出现过这样的现象：前一帧已经降到约 1.2m/s，下一帧因为切换到
      // 停车轨迹，首段又沿用候选轨迹 1.4~1.5m/s，车辆表现为“先加速再停车”。
      // 因此停车让行先套一层保守速度上限，再叠加到 stop_s 的制动包络。
      target_v = std::min(target_v, std::max(0.0, timeout_max_speed_));

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
    if (stop_by_constraint)
    {
      truncateTrajectoryAtStopS(trajectory, stitching_end_index, stop_s);
      recomputeTimingFrom(trajectory, stitching_end_index);
      extendStopPointTimeHorizon(trajectory, stitching_end_index, kMinStopTimeHorizon);
      enforceStrictlyIncreasingRelativeTime(trajectory, kFinalTimeDtFloor);

      std::string stop_reject_reason;
      if (validateTrajectoryForController(trajectory, mutable_start_index, &stop_reject_reason))
      {
        // 已经明确进入停车让行时，规则轨迹就是安全结果。这里不再送入 QP，
        // 避免定时间优化把零速等待尾迹压成一串重复 s，再被截断成过短轨迹。
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
                          "[Conflict Velocity Decision] ego=%s peer=%s "
                          "output=EMERGENCY_STOP_TRAJECTORY. Rule-based stop trajectory "
                          "validation failed. reason=%s stop_s=%.2f",
                          constraint.ego_id.c_str(),
                          constraint.peer_id.c_str(),
                          stop_reject_reason.c_str(),
                          stop_s);
        trajectory = emergency_stop_trajectory;
        return;
      }

      ROS_WARN_THROTTLE(1.0,
                        "[Conflict Velocity Decision] ego=%s peer=%s "
                        "output=ORIGINAL_PLANNER_TRAJECTORY. Both rule-based and emergency "
                        "stop trajectories failed validation. reason=%s "
                        "emergency_reason=%s stop_s=%.2f",
                        constraint.ego_id.c_str(),
                        constraint.peer_id.c_str(),
                        stop_reject_reason.c_str(),
                        emergency_reject_reason.c_str(),
                        stop_s);
      trajectory = original_trajectory;
      return;
    }

    recomputeTimingFrom(trajectory, stitching_end_index);
    resampleNonStitchedSuffixToFixedTime(trajectory,
                                         stitching_end_index,
                                         fixed_time_coarse_dt_,
                                         fixed_time_max_points_);
    const bool qp_success = velocity_optimizer_.optimize(trajectory,
                                                         stitching_end_index,
                                                         speed_cap,
                                                         std::isfinite(yield_entry_s),
                                                         yield_entry_s,
                                                         target_entry_time_from_now);
    if (!qp_success)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[Conflict Velocity Decision] ego=%s peer=%s QP optimization failed; "
          "retain the rule-based smooth-yield profile and continue safety validation.",
          constraint.ego_id.c_str(),
          constraint.peer_id.c_str());
    }

    double checked_entry_time = std::numeric_limits<double>::infinity();
    const bool entry_time_satisfied = velocity_optimizer_.satisfiesEntryTimeConstraint(
        trajectory, yield_entry_s, target_entry_time_from_now, &checked_entry_time);
    if (!entry_time_satisfied)
    {
      // 入口时间验收由 ConflictVelocityOptimizer 统一定义。
      // 若规则粗解或 QP 结果仍会过早进入冲突区，说明当前速度消解结果不可信，
      // 直接切换到高减速度停车兜底，不再额外叠加一层低速 fallback。
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
                          "[Conflict Velocity Decision] ego=%s peer=%s "
                          "output=EMERGENCY_STOP_TRAJECTORY. Velocity profile violates the "
                          "entry-time constraint. qp_success=%d "
                          "entry_time=%.2f target=%.2f stop_s=%.2f",
                          constraint.ego_id.c_str(),
                          constraint.peer_id.c_str(),
                          qp_success,
                          checked_entry_time,
                          target_entry_time_from_now,
                          stop_s);
        trajectory = emergency_stop_trajectory;
        return;
      }

      ROS_WARN_THROTTLE(1.0,
                        "conflict emergency stop fallback invalid after entry-time violation. "
                        "reason=%s entry_time=%.2f target=%.2f",
                        emergency_reject_reason.c_str(),
                        checked_entry_time,
                        target_entry_time_from_now);
    }

    enforceStrictlyIncreasingRelativeTime(trajectory, kFinalTimeDtFloor);
    std::string reject_reason;
    if (!validateTrajectoryForController(trajectory, mutable_start_index, &reject_reason))
    {
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
                          "[Conflict Velocity Decision] ego=%s peer=%s "
                          "output=EMERGENCY_STOP_TRAJECTORY. QP/rule-based velocity profile "
                          "failed controller-input validation. reason=%s stop_s=%.2f",
                          constraint.ego_id.c_str(),
                          constraint.peer_id.c_str(),
                          reject_reason.c_str(),
                          stop_s);
        trajectory = emergency_stop_trajectory;
        return;
      }

      ROS_WARN_THROTTLE(1.0,
                        "conflict velocity result rejected before publish, "
                        "emergency stop fallback also invalid, restore original planner trajectory "
                        "as unavoidable last resort. reason=%s emergency_reason=%s "
                        "points=%zu stitching_end=%zu stop_by_constraint=%d "
                        "yield_by_time=%d stop_s=%.2f yield_entry_s=%.2f "
                        "target_entry_time_from_now=%.2f",
                        reject_reason.c_str(),
                        emergency_reject_reason.c_str(),
                        trajectory.points.size(),
                        stitching_end_index,
                        stop_by_constraint,
                        yield_by_time,
                        stop_s,
                        yield_entry_s,
                        target_entry_time_from_now);
      trajectory = original_trajectory;
      return;
    }

    ROS_INFO_THROTTLE(
        1.0,
        "[Conflict Velocity Decision] ego=%s peer=%s output=%s. "
        "entry_time=%.2fs target_entry_time=%.2fs entry_s=%.2fm "
        "points=%zu last_time=%.2fs.",
        constraint.ego_id.c_str(),
        constraint.peer_id.c_str(),
        constraint_timeout
            ? (qp_success ? "QP_TIMEOUT_CONSERVATIVE_TRAJECTORY"
                          : "RULE_BASED_TIMEOUT_CONSERVATIVE_TRAJECTORY")
            : (qp_success ? "QP_YIELD_TRAJECTORY"
                          : "RULE_BASED_SMOOTH_YIELD_TRAJECTORY"),
        checked_entry_time,
        target_entry_time_from_now,
        yield_entry_s,
        trajectory.points.size(),
        trajectory.points.back().relative_time);
  }
}

}  // namespace conflict_prediction_resolution
