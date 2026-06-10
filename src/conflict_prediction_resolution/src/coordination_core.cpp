#include "conflict_prediction_resolution/coordination_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

namespace conflict_prediction_resolution
{
namespace coordination
{
namespace
{

constexpr double kEpsilon = 1.0e-6;

struct Axis2d
{
  double x = 0.0;
  double y = 0.0;
};

struct ProjectionRange
{
  double min = 0.0;
  double max = 0.0;
};

using Corners = std::array<Pose2d, 4>;

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

double distance2d(const double ax, const double ay, const double bx, const double by)
{
  return std::hypot(ax - bx, ay - by);
}

Corners makeCorners(const Pose2d& pose, const double length, const double width)
{
  // 将车辆矩形从车体坐标系旋转/平移到地图坐标系。
  const double half_length = 0.5 * length;
  const double half_width = 0.5 * width;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const double local_x[4] = {half_length, half_length, -half_length, -half_length};
  const double local_y[4] = {half_width, -half_width, -half_width, half_width};

  Corners corners;
  for (int i = 0; i < 4; ++i)
  {
    corners[static_cast<size_t>(i)].x = pose.x + local_x[i] * cos_yaw - local_y[i] * sin_yaw;
    corners[static_cast<size_t>(i)].y = pose.y + local_x[i] * sin_yaw + local_y[i] * cos_yaw;
  }
  return corners;
}

ProjectionRange project(const Corners& corners, const Axis2d& axis)
{
  // 分离轴定理：把矩形角点投影到候选轴上，之后比较投影区间是否重叠。
  ProjectionRange range;
  range.min = corners.front().x * axis.x + corners.front().y * axis.y;
  range.max = range.min;
  for (const auto& corner : corners)
  {
    const double value = corner.x * axis.x + corner.y * axis.y;
    range.min = std::min(range.min, value);
    range.max = std::max(range.max, value);
  }
  return range;
}

bool rangesOverlap(const ProjectionRange& first, const ProjectionRange& second)
{
  return first.max >= second.min && second.max >= first.min;
}

double pointSegmentDistance(const TrajectoryPoint& start,
                            const TrajectoryPoint& end,
                            const Pose2d& target,
                            double& ratio)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq < kEpsilon)
  {
    ratio = 0.0;
    return distance2d(start.x, start.y, target.x, target.y);
  }

  ratio = clamp(((target.x - start.x) * dx + (target.y - start.y) * dy) / length_sq, 0.0, 1.0);
  return distance2d(start.x + ratio * dx, start.y + ratio * dy, target.x, target.y);
}

double conflictEntrySFor(const PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_s_in;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_s_in;
  }
  return 0.0;
}

double conflictExitSFor(const PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_s_out;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_s_out;
  }
  return 0.0;
}

}  // namespace

MultiVehicleCoordinator::MultiVehicleCoordinator(CoordinatorConfig config) : config_(config) {}

CoordinationResult MultiVehicleCoordinator::resolve(const std::vector<VehicleAgent>& agents,
                                                    const int ego_index)
{
  CoordinationResult result;
  const bool centralized_mode = ego_index < 0;
  const bool ego_index_valid = !centralized_mode && static_cast<size_t>(ego_index) < agents.size();
  const auto agent_ready = [](const VehicleAgent& agent) {
    return agent.have_pose && agent.have_trajectory && !agent.trajectory.points.empty();
  };

  if (centralized_mode)
  {
    // 集中式模式需要全局状态完整，才能进行所有车辆对的统一排序。
    for (const auto& agent : agents)
    {
      if (!agent_ready(agent))
      {
        decision_locks_.clear();
        result.status = "WAITING_FOR_TRAJECTORY_OR_POSE";
        return result;
      }
    }
  }
  else if (!ego_index_valid)
  {
    decision_locks_.clear();
    result.status = "CLEAR_EGO_ST_OCCUPANCY: ego_index outside active vehicle_count";
    return result;
  }
  else if (!agent_ready(agents[static_cast<size_t>(ego_index)]))
  {
    decision_locks_.clear();
    result.status = "WAITING_FOR_EGO_TRAJECTORY_OR_POSE";
    return result;
  }

  result.ready = true;

  std::vector<bool> already_yielding(agents.size(), false);
  std::vector<std::pair<int, int>> active_pairs;
  for (size_t i = 0; i < agents.size(); ++i)
  {
    for (size_t j = i + 1; j < agents.size(); ++j)
    {
      if (ego_index_valid && i != static_cast<size_t>(ego_index) && j != static_cast<size_t>(ego_index))
      {
        continue;
      }
      if (!agent_ready(agents[i]) || !agent_ready(agents[j]))
      {
        continue;
      }

      PairConflict conflict =
          detectPairConflict(agents[i], agents[j], static_cast<int>(i), static_cast<int>(j));
      if (!conflict.active)
      {
        continue;
      }

      active_pairs.push_back(pairKey(conflict));
      chooseOrder(agents, already_yielding, conflict);
      result.conflicts.push_back(conflict);
      result.conflict_active = true;
      // already_yielding 是集中式全局排序约束；分布式 ego-only 模式下不记录，
      // 否则不同车辆的 resolver 会因本地历史不同而对同一 pair 给出相反决策。
      if (centralized_mode && conflict.yield_index >= 0)
      {
        already_yielding[static_cast<size_t>(conflict.yield_index)] = true;
      }
    }
  }
  clearInactiveDecisionLocks(active_pairs);

  if (!result.conflict_active)
  {
    result.status = "CLEAR_ST_OCCUPANCY: no predicted future footprint overlap";
    return result;
  }

  std::ostringstream status;
  status << "ST_CONFLICT count=" << result.conflicts.size();
  for (const auto& conflict : result.conflicts)
  {
    status << " [" << agents[static_cast<size_t>(conflict.first_index)].id << ","
           << agents[static_cast<size_t>(conflict.second_index)].id << " t="
           << conflict.conflict_time << " clear="
           << conflict.conflict_clear_time << " first_window=["
           << conflict.first_t_in << "," << conflict.first_t_out << "] second_window=["
           << conflict.second_t_in << "," << conflict.second_t_out << "] proceed="
           << agents[static_cast<size_t>(conflict.proceed_index)].id << " yield="
           << agents[static_cast<size_t>(conflict.yield_index)].id << " decision="
           << conflict.decision_source << " reason=" << conflict.decision_reason
           << " score=[" << conflict.first_score << "," << conflict.second_score << "]";
    if (conflict.decision_locked)
    {
      status << " locked";
    }
    status << "]";
  }
  result.status = status.str();
  return result;
}

PairConflict MultiVehicleCoordinator::detectPairConflict(const VehicleAgent& first,
                                                         const VehicleAgent& second,
                                                         const int first_index,
                                                         const int second_index) const
{
  PairConflict conflict;
  conflict.first_index = first_index;
  conflict.second_index = second_index;

  const double first_progress = estimateProgress(first.trajectory, first.pose);
  const double second_progress = estimateProgress(second.trajectory, second.pose);
  const double first_speed = plannedSpeedAtProgress(first.trajectory, first_progress, first.nominal_speed);
  const double second_speed = plannedSpeedAtProgress(second.trajectory, second_progress, second.nominal_speed);
  const double first_horizon_s =
      reachableSAtHorizon(first.trajectory, first_progress, config_.prediction_horizon, first.nominal_speed);
  const double second_horizon_s =
      reachableSAtHorizon(second.trajectory, second_progress, config_.prediction_horizon, second.nominal_speed);
  const std::vector<double> first_samples = sampledSRange(first.trajectory, first_progress, first_horizon_s);
  const std::vector<double> second_samples = sampledSRange(second.trajectory, second_progress, second_horizon_s);

  bool found_spatial_overlap = false;
  double first_s_min = std::numeric_limits<double>::infinity();
  double first_s_max = -std::numeric_limits<double>::infinity();
  double second_s_min = std::numeric_limits<double>::infinity();
  double second_s_max = -std::numeric_limits<double>::infinity();
  Pose2d first_collision_pose;
  Pose2d second_collision_pose;
  double unused_speed = 0.0;

  // 先在预测时域内可达的 s 区间上扫描空间 footprint 重叠区，再做时间窗判定。
  for (const double first_s : first_samples)
  {
    Pose2d first_pose;
    if (!sampleByS(first.trajectory, first_s, first_pose, unused_speed))
    {
      continue;
    }
    for (const double second_s : second_samples)
    {
      Pose2d second_pose;
      if (!sampleByS(second.trajectory, second_s, second_pose, unused_speed))
      {
        continue;
      }

      if (!footprintsOverlap(first_pose, first.length, first.width, second_pose, second.length, second.width))
      {
        continue;
      }

      if (!found_spatial_overlap)
      {
        first_collision_pose = first_pose;
        second_collision_pose = second_pose;
      }
      found_spatial_overlap = true;
      first_s_min = std::min(first_s_min, first_s);
      first_s_max = std::max(first_s_max, first_s);
      second_s_min = std::min(second_s_min, second_s);
      second_s_max = std::max(second_s_max, second_s);
    }
  }

  if (!found_spatial_overlap)
  {
    return conflict;
  }

  const double first_t_in = elapsedTimeBetweenS(first.trajectory, first_progress, first_s_min, first_speed);
  const double first_t_out = elapsedTimeBetweenS(first.trajectory, first_progress, first_s_max, first_speed);
  const double second_t_in = elapsedTimeBetweenS(second.trajectory, second_progress, second_s_min, second_speed);
  const double second_t_out = elapsedTimeBetweenS(second.trajectory, second_progress, second_s_max, second_speed);
  const bool time_windows_conflict =
      first_t_in <= second_t_out + config_.conflict_time_clearance &&
      second_t_in <= first_t_out + config_.conflict_time_clearance;
  if (!time_windows_conflict)
  {
    return conflict;
  }

  conflict.active = true;
  conflict.conflict_time = std::max(first_t_in, second_t_in);
  conflict.conflict_clear_time = std::min(first_t_out, second_t_out);
  conflict.collision_point.x = 0.5 * (first_collision_pose.x + second_collision_pose.x);
  conflict.collision_point.y = 0.5 * (first_collision_pose.y + second_collision_pose.y);
  conflict.collision_point.yaw = first_collision_pose.yaw;
  conflict.first_s_in = first_s_min;
  conflict.first_s_out = first_s_max;
  conflict.second_s_in = second_s_min;
  conflict.second_s_out = second_s_max;
  conflict.first_t_in = first_t_in;
  conflict.first_t_out = first_t_out;
  conflict.second_t_in = second_t_in;
  conflict.second_t_out = second_t_out;
  sampleByS(first.trajectory, conflict.first_s_in, conflict.first_entry_pose, unused_speed);
  sampleByS(first.trajectory, conflict.first_s_out, conflict.first_exit_pose, unused_speed);
  sampleByS(second.trajectory, conflict.second_s_in, conflict.second_entry_pose, unused_speed);
  sampleByS(second.trajectory, conflict.second_s_out, conflict.second_exit_pose, unused_speed);
  return conflict;
}

void MultiVehicleCoordinator::chooseOrder(const std::vector<VehicleAgent>& agents,
                                          const std::vector<bool>& already_yielding,
                                          PairConflict& conflict)
{
  const auto key = pairKey(conflict);
  const auto locked = decision_locks_.find(key);
  bool released_lock_for_recompute = false;
  if (locked != decision_locks_.end())
  {
    const auto now = std::chrono::steady_clock::now();
    const double lock_age =
        std::chrono::duration<double>(now - locked->second.created_at).count();
    const auto far_from_entry = [&](const VehicleAgent& agent, const int vehicle_index) {
      const double current_s = estimateProgress(agent.trajectory, agent.pose);
      const double distance_to_entry = std::max(0.0, conflictEntrySFor(conflict, vehicle_index) - current_s);
      return distance_to_entry >= config_.decision_unlock_distance;
    };
    const bool hold_time_elapsed = lock_age >= config_.minimum_lock_hold_time;
    const bool both_far_from_entry =
        far_from_entry(agents[static_cast<size_t>(conflict.first_index)], conflict.first_index) &&
        far_from_entry(agents[static_cast<size_t>(conflict.second_index)], conflict.second_index);
    if (!hold_time_elapsed || !both_far_from_entry)
    {
      // 锁定后至少保持 minimum_lock_hold_time，并且需要离冲突入口足够远才允许重算。
      conflict.proceed_index = locked->second.proceed_index;
      conflict.yield_index = locked->second.yield_index;
      conflict.decision_locked = true;
      conflict.decision_source = "LOCK";
      conflict.decision_reason = "keep_locked_order";
      return;
    }

    decision_locks_.erase(locked);
    released_lock_for_recompute = true;
  }

  // 冲突区已很近时，在本轮普通决策后写入锁，避免后续实时重评分反复切换。
  const bool should_lock = shouldLockDecision(agents, conflict);

  // 尚未锁定时，决策分数综合优先级、当前速度和路线进度；分数接近时用 priority 兜底。
  const VehicleAgent& first = agents[static_cast<size_t>(conflict.first_index)];
  const VehicleAgent& second = agents[static_cast<size_t>(conflict.second_index)];
  const bool first_is_blocked =
      static_cast<size_t>(conflict.first_index) < already_yielding.size() &&
      already_yielding[static_cast<size_t>(conflict.first_index)];
  const bool second_is_blocked =
      static_cast<size_t>(conflict.second_index) < already_yielding.size() &&
      already_yielding[static_cast<size_t>(conflict.second_index)];

  const bool first_inside_conflict = isVehicleInsideConflictInterval(first, conflict, conflict.first_index);
  const bool second_inside_conflict = isVehicleInsideConflictInterval(second, conflict, conflict.second_index);
  if (first_inside_conflict != second_inside_conflict)
  {
    // 已经进入本 pair 冲突区的车辆不再被要求停车避让，否则容易在共享空间内停住。
    const bool first_goes = first_inside_conflict;
    conflict.proceed_index = first_goes ? conflict.first_index : conflict.second_index;
    conflict.yield_index = first_goes ? conflict.second_index : conflict.first_index;
    conflict.decision_source = "RULE";
    conflict.decision_reason = "inside_conflict_interval";
    if (should_lock)
    {
      decision_locks_[key] = LockedDecision{conflict.proceed_index, conflict.yield_index};
      conflict.decision_locked = true;
    }
    return;
  }

  const bool first_cannot_stop = cannotStopBeforeConflictEntry(first, conflict, conflict.first_index);
  const bool second_cannot_stop = cannotStopBeforeConflictEntry(second, conflict, conflict.second_index);
  if (first_cannot_stop != second_cannot_stop)
  {
    // 距离 s_in 太近或速度太高时，临时让该车避让反而不安全，因此让它先通过。
    const bool first_goes = first_cannot_stop;
    conflict.proceed_index = first_goes ? conflict.first_index : conflict.second_index;
    conflict.yield_index = first_goes ? conflict.second_index : conflict.first_index;
    conflict.decision_source = "RULE";
    conflict.decision_reason = "cannot_stop_before_s_in";
    if (should_lock)
    {
      decision_locks_[key] = LockedDecision{conflict.proceed_index, conflict.yield_index};
      conflict.decision_locked = true;
    }
    return;
  }

  if (first_is_blocked != second_is_blocked)
  {
    // 已经在别的冲突中让行的车辆，不再继续要求后续车辆给它让路，避免链式等待。
    const bool first_goes = second_is_blocked;
    conflict.proceed_index = first_goes ? conflict.first_index : conflict.second_index;
    conflict.yield_index = first_goes ? conflict.second_index : conflict.first_index;
    conflict.decision_source = "RULE";
    conflict.decision_reason = "already_yielding_in_central_order";
    if (should_lock)
    {
      decision_locks_[key] = LockedDecision{conflict.proceed_index, conflict.yield_index};
      conflict.decision_locked = true;
    }
    return;
  }

  const double first_progress = estimateProgress(first.trajectory, first.pose);
  const double second_progress = estimateProgress(second.trajectory, second.pose);
  const double first_yield_delay =
      std::max(0.0, conflict.second_t_out + config_.conflict_time_clearance - conflict.first_t_in);
  const double second_yield_delay =
      std::max(0.0, conflict.first_t_out + config_.conflict_time_clearance - conflict.second_t_in);
  const double first_score =
      decisionScore(first,
                    first_progress,
                    trajectoryLength(first.trajectory),
                    second.priority,
                    conflict.first_t_in,
                    first_yield_delay);
  const double second_score =
      decisionScore(second,
                    second_progress,
                    trajectoryLength(second.trajectory),
                    first.priority,
                    conflict.second_t_in,
                    second_yield_delay);
  conflict.first_score = first_score;
  conflict.second_score = second_score;

  const double effective_switch_margin =
      released_lock_for_recompute ? std::max(config_.score_tie_epsilon, config_.decision_switch_margin)
                                  : config_.score_tie_epsilon;
  bool first_goes = first_score > second_score + effective_switch_margin;
  if (std::abs(first_score - second_score) <= effective_switch_margin)
  {
    first_goes = first.priority <= second.priority;
    conflict.decision_source = "PRIORITY";
    conflict.decision_reason = released_lock_for_recompute
                                   ? "released_lock_score_margin_tie"
                                   : "score_tie_priority_fallback";
  }
  else
  {
    conflict.decision_source = "SCORE";
    conflict.decision_reason = "weighted_priority_speed_progress_ttc_delay";
  }

  conflict.proceed_index = first_goes ? conflict.first_index : conflict.second_index;
  conflict.yield_index = first_goes ? conflict.second_index : conflict.first_index;
  if (should_lock)
  {
    decision_locks_[key] = LockedDecision{conflict.proceed_index, conflict.yield_index};
    conflict.decision_locked = true;
  }
}

double MultiVehicleCoordinator::estimateProgress(const Trajectory& trajectory, const Pose2d& pose) const
{
  if (trajectory.points.empty())
  {
    return 0.0;
  }

  double best_s = trajectory.points.front().s;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    double ratio = 0.0;
    const double distance = pointSegmentDistance(trajectory.points[i - 1], trajectory.points[i], pose, ratio);
    if (distance >= best_distance)
    {
      continue;
    }

    best_distance = distance;
    best_s = trajectory.points[i - 1].s +
             ratio * (trajectory.points[i].s - trajectory.points[i - 1].s);
  }
  return best_s;
}

double MultiVehicleCoordinator::trajectoryLength(const Trajectory& trajectory) const
{
  return trajectory.points.empty() ? 0.0 : trajectory.points.back().s;
}

double MultiVehicleCoordinator::plannedSpeedAtProgress(const Trajectory& trajectory,
                                                       const double progress_s,
                                                       const double fallback_speed) const
{
  Pose2d unused_pose;
  double speed = fallback_speed;
  if (!sampleByS(trajectory, progress_s, unused_pose, speed))
  {
    return std::max(0.0, fallback_speed);
  }
  return std::max(0.0, speed);
}

bool MultiVehicleCoordinator::sampleByS(const Trajectory& trajectory,
                                        const double s,
                                        Pose2d& pose,
                                        double& speed) const
{
  if (trajectory.points.empty())
  {
    return false;
  }

  if (s <= trajectory.points.front().s)
  {
    const auto& point = trajectory.points.front();
    pose = Pose2d{point.x, point.y, point.yaw};
    speed = std::max(0.0, point.v);
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

    // 轨迹点按弧长 s 插值，输出预测时刻对应的位姿和速度。
    const double ds = std::max(kEpsilon, next.s - prev.s);
    const double ratio = clamp((s - prev.s) / ds, 0.0, 1.0);
    pose.x = prev.x + (next.x - prev.x) * ratio;
    pose.y = prev.y + (next.y - prev.y) * ratio;
    pose.yaw = prev.yaw + (next.yaw - prev.yaw) * ratio;
    speed = std::max(0.0, prev.v + (next.v - prev.v) * ratio);
    return true;
  }

  const auto& point = trajectory.points.back();
  pose = Pose2d{point.x, point.y, point.yaw};
  speed = 0.0;
  return true;
}

double MultiVehicleCoordinator::elapsedTimeBetweenS(const Trajectory& trajectory,
                                                    const double start_s,
                                                    const double end_s,
                                                    const double fallback_speed) const
{
  if (trajectory.points.empty() || end_s <= start_s + kEpsilon)
  {
    return 0.0;
  }

  const auto relative_time_at_s = [&](const double query_s, double& relative_time) {
    if (trajectory.points.empty())
    {
      return false;
    }
    bool monotonic_time = true;
    for (size_t i = 1; i < trajectory.points.size(); ++i)
    {
      if (trajectory.points[i].relative_time + kEpsilon < trajectory.points[i - 1].relative_time)
      {
        monotonic_time = false;
        break;
      }
    }
    if (!monotonic_time)
    {
      return false;
    }
    if (trajectory.points.back().relative_time <= trajectory.points.front().relative_time + kEpsilon)
    {
      return false;
    }

    if (query_s <= trajectory.points.front().s)
    {
      relative_time = trajectory.points.front().relative_time;
      return true;
    }
    for (size_t i = 1; i < trajectory.points.size(); ++i)
    {
      const auto& prev = trajectory.points[i - 1];
      const auto& next = trajectory.points[i];
      if (query_s > next.s)
      {
        continue;
      }
      const double ds = std::max(kEpsilon, next.s - prev.s);
      const double ratio = clamp((query_s - prev.s) / ds, 0.0, 1.0);
      relative_time = prev.relative_time + ratio * (next.relative_time - prev.relative_time);
      return true;
    }

    relative_time = trajectory.points.back().relative_time;
    return true;
  };

  double start_time = 0.0;
  double end_time = 0.0;
  if (relative_time_at_s(start_s, start_time) && relative_time_at_s(end_s, end_time) &&
      end_time + kEpsilon >= start_time)
  {
    return end_time - start_time;
  }

  double elapsed = 0.0;
  double segment_start_s = start_s;
  Pose2d unused_pose;
  double previous_speed = std::max(0.0, fallback_speed);
  sampleByS(trajectory, segment_start_s, unused_pose, previous_speed);

  for (size_t i = 1; i < trajectory.points.size() && segment_start_s < end_s - kEpsilon; ++i)
  {
    const double segment_end_s = std::min(end_s, trajectory.points[i].s);
    if (segment_end_s <= segment_start_s + kEpsilon)
    {
      continue;
    }

    double next_speed = std::max(0.0, trajectory.points[i].v);
    if (segment_end_s < trajectory.points[i].s - kEpsilon)
    {
      sampleByS(trajectory, segment_end_s, unused_pose, next_speed);
    }
    const double average_speed = std::max(0.1, 0.5 * (previous_speed + next_speed));
    elapsed += (segment_end_s - segment_start_s) / average_speed;
    segment_start_s = segment_end_s;
    previous_speed = next_speed;
  }

  if (segment_start_s < end_s - kEpsilon)
  {
    elapsed += (end_s - segment_start_s) / std::max(0.1, fallback_speed);
  }
  return elapsed;
}

double MultiVehicleCoordinator::reachableSAtHorizon(const Trajectory& trajectory,
                                                    const double current_s,
                                                    const double horizon,
                                                    const double fallback_speed) const
{
  const double route_end_s = trajectoryLength(trajectory);
  if (trajectory.points.empty() || current_s >= route_end_s || horizon <= kEpsilon)
  {
    return std::max(current_s, route_end_s);
  }

  if (elapsedTimeBetweenS(trajectory, current_s, route_end_s, fallback_speed) <= horizon)
  {
    return route_end_s;
  }

  double lower_s = current_s;
  double upper_s = route_end_s;
  for (int iter = 0; iter < 40; ++iter)
  {
    const double mid_s = 0.5 * (lower_s + upper_s);
    if (elapsedTimeBetweenS(trajectory, current_s, mid_s, fallback_speed) <= horizon)
    {
      lower_s = mid_s;
      continue;
    }
    upper_s = mid_s;
  }
  return lower_s;
}

std::vector<double> MultiVehicleCoordinator::sampledSRange(const Trajectory& trajectory,
                                                           const double start_s,
                                                           const double end_s) const
{
  std::vector<double> samples;
  if (trajectory.points.empty())
  {
    return samples;
  }

  const double lower_s = std::max(trajectory.points.front().s, std::min(start_s, end_s));
  const double upper_s = std::min(trajectory.points.back().s, std::max(start_s, end_s));
  samples.push_back(lower_s);
  for (const auto& point : trajectory.points)
  {
    if (point.s <= lower_s + kEpsilon || point.s >= upper_s - kEpsilon)
    {
      continue;
    }
    samples.push_back(point.s);
  }
  if (upper_s > lower_s + kEpsilon)
  {
    samples.push_back(upper_s);
  }
  return samples;
}

bool MultiVehicleCoordinator::footprintsOverlap(const Pose2d& first_pose,
                                                const double first_length,
                                                const double first_width,
                                                const Pose2d& second_pose,
                                                const double second_length,
                                                const double second_width) const
{
  const double first_safe_length = first_length + 2.0 * config_.footprint_safety_margin;
  const double first_safe_width = first_width + 2.0 * config_.footprint_safety_margin;
  const double second_safe_length = second_length + 2.0 * config_.footprint_safety_margin;
  const double second_safe_width = second_width + 2.0 * config_.footprint_safety_margin;
  const Corners first_corners = makeCorners(first_pose, first_safe_length, first_safe_width);
  const Corners second_corners = makeCorners(second_pose, second_safe_length, second_safe_width);
  // 使用两车自身的纵向/横向轴作为分离轴，适用于任意朝向的矩形 footprint。
  const std::array<Axis2d, 4> axes = {{{std::cos(first_pose.yaw), std::sin(first_pose.yaw)},
                                       {-std::sin(first_pose.yaw), std::cos(first_pose.yaw)},
                                       {std::cos(second_pose.yaw), std::sin(second_pose.yaw)},
                                       {-std::sin(second_pose.yaw), std::cos(second_pose.yaw)}}};

  for (const auto& axis : axes)
  {
    if (!rangesOverlap(project(first_corners, axis), project(second_corners, axis)))
    {
      return false;
    }
  }
  return true;
}

double MultiVehicleCoordinator::decisionScore(const VehicleAgent& agent,
                                              const double progress_s,
                                              const double route_length,
                                              const int other_priority,
                                              const double ttc,
                                              const double yield_delay) const
{
  const double priority_component = agent.priority <= other_priority ? 1.0 : 0.0;
  const double speed_component =
      clamp(agent.speed / std::max(0.1, agent.nominal_speed), 0.0, 1.5);
  const double progress_component =
      route_length > kEpsilon ? clamp(progress_s / route_length, 0.0, 1.0) : 0.0;
  // TTC 越小表示越接近冲突入口，越不适合临时让行；让行延误越大也越倾向先行。
  const double ttc_component =
      1.0 - clamp(ttc / std::max(0.1, config_.prediction_horizon), 0.0, 1.0);
  const double yield_delay_component =
      clamp(yield_delay / std::max(0.1, config_.prediction_horizon), 0.0, 1.0);
  return config_.priority_weight * priority_component +
         config_.speed_weight * speed_component +
         config_.progress_weight * progress_component +
         config_.ttc_weight * ttc_component +
         config_.yield_delay_weight * yield_delay_component;
}

bool MultiVehicleCoordinator::isVehicleInsideConflictInterval(const VehicleAgent& agent,
                                                              const PairConflict& conflict,
                                                              const int vehicle_index) const
{
  const double current_s = estimateProgress(agent.trajectory, agent.pose);
  const double s_in = conflictEntrySFor(conflict, vehicle_index);
  const double s_out = conflictExitSFor(conflict, vehicle_index);
  return current_s >= s_in - kEpsilon && current_s <= s_out + kEpsilon;
}

bool MultiVehicleCoordinator::cannotStopBeforeConflictEntry(const VehicleAgent& agent,
                                                            const PairConflict& conflict,
                                                            const int vehicle_index) const
{
  const double current_s = estimateProgress(agent.trajectory, agent.pose);
  const double entry_s = conflictEntrySFor(conflict, vehicle_index);
  const double distance_to_entry = entry_s - current_s;
  if (distance_to_entry <= 0.0)
  {
    return true;
  }

  const double fallback_speed = plannedSpeedAtProgress(agent.trajectory, current_s, agent.nominal_speed);
  const double speed = agent.have_speed ? agent.speed : fallback_speed;
  const double braking_distance =
      speed * speed / (2.0 * std::max(0.1, config_.comfortable_deceleration));
  return braking_distance + config_.stop_margin >= distance_to_entry;
}

bool MultiVehicleCoordinator::shouldLockDecision(const std::vector<VehicleAgent>& agents,
                                                 const PairConflict& conflict) const
{
  if (!config_.enable_decision_lock ||
      conflict.first_index < 0 ||
      conflict.second_index < 0 ||
      static_cast<size_t>(conflict.first_index) >= agents.size() ||
      static_cast<size_t>(conflict.second_index) >= agents.size())
  {
    return false;
  }

  return isVehicleNearConflictEntry(agents[static_cast<size_t>(conflict.first_index)],
                                    conflict,
                                    conflict.first_index) ||
         isVehicleNearConflictEntry(agents[static_cast<size_t>(conflict.second_index)],
                                    conflict,
                                    conflict.second_index);
}

bool MultiVehicleCoordinator::isVehicleNearConflictEntry(const VehicleAgent& agent,
                                                         const PairConflict& conflict,
                                                         const int vehicle_index) const
{
  const double current_s = estimateProgress(agent.trajectory, agent.pose);
  const double entry_s = conflictEntrySFor(conflict, vehicle_index);
  const double distance_to_entry = std::max(0.0, entry_s - current_s);
  if (distance_to_entry <= config_.decision_lock_distance)
  {
    return true;
  }

  const double fallback_speed = plannedSpeedAtProgress(agent.trajectory, current_s, agent.nominal_speed);
  const double speed = agent.have_speed ? agent.speed : fallback_speed;
  if (speed < 0.1)
  {
    return false;
  }

  const double ttc = distance_to_entry / speed;
  return ttc <= config_.decision_lock_ttc;
}

std::pair<int, int> MultiVehicleCoordinator::pairKey(const PairConflict& conflict) const
{
  return std::make_pair(std::min(conflict.first_index, conflict.second_index),
                        std::max(conflict.first_index, conflict.second_index));
}

void MultiVehicleCoordinator::clearInactiveDecisionLocks(const std::vector<std::pair<int, int>>& active_pairs)
{
  std::set<std::pair<int, int>> active_set(active_pairs.begin(), active_pairs.end());
  for (auto it = decision_locks_.begin(); it != decision_locks_.end();)
  {
    if (active_set.find(it->first) == active_set.end())
    {
      it = decision_locks_.erase(it);
      continue;
    }
    ++it;
  }
}

}  // namespace coordination
}  // namespace conflict_prediction_resolution
