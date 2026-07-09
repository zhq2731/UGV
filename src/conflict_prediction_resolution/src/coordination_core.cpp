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

struct OverlapAngleSample
{
  double first_s = 0.0;
  double second_s = 0.0;
  double angle_deg = 0.0;
};

struct SRange
{
  bool valid = false;
  size_t start = 0;
  size_t end = 0;
};

struct ClassifiedConflictSection
{
  bool active = false;
  double first_s_in = 0.0;
  double first_s_out = 0.0;
  double second_s_in = 0.0;
  double second_s_out = 0.0;
  std::string type;
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

double normalizeAngle(const double angle)
{
  const double pi = std::acos(-1.0);
  double normalized = std::fmod(angle + pi, 2.0 * pi);
  if (normalized < 0.0)
  {
    normalized += 2.0 * pi;
  }
  return normalized - pi;
}

double angleDiffDeg(const double first_yaw, const double second_yaw)
{
  return std::abs(normalizeAngle(first_yaw - second_yaw)) * 180.0 / std::acos(-1.0);
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

Pose2d conflictEntryPoseFor(const PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_entry_pose;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_entry_pose;
  }
  return {};
}

Pose2d conflictExitPoseFor(const PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_exit_pose;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_exit_pose;
  }
  return {};
}

void setConflictSectionForVehicle(PairConflict& conflict,
                                  const int vehicle_index,
                                  const double s_in,
                                  const double s_out,
                                  const double t_in,
                                  const double t_out,
                                  const Pose2d& entry_pose,
                                  const Pose2d& exit_pose)
{
  if (vehicle_index == conflict.first_index)
  {
    conflict.first_s_in = s_in;
    conflict.first_s_out = s_out;
    conflict.first_t_in = t_in;
    conflict.first_t_out = t_out;
    conflict.first_entry_pose = entry_pose;
    conflict.first_exit_pose = exit_pose;
    return;
  }
  if (vehicle_index == conflict.second_index)
  {
    conflict.second_s_in = s_in;
    conflict.second_s_out = s_out;
    conflict.second_t_in = t_in;
    conflict.second_t_out = t_out;
    conflict.second_entry_pose = entry_pose;
    conflict.second_exit_pose = exit_pose;
  }
}

SRange findFirstStableAngleRange(const std::vector<OverlapAngleSample>& samples,
                                 const size_t begin_index,
                                 const bool want_large_angle,
                                 const double angle_threshold_deg,
                                 const double min_length)
{
  // 在按 first_s 排序的重叠采样里寻找第一段稳定角度区间。
  // want_large_angle=true  表示 angle >= threshold 的冲突段；
  // want_large_angle=false 表示 angle <= threshold 的同向/跟车段。
  // 要求区间沿 first_s 的长度至少达到 min_length，避免单个抖动点改变冲突类型。
  if (samples.empty() || begin_index >= samples.size())
  {
    return {};
  }

  const auto match = [&](const double angle_deg) {
    return want_large_angle ? angle_deg >= angle_threshold_deg
                            : angle_deg <= angle_threshold_deg;
  };

  size_t start = 0;
  bool in_range = false;
  for (size_t i = begin_index; i < samples.size(); ++i)
  {
    if (!match(samples[i].angle_deg))
    {
      if (in_range)
      {
        const double length = samples[i - 1].first_s - samples[start].first_s;
        if (length >= min_length)
        {
          return SRange{true, start, i - 1};
        }
      }
      in_range = false;
      continue;
    }

    if (!in_range)
    {
      start = i;
      in_range = true;
    }
  }

  if (in_range)
  {
    const double length = samples.back().first_s - samples[start].first_s;
    if (length >= min_length)
    {
      return SRange{true, start, samples.size() - 1};
    }
  }
  return {};
}

ClassifiedConflictSection classifyOverlapByRelativeAngle(
    const std::vector<OverlapAngleSample>& samples,
    const CoordinatorConfig& config)
{
  ClassifiedConflictSection section;
  if (samples.empty())
  {
    return section;
  }

  if (!config.enable_conflict_type_classification)
  {
    section.active = true;
    section.first_s_in = samples.front().first_s;
    section.first_s_out = samples.back().first_s;
    section.second_s_in = samples.front().second_s;
    section.second_s_out = samples.front().second_s;
    for (const auto& sample : samples)
    {
      section.second_s_in = std::min(section.second_s_in, sample.second_s);
      section.second_s_out = std::max(section.second_s_out, sample.second_s);
    }
    section.type = "RAW_FOOTPRINT_OVERLAP";
    return section;
  }

  const double min_length = std::max(0.0, config.angle_classification_min_length);
  const SRange first_following =
      findFirstStableAngleRange(samples,
                                0,
                                false,
                                config.following_angle_threshold_deg,
                                min_length);
  const SRange first_conflict =
      findFirstStableAngleRange(samples,
                                0,
                                true,
                                config.conflict_angle_threshold_deg,
                                min_length);
  if (!first_conflict.valid)
  {
    // 只有稳定小角度重叠时，它不再走停车让行，而是输出跟车场景，
    // 由同一套 pair 状态机发布 FOLLOW 策略给后车。
    if (!first_following.valid)
    {
      section.type = "UNCLASSIFIED_OVERLAP";
      return section;
    }
    section.active = true;
    section.first_s_in = samples[first_following.start].first_s;
    section.first_s_out = samples[first_following.end].first_s;
    section.second_s_in = std::numeric_limits<double>::infinity();
    section.second_s_out = -std::numeric_limits<double>::infinity();
    for (size_t i = first_following.start; i <= first_following.end; ++i)
    {
      section.second_s_in = std::min(section.second_s_in, samples[i].second_s);
      section.second_s_out = std::max(section.second_s_out, samples[i].second_s);
    }
    section.type = "FOLLOWING_ONLY";
    return section;
  }

  double first_s_in = samples[first_conflict.start].first_s;
  double first_s_out = samples[first_conflict.end].first_s;
  std::string type = "CROSSING_OR_MERGING_CONFLICT";

  if (first_following.valid && first_following.start < first_conflict.start)
  {
    // 一开始是同向跟车，后面角度变大并分开：把分离冲突段起点往前回退一小段，
    // 让速度规划提前进入保护，而不是等到角度已经很大才响应。
    first_s_in = std::max(samples[first_following.start].first_s,
                          samples[first_conflict.start].first_s -
                              std::max(0.0, config.divergence_conflict_back_distance));
    type = "DIVERGING_CONFLICT";
  }
  else
  {
    // 先出现大角度冲突，后续若稳定变成小角度同向，说明车辆开始汇入同一路径。
    // 冲突结束点延长到跟车段内一小段距离，然后由后续跟车逻辑接管。
    const SRange following_after_conflict =
        findFirstStableAngleRange(samples,
                                  first_conflict.end + 1,
                                  false,
                                  config.following_angle_threshold_deg,
                                  min_length);
    if (following_after_conflict.valid)
    {
      first_s_out = std::min(samples[following_after_conflict.end].first_s,
                             std::max(samples[first_conflict.end].first_s,
                                      samples[following_after_conflict.start].first_s) +
                                 std::max(0.0, config.conflict_follow_extension));
      type = "MERGING_CONFLICT_THEN_FOLLOWING";
    }
  }

  section.active = true;
  section.first_s_in = first_s_in;
  section.first_s_out = first_s_out;
  section.second_s_in = std::numeric_limits<double>::infinity();
  section.second_s_out = -std::numeric_limits<double>::infinity();
  for (const auto& sample : samples)
  {
    if (sample.first_s + kEpsilon < section.first_s_in ||
        sample.first_s > section.first_s_out + kEpsilon)
    {
      continue;
    }
    section.second_s_in = std::min(section.second_s_in, sample.second_s);
    section.second_s_out = std::max(section.second_s_out, sample.second_s);
  }

  if (!std::isfinite(section.second_s_in) || !std::isfinite(section.second_s_out))
  {
    return {};
  }
  section.type = type;
  return section;
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
  appendHeldDecisionLocks(agents, active_pairs, result);

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
           << " type=" << conflict.conflict_type
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

  double unused_speed = 0.0;
  std::vector<OverlapAngleSample> overlap_samples;

  // 先在预测时域内可达的 s 区间上扫描空间 footprint 重叠候选。
  // 不能把所有重叠采样直接合并成一个 s_min/s_max 大区间，否则在“汇入同车道”
  // 场景中会把后续跟车段也误认为冲突区。这里每个 first_s 只保留最近的一组
  // overlap，并记录两车轨迹相对航向角，后续用角度序列进行冲突/跟车/分离切分。
  for (const double first_s : first_samples)
  {
    Pose2d first_pose;
    if (!sampleByS(first.trajectory, first_s, first_pose, unused_speed))
    {
      continue;
    }
    bool found_overlap_for_first_s = false;
    double best_second_s = 0.0;
    double best_angle_deg = 0.0;
    double best_distance = std::numeric_limits<double>::infinity();
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

      const double distance = distance2d(first_pose.x, first_pose.y, second_pose.x, second_pose.y);
      if (!found_overlap_for_first_s || distance < best_distance)
      {
        found_overlap_for_first_s = true;
        best_distance = distance;
        best_second_s = second_s;
        best_angle_deg = angleDiffDeg(first_pose.yaw, second_pose.yaw);
      }
    }

    if (found_overlap_for_first_s)
    {
      overlap_samples.push_back(OverlapAngleSample{first_s, best_second_s, best_angle_deg});
    }
  }

  if (overlap_samples.empty())
  {
    return conflict;
  }

  const ClassifiedConflictSection classified_section =
      classifyOverlapByRelativeAngle(overlap_samples, config_);
  if (!classified_section.active)
  {
    return conflict;
  }

  const double first_t_in =
      elapsedTimeBetweenS(first.trajectory, first_progress, classified_section.first_s_in, first_speed);
  const double first_t_out =
      elapsedTimeBetweenS(first.trajectory, first_progress, classified_section.first_s_out, first_speed);
  const double second_t_in =
      elapsedTimeBetweenS(second.trajectory, second_progress, classified_section.second_s_in, second_speed);
  const double second_t_out =
      elapsedTimeBetweenS(second.trajectory, second_progress, classified_section.second_s_out, second_speed);
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
  conflict.first_s_in = classified_section.first_s_in;
  conflict.first_s_out = classified_section.first_s_out;
  conflict.second_s_in = classified_section.second_s_in;
  conflict.second_s_out = classified_section.second_s_out;
  conflict.first_t_in = first_t_in;
  conflict.first_t_out = first_t_out;
  conflict.second_t_in = second_t_in;
  conflict.second_t_out = second_t_out;
  sampleByS(first.trajectory, conflict.first_s_in, conflict.first_entry_pose, unused_speed);
  sampleByS(first.trajectory, conflict.first_s_out, conflict.first_exit_pose, unused_speed);
  sampleByS(second.trajectory, conflict.second_s_in, conflict.second_entry_pose, unused_speed);
  sampleByS(second.trajectory, conflict.second_s_out, conflict.second_exit_pose, unused_speed);
  conflict.collision_point.x = 0.5 * (conflict.first_entry_pose.x + conflict.second_entry_pose.x);
  conflict.collision_point.y = 0.5 * (conflict.first_entry_pose.y + conflict.second_entry_pose.y);
  conflict.collision_point.yaw = conflict.first_entry_pose.yaw;
  conflict.conflict_type = classified_section.type;
  conflict.summary = classified_section.type;
  if (conflict.conflict_type == "FOLLOWING_ONLY")
  {
    const double second_on_first_s = estimateProgress(first.trajectory, second.pose);
    const double first_on_second_s = estimateProgress(second.trajectory, first.pose);
    const double second_lead_gap_on_first_path = second_on_first_s - first_progress;
    const double second_lead_gap_on_second_path = second_progress - first_on_second_s;
    const double lead_tie_epsilon = std::max(0.0, config_.following_lead_tie_epsilon);
    const bool second_ahead_on_first = second_lead_gap_on_first_path > lead_tie_epsilon;
    const bool first_ahead_on_first = second_lead_gap_on_first_path < -lead_tie_epsilon;
    const bool tied_on_first = std::fabs(second_lead_gap_on_first_path) <= lead_tie_epsilon;
    const bool second_ahead_on_second = second_lead_gap_on_second_path > lead_tie_epsilon;
    const bool first_ahead_on_second = second_lead_gap_on_second_path < -lead_tie_epsilon;
    const bool tied_on_second = std::fabs(second_lead_gap_on_second_path) <= lead_tie_epsilon;
    const bool second_leads =
        (second_ahead_on_first && (second_ahead_on_second || tied_on_second)) ||
        (second_ahead_on_second && tied_on_first);
    const bool first_leads =
        (first_ahead_on_first && (first_ahead_on_second || tied_on_second)) ||
        (first_ahead_on_second && tied_on_first);
    if (second_leads == first_leads)
    {
      conflict.active = false;
      return conflict;
    }
    const VehicleAgent& lead_agent = second_leads ? second : first;
    const VehicleAgent& rear_agent = second_leads ? first : second;
    const int lead_index = second_leads ? second_index : first_index;
    const int rear_index = second_leads ? first_index : second_index;
    const double lead_center_gap =
        second_leads
            ? std::max(second_lead_gap_on_first_path, second_lead_gap_on_second_path)
            : std::max(-second_lead_gap_on_first_path, -second_lead_gap_on_second_path);
    const double bumper_gap =
        std::max(0.0, lead_center_gap - 0.5 * lead_agent.length - 0.5 * rear_agent.length);
    if (config_.following_detect_max_gap > 0.0 &&
        bumper_gap > config_.following_detect_max_gap)
    {
      conflict.active = false;
      return conflict;
    }

    conflict.follow_lead_index = lead_index;
    conflict.follow_rear_index = rear_index;
  }
  return conflict;
}

void MultiVehicleCoordinator::chooseOrder(const std::vector<VehicleAgent>& agents,
                                          const std::vector<bool>& already_yielding,
                                          PairConflict& conflict)
{
  const auto key = pairKey(conflict);
  const auto locked = decision_locks_.find(key);
  if (locked != decision_locks_.end())
  {
    if (locked->second.state == PairScenarioState::STOP_LOCKED)
    {
      const PairConflict projected_lock =
          reprojectTrackedConflict(agents, locked->second.conflict);
      const int locked_proceed = locked->second.proceed_index;
      const bool stop_lock_released =
          locked_proceed >= 0 &&
          static_cast<size_t>(locked_proceed) < agents.size() &&
          hasVehiclePassedConflictExit(agents[static_cast<size_t>(locked_proceed)],
                                       projected_lock,
                                       locked_proceed);
      if (!stop_lock_released)
      {
        // STOP 锁仍在生效时，不把当前 FOLLOW 分类直接下发，避免同一 pair 同时输出
        // 停车让行和跟车两套决策。
        if (conflict.conflict_type == "FOLLOWING_ONLY")
        {
          conflict = projected_lock;
        }
        else
        {
          conflict = mergeTrackedConflictWithCurrent(agents, conflict, locked->second.conflict);
        }
        conflict.proceed_index = locked->second.proceed_index;
        conflict.yield_index = locked->second.yield_index;
        conflict.decision_locked = true;
        conflict.decision_source = "LOCK";
        conflict.decision_reason = "hold_stop_before_follow";
        refreshDecisionLock(conflict);
        return;
      }

      decision_locks_.erase(key);
    }
    else if (locked->second.state == PairScenarioState::FOLLOWING &&
             conflict.conflict_type == "FOLLOWING_ONLY")
    {
      // FOLLOWING 锁定的是进入跟车时的前后车关系。当前帧的投影分类只用于确认
      // 仍是同向跟车场景，不能因为后车追近或越过就反向改写 lead/rear。
      conflict.follow_lead_index = locked->second.proceed_index;
      conflict.follow_rear_index = locked->second.yield_index;
      conflict.proceed_index = locked->second.proceed_index;
      conflict.yield_index = locked->second.yield_index;
      conflict.decision_locked = true;
      conflict.decision_source = "LOCK";
      conflict.decision_reason = "following_gap_control";
      storeFollowDecision(conflict);
      return;
    }
    else if (locked->second.state == PairScenarioState::FOLLOWING)
    {
      // FOLLOWING 只用于同向跟车；如果当前帧重新检测到普通冲突，立即退出跟车，
      // 让下面的普通冲突规则重新生成 STOP/PROCEED 决策。
      decision_locks_.erase(key);
    }
  }

  if (conflict.conflict_type == "FOLLOWING_ONLY")
  {
    conflict.proceed_index = conflict.follow_lead_index;
    conflict.yield_index = conflict.follow_rear_index;
    conflict.decision_locked = config_.enable_decision_lock;
    conflict.decision_source = "RULE";
    conflict.decision_reason = "following_gap_control";
    storeFollowDecision(conflict);
    return;
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
      storeDecisionLock(conflict);
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
      storeDecisionLock(conflict);
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
      storeDecisionLock(conflict);
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

  bool first_goes = first_score > second_score + config_.score_tie_epsilon;
  if (std::abs(first_score - second_score) <= config_.score_tie_epsilon)
  {
    first_goes = first.priority <= second.priority;
    conflict.decision_source = "PRIORITY";
    conflict.decision_reason = "score_tie_priority_fallback";
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
    storeDecisionLock(conflict);
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
  // 当前系统的冲突消解策略是 STOP_AND_WAIT：一旦判定出先行/让行关系，
  // 避让车会立即刹停等待。如果下一帧因为避让车减速导致预测 footprint 暂时不重叠，
  // 却没有锁住顺序，就会出现 YIELD/NONE 抖动，车辆反复起停。
  // 因此这里采用首次有效决策即锁定，
  // 释放条件统一放在 appendHeldDecisionLocks() 中判断。
  if (!config_.enable_decision_lock ||
      conflict.first_index < 0 ||
      conflict.second_index < 0 ||
      static_cast<size_t>(conflict.first_index) >= agents.size() ||
      static_cast<size_t>(conflict.second_index) >= agents.size())
  {
    return false;
  }

  return true;
}

std::pair<int, int> MultiVehicleCoordinator::pairKey(const PairConflict& conflict) const
{
  return std::make_pair(std::min(conflict.first_index, conflict.second_index),
                        std::max(conflict.first_index, conflict.second_index));
}

PairConflict MultiVehicleCoordinator::reprojectTrackedConflict(
    const std::vector<VehicleAgent>& agents,
    const PairConflict& tracked_conflict) const
{
  PairConflict projected = tracked_conflict;
  if (tracked_conflict.first_index < 0 || tracked_conflict.second_index < 0 ||
      static_cast<size_t>(tracked_conflict.first_index) >= agents.size() ||
      static_cast<size_t>(tracked_conflict.second_index) >= agents.size())
  {
    return projected;
  }

  const auto project_vehicle_section = [&](const int vehicle_index) {
    const VehicleAgent& agent = agents[static_cast<size_t>(vehicle_index)];
    Pose2d stored_entry_pose = conflictEntryPoseFor(tracked_conflict, vehicle_index);
    Pose2d stored_exit_pose = conflictExitPoseFor(tracked_conflict, vehicle_index);

    // 锁内跨周期保存的是 map 绝对坐标；本轮需要先投影到当前候选轨迹，
    // 再得到当前帧可用于时间估算和 RViz 区间绘制的 s。
    double entry_s = estimateProgress(agent.trajectory, stored_entry_pose);
    double exit_s = estimateProgress(agent.trajectory, stored_exit_pose);

    const double current_s = estimateProgress(agent.trajectory, agent.pose);
    const double fallback_speed =
        plannedSpeedAtProgress(agent.trajectory, current_s, agent.nominal_speed);
    const double t_in = elapsedTimeBetweenS(agent.trajectory, current_s, entry_s, fallback_speed);
    const double t_out = elapsedTimeBetweenS(agent.trajectory, current_s, exit_s, fallback_speed);

    // 入口/出口位姿继续使用锁内保存的绝对坐标。这样下游 planner 每轮都会把同一
    // 冲突空间边界重新投影到自己的当前轨迹，避免把旧轨迹 s 当成长期真值。
    setConflictSectionForVehicle(projected,
                                 vehicle_index,
                                 entry_s,
                                 exit_s,
                                 t_in,
                                 t_out,
                                 stored_entry_pose,
                                 stored_exit_pose);
  };

  project_vehicle_section(projected.first_index);
  project_vehicle_section(projected.second_index);
  projected.conflict_time = std::max(projected.first_t_in, projected.second_t_in);
  projected.conflict_clear_time = std::min(projected.first_t_out, projected.second_t_out);
  projected.collision_point.x = 0.5 * (projected.first_entry_pose.x + projected.second_entry_pose.x);
  projected.collision_point.y = 0.5 * (projected.first_entry_pose.y + projected.second_entry_pose.y);
  projected.collision_point.yaw = projected.first_entry_pose.yaw;
  return projected;
}

PairConflict MultiVehicleCoordinator::mergeTrackedConflictWithCurrent(
    const std::vector<VehicleAgent>& agents,
    const PairConflict& current_conflict,
    const PairConflict& tracked_conflict) const
{
  PairConflict merged = current_conflict;
  if (current_conflict.first_index < 0 || current_conflict.second_index < 0 ||
      static_cast<size_t>(current_conflict.first_index) >= agents.size() ||
      static_cast<size_t>(current_conflict.second_index) >= agents.size())
  {
    return merged;
  }

  const auto merge_vehicle_section = [&](const int vehicle_index) {
    const VehicleAgent& agent = agents[static_cast<size_t>(vehicle_index)];
    const double current_entry_s = conflictEntrySFor(current_conflict, vehicle_index);
    const double current_exit_s = conflictExitSFor(current_conflict, vehicle_index);

    const Pose2d tracked_entry_pose = conflictEntryPoseFor(tracked_conflict, vehicle_index);
    const Pose2d tracked_exit_pose = conflictExitPoseFor(tracked_conflict, vehicle_index);
    double tracked_entry_s = estimateProgress(agent.trajectory, tracked_entry_pose);
    double tracked_exit_s = estimateProgress(agent.trajectory, tracked_exit_pose);
    if (tracked_exit_s < tracked_entry_s)
    {
      std::swap(tracked_entry_s, tracked_exit_s);
    }

    // 冲突事件一旦建立，空间域允许随新检测结果向前/向后扩展，但不因某一帧预测
    // 变短而收缩。这样可以覆盖“冲突区域逐渐扩展到稳态”的场景。
    const double merged_s_in = std::min(std::min(current_entry_s, current_exit_s), tracked_entry_s);
    const double merged_s_out = std::max(std::max(current_entry_s, current_exit_s), tracked_exit_s);

    double unused_speed = 0.0;
    Pose2d merged_entry_pose;
    Pose2d merged_exit_pose;
    sampleByS(agent.trajectory, merged_s_in, merged_entry_pose, unused_speed);
    sampleByS(agent.trajectory, merged_s_out, merged_exit_pose, unused_speed);

    const double current_s = estimateProgress(agent.trajectory, agent.pose);
    const double fallback_speed =
        plannedSpeedAtProgress(agent.trajectory, current_s, agent.nominal_speed);
    const double t_in = elapsedTimeBetweenS(agent.trajectory, current_s, merged_s_in, fallback_speed);
    const double t_out = elapsedTimeBetweenS(agent.trajectory, current_s, merged_s_out, fallback_speed);

    setConflictSectionForVehicle(merged,
                                 vehicle_index,
                                 merged_s_in,
                                 merged_s_out,
                                 t_in,
                                 t_out,
                                 merged_entry_pose,
                                 merged_exit_pose);
  };

  merge_vehicle_section(merged.first_index);
  merge_vehicle_section(merged.second_index);
  merged.active = true;
  merged.conflict_time = std::max(merged.first_t_in, merged.second_t_in);
  merged.conflict_clear_time = std::min(merged.first_t_out, merged.second_t_out);
  merged.collision_point.x = 0.5 * (merged.first_entry_pose.x + merged.second_entry_pose.x);
  merged.collision_point.y = 0.5 * (merged.first_entry_pose.y + merged.second_entry_pose.y);
  merged.collision_point.yaw = merged.first_entry_pose.yaw;
  if (merged.conflict_type.empty())
  {
    merged.conflict_type = tracked_conflict.conflict_type;
  }
  merged.summary = merged.conflict_type;
  merged.first_target_entry_time = std::numeric_limits<double>::quiet_NaN();
  merged.second_target_entry_time = std::numeric_limits<double>::quiet_NaN();
  return merged;
}

bool MultiVehicleCoordinator::hasVehiclePassedConflictExit(const VehicleAgent& agent,
                                                           const PairConflict& conflict,
                                                           const int vehicle_index) const
{
  if (vehicle_index < 0 || !agent.have_pose)
  {
    return false;
  }

  const double margin = std::max(0.0, config_.stop_margin);
  // 当前工程的 trajectory.s 是每轮候选轨迹上的局部弧长，不是全局道路里程。
  // 轨迹持续重规划后，车辆当前位置和历史冲突出口重新投影出来的 s 都可能很小，
  // 因此释放锁时不再使用 current_s / exit_s 比较，而是直接用 map 坐标判断
  // 先行车是否已经沿冲突出口方向越过出口点。
  const Pose2d exit_pose = conflictExitPoseFor(conflict, vehicle_index);
  const double dx = agent.pose.x - exit_pose.x;
  const double dy = agent.pose.y - exit_pose.y;
  const double passed_distance = dx * std::cos(exit_pose.yaw) + dy * std::sin(exit_pose.yaw);
  return passed_distance >= margin;
}

void MultiVehicleCoordinator::storeDecisionLock(const PairConflict& conflict)
{
  if (!config_.enable_decision_lock ||
      conflict.proceed_index < 0 ||
      conflict.yield_index < 0)
  {
    return;
  }

  LockedDecision lock;
  lock.state = PairScenarioState::STOP_LOCKED;
  lock.proceed_index = conflict.proceed_index;
  lock.yield_index = conflict.yield_index;
  lock.conflict = conflict;
  decision_locks_[pairKey(conflict)] = lock;
}

void MultiVehicleCoordinator::storeFollowDecision(const PairConflict& conflict)
{
  if (!config_.enable_decision_lock ||
      conflict.follow_lead_index < 0 ||
      conflict.follow_rear_index < 0)
  {
    return;
  }

  const auto key = pairKey(conflict);
  LockedDecision lock;
  lock.state = PairScenarioState::FOLLOWING;
  lock.proceed_index = conflict.follow_lead_index;
  lock.yield_index = conflict.follow_rear_index;
  lock.follow_miss_count = 0;
  lock.conflict = conflict;
  decision_locks_[key] = lock;
}

void MultiVehicleCoordinator::refreshDecisionLock(const PairConflict& conflict)
{
  if (!config_.enable_decision_lock ||
      conflict.proceed_index < 0 ||
      conflict.yield_index < 0)
  {
    return;
  }

  const auto key = pairKey(conflict);
  auto it = decision_locks_.find(key);
  if (it == decision_locks_.end())
  {
    storeDecisionLock(conflict);
    return;
  }

  // 锁的角色不变，只刷新动态维护后的冲突空间域。
  // 释放不再依赖预测时间，统一用先行车是否已经越过冲突出口判断。
  it->second.state = PairScenarioState::STOP_LOCKED;
  it->second.conflict = conflict;
  it->second.proceed_index = conflict.proceed_index;
  it->second.yield_index = conflict.yield_index;
}

void MultiVehicleCoordinator::appendHeldDecisionLocks(
    const std::vector<VehicleAgent>& agents,
    const std::vector<std::pair<int, int>>& active_pairs,
    CoordinationResult& result)
{
  for (auto it = decision_locks_.begin(); it != decision_locks_.end();)
  {
    if (std::find(active_pairs.begin(), active_pairs.end(), it->first) != active_pairs.end())
    {
      ++it;
      continue;
    }

    LockedDecision& lock = it->second;
    if (lock.state == PairScenarioState::FOLLOWING)
    {
      // 跟车状态必须依赖当前帧的同向重叠和实时车距刷新；无 active FOLLOW 时，
      // 只保留状态用于短时检测丢失滞回，不向 result 追加旧跟车约束。
      ++lock.follow_miss_count;
      if (lock.follow_miss_count >= std::max(1, config_.following_release_count))
      {
        it = decision_locks_.erase(it);
        continue;
      }
      ++it;
      continue;
    }

    if (lock.proceed_index < 0 || lock.yield_index < 0 ||
        static_cast<size_t>(lock.proceed_index) >= agents.size() ||
        static_cast<size_t>(lock.yield_index) >= agents.size())
    {
      it = decision_locks_.erase(it);
      continue;
    }

    const auto& proceed_agent = agents[static_cast<size_t>(lock.proceed_index)];
    const PairConflict projected_lock = reprojectTrackedConflict(agents, lock.conflict);
    if (hasVehiclePassedConflictExit(proceed_agent, projected_lock, lock.proceed_index))
    {
      it = decision_locks_.erase(it);
      continue;
    }

    PairConflict held_conflict = projected_lock;

    // 当前帧没有检测到 footprint overlap 时，不能只按时间释放锁：
    // 避让车可能只是因为已经开始减速，预测时窗内暂时不再与先行车重叠。
    // 现在的释放条件简化为：只要先行车确实通过动态维护的冲突出口，就解除锁。
    // 在此之前继续发布锁定的冲突事件。空间边界使用锁内保存的 map 坐标，
    // 并已在本轮重新投影到当前候选轨迹，因此这里的 t 已经是当前帧相对时间。
    held_conflict.active = true;
    held_conflict.decision_locked = true;
    held_conflict.proceed_index = lock.proceed_index;
    held_conflict.yield_index = lock.yield_index;
    held_conflict.decision_source = "LOCK";
    held_conflict.decision_reason = "hold_locked_order_until_proceed_exit";
    held_conflict.first_target_entry_time = held_conflict.first_t_in;
    held_conflict.second_target_entry_time = held_conflict.second_t_in;

    result.conflicts.push_back(held_conflict);
    result.conflict_active = true;
    ++it;
  }
}

}  // namespace coordination
}  // namespace conflict_prediction_resolution
