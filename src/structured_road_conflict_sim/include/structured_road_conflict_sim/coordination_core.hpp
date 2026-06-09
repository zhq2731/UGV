#pragma once

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace structured_road_conflict_sim
{
namespace coordination
{

struct Pose2d
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct TrajectoryPoint
{
  // 纯算法轨迹点，字段与 planning_msgs::TrajectoryPoint 保持一一对应。
  double relative_time = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
};

struct Trajectory
{
  std::vector<TrajectoryPoint> points;
};

struct VehicleAgent
{
  // 协调器只关心车辆状态、尺寸、优先级和候选轨迹。
  std::string id;
  int priority = 0;
  double nominal_speed = 2.0;
  double length = 3.7;
  double width = 1.85;
  Pose2d pose;
  double speed = 0.0;
  bool have_pose = false;
  bool have_speed = false;
  bool have_trajectory = false;
  Trajectory trajectory;
};

struct CoordinatorConfig
{
  // 冲突预测窗口和车辆 footprint 安全边界。
  double prediction_horizon = 20.0;
  double footprint_safety_margin = 0.25;
  double conflict_time_clearance = 1.0;
  double minimum_yield_speed = 0.25;
  double yield_stop_time_threshold = 2.0;
  double comfortable_deceleration = 2.0;
  double stop_margin = 1.0;
  // 重定时阶段使用的纵向加减速约束。
  double retiming_acceleration_limit = 1.4;
  double retiming_deceleration_limit = 2.5;
  // 软评分权重：只有硬规则无法确定顺序时，才综合这些分量决定谁先行。
  double priority_weight = 0.5;
  double speed_weight = 0.25;
  double progress_weight = 0.25;
  double ttc_weight = 0.25;
  double yield_delay_weight = 0.25;
  double score_tie_epsilon = 0.02;
  // 决策锁参数：靠近冲突区时锁定 pair 顺序，离开足够远并满足保持时间后才允许重算。
  double decision_lock_distance = 6.0;
  double decision_lock_ttc = 3.0;
  double decision_unlock_distance = 9.0;
  double minimum_lock_hold_time = 2.0;
  double decision_switch_margin = 0.15;
  bool enable_conflict_resolution = true;
  bool enable_longitudinal_retiming = true;
  bool enable_decision_lock = true;
};

struct PairConflict
{
  // 记录一对车辆的冲突时刻、清空时刻以及通行/让行顺序。
  bool active = false;
  int first_index = -1;
  int second_index = -1;
  int proceed_index = -1;
  int yield_index = -1;
  bool decision_locked = false;
  double conflict_time = 0.0;
  double conflict_clear_time = 0.0;
  double first_s_in = 0.0;
  double first_s_out = 0.0;
  double second_s_in = 0.0;
  double second_s_out = 0.0;
  double first_t_in = 0.0;
  double first_t_out = 0.0;
  double second_t_in = 0.0;
  double second_t_out = 0.0;
  Pose2d collision_point;
  Pose2d first_entry_pose;
  Pose2d first_exit_pose;
  Pose2d second_entry_pose;
  Pose2d second_exit_pose;
  double first_score = 0.0;
  double second_score = 0.0;
  std::string decision_source;
  std::string decision_reason;
  std::string summary;
};

struct CoordinationResult
{
  // resolve() 的输出：批准轨迹、速度限制、冲突状态和可读状态文本。
  bool ready = false;
  bool conflict_active = false;
  std::string status;
  std::vector<Trajectory> approved_trajectories;
  std::vector<double> speed_limits;
  std::vector<PairConflict> conflicts;
};

class MultiVehicleCoordinator
{
public:
  explicit MultiVehicleCoordinator(CoordinatorConfig config = CoordinatorConfig());

  // ego_index >= 0 时只检测 ego 与其他车辆的冲突；ego_index < 0 保留全量集中式检测。
  CoordinationResult resolve(const std::vector<VehicleAgent>& agents, int ego_index = -1);

private:
  struct LockedDecision
  {
    int proceed_index = -1;
    int yield_index = -1;
    std::chrono::steady_clock::time_point created_at = std::chrono::steady_clock::now();
  };

  PairConflict detectPairConflict(const VehicleAgent& first,
                                  const VehicleAgent& second,
                                  int first_index,
                                  int second_index) const;
  void chooseOrder(const std::vector<VehicleAgent>& agents,
                   const std::vector<bool>& already_yielding,
                   PairConflict& conflict);
  Trajectory retimeYieldTrajectory(const VehicleAgent& yielding_agent,
                                   const PairConflict& conflict,
                                   double target_conflict_time) const;
  double computeYieldSpeedCap(const VehicleAgent& yielding_agent,
                              const PairConflict& conflict,
                              double target_conflict_time) const;

  double estimateProgress(const Trajectory& trajectory, const Pose2d& pose) const;
  double trajectoryLength(const Trajectory& trajectory) const;
  double plannedSpeedAtProgress(const Trajectory& trajectory,
                                double progress_s,
                                double fallback_speed) const;
  bool sampleByS(const Trajectory& trajectory, double s, Pose2d& pose, double& speed) const;
  double elapsedTimeBetweenS(const Trajectory& trajectory,
                             double start_s,
                             double end_s,
                             double fallback_speed) const;
  double reachableSAtHorizon(const Trajectory& trajectory,
                             double current_s,
                             double horizon,
                             double fallback_speed) const;
  std::vector<double> sampledSRange(const Trajectory& trajectory,
                                    double start_s,
                                    double end_s) const;
  bool footprintsOverlap(const Pose2d& first_pose,
                         double first_length,
                         double first_width,
                         const Pose2d& second_pose,
                         double second_length,
                         double second_width) const;
  double decisionScore(const VehicleAgent& agent,
                       double progress_s,
                       double route_length,
                       int other_priority,
                       double ttc,
                       double yield_delay) const;
  bool shouldLockDecision(const std::vector<VehicleAgent>& agents,
                          const PairConflict& conflict) const;
  bool isVehicleInsideConflictInterval(const VehicleAgent& agent,
                                       const PairConflict& conflict,
                                       int vehicle_index) const;
  bool cannotStopBeforeConflictEntry(const VehicleAgent& agent,
                                     const PairConflict& conflict,
                                     int vehicle_index) const;
  bool isVehicleNearConflictEntry(const VehicleAgent& agent,
                                  const PairConflict& conflict,
                                  int vehicle_index) const;
  std::pair<int, int> pairKey(const PairConflict& conflict) const;
  void clearInactiveDecisionLocks(const std::vector<std::pair<int, int>>& active_pairs);

  CoordinatorConfig config_;
  // 决策锁用于抑制临近冲突区时的实时重评分抖动；key 为车辆索引对。
  std::map<std::pair<int, int>, LockedDecision> decision_locks_;
};

}  // namespace coordination
}  // namespace structured_road_conflict_sim
