#pragma once

#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace conflict_prediction_resolution
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
  double comfortable_deceleration = 2.0;
  double stop_margin = 1.0;
  // 冲突类型切分参数：先用 footprint 找重叠候选，再用两车轨迹相对航向角把重叠段
  // 分成大角度冲突段、小角度跟车段和分离段。
  bool enable_conflict_type_classification = true;
  double conflict_angle_threshold_deg = 30.0;
  double following_angle_threshold_deg = 20.0;
  double angle_classification_min_length = 4.0;
  double conflict_follow_extension = 3.0;
  double divergence_conflict_back_distance = 3.0;
  int following_release_count = 2;
  double following_detect_max_gap = 30.0;
  double following_lead_tie_epsilon = 0.5;
  // 软评分权重：只有硬规则无法确定顺序时，才综合这些分量决定谁先行。
  double priority_weight = 0.5;
  double speed_weight = 0.25;
  double progress_weight = 0.25;
  double ttc_weight = 0.25;
  double yield_delay_weight = 0.25;
  double score_tie_epsilon = 0.02;
  // 决策锁参数：一旦产生 pair 顺序就保持，并在锁内动态维护同一冲突事件的空间边界。
  bool enable_conflict_resolution = true;
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
  int follow_lead_index = -1;
  int follow_rear_index = -1;
  // 判定层直接给每辆车的目标进入时间。默认 NaN 表示使用
  // “先行车 t_out + conflict_time_clearance”的普通计算方式。
  // 当冲突段因为避让车停车而暂时不再重叠时，锁定决策会用这里保存倒计时后的释放时间，
  // 继续向速度层发布稳定的让行约束。
  double first_target_entry_time = std::numeric_limits<double>::quiet_NaN();
  double second_target_entry_time = std::numeric_limits<double>::quiet_NaN();
  std::string decision_source;
  std::string decision_reason;
  std::string conflict_type;
  std::string summary;
};

struct CoordinationResult
{
  // resolve() 的输出：冲突状态、pair 决策和可读状态文本。
  bool ready = false;
  bool conflict_active = false;
  std::string status;
  std::vector<PairConflict> conflicts;
};

class MultiVehicleCoordinator
{
public:
  explicit MultiVehicleCoordinator(CoordinatorConfig config = CoordinatorConfig());

  // ego_index >= 0 时只检测 ego 与其他车辆的冲突；ego_index < 0 保留全量集中式检测。
  CoordinationResult resolve(const std::vector<VehicleAgent>& agents, int ego_index = -1);

private:
  enum class PairScenarioState
  {
    STOP_LOCKED,
    FOLLOWING
  };

  struct LockedDecision
  {
    PairScenarioState state = PairScenarioState::STOP_LOCKED;
    int proceed_index = -1;
    int yield_index = -1;
    int follow_miss_count = 0;
    PairConflict conflict;
  };

  PairConflict detectPairConflict(const VehicleAgent& first,
                                  const VehicleAgent& second,
                                  int first_index,
                                  int second_index) const;
  void chooseOrder(const std::vector<VehicleAgent>& agents,
                   const std::vector<bool>& already_yielding,
                   PairConflict& conflict);

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
  std::pair<int, int> pairKey(const PairConflict& conflict) const;
  PairConflict reprojectTrackedConflict(const std::vector<VehicleAgent>& agents,
                                        const PairConflict& tracked_conflict) const;
  PairConflict mergeTrackedConflictWithCurrent(const std::vector<VehicleAgent>& agents,
                                               const PairConflict& current_conflict,
                                               const PairConflict& tracked_conflict) const;
  bool hasVehiclePassedConflictExit(const VehicleAgent& agent,
                                    const PairConflict& conflict,
                                    int vehicle_index) const;
  void storeDecisionLock(const PairConflict& conflict);
  void storeFollowDecision(const PairConflict& conflict);
  void refreshDecisionLock(const PairConflict& conflict);
  void appendHeldDecisionLocks(const std::vector<VehicleAgent>& agents,
                               const std::vector<std::pair<int, int>>& active_pairs,
                               CoordinationResult& result);

  CoordinatorConfig config_;
  // 决策锁用于抑制临近冲突区时的实时重评分抖动；key 为车辆索引对。
  std::map<std::pair<int, int>, LockedDecision> decision_locks_;
};

}  // namespace coordination
}  // namespace conflict_prediction_resolution
