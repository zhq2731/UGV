#pragma once

#include <mutex>

#include <geometry_msgs/Point.h>
#include <planning_msgs/ConflictConstraint.h>
#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/node_handle.h>

#include "conflict_prediction_resolution/conflict_velocity_optimizer.hpp"

namespace conflict_prediction_resolution
{

class ConflictConstraintProcessor
{
public:
  // 从 planner 私有参数读取冲突约束处理开关和速度修正参数。
  void loadParam(ros::NodeHandle& private_nh);

  bool enabled() const { return enabled_; }

  // ROS subscriber 回调只负责把最新约束交给处理器缓存，避免 PlanningNode 关心锁和消息细节。
  void updateConstraint(const planning_msgs::ConflictConstraint& constraint);

  // 在基础速度规划之后调用。该函数会原地修改 trajectory 的 v/a/relative_time。
  // planning_start_point 是路径规划使用的拼接段末点；冲突规则速度只修改它之后的非拼接段。
  void apply(planning_msgs::TrajectoryPointArray& trajectory,
             const geometry_msgs::Point& planning_start_point);

private:
  bool enabled_ = false;
  bool have_constraint_ = false;

  // 冲突约束超过该时长未更新后，不再按旧让行决策执行，转为保守降速。
  double constraint_timeout_ = 2.1;
  // 冲突约束超时时采用的保守速度上限，避免通信/决策中断后继续高速行驶。
  double timeout_max_speed_ = 1.0;
  // 依据当前轨迹投影出的 stop_s 停车兜底时使用的最大减速度，越大越靠近停车点才明显降速。
  double deceleration_limit_ = 1.5;
  // 冲突速度规划和 QP 都无法给出可靠结果时使用的安全兜底减速度，通常高于舒适减速度。
  double emergency_stop_deceleration_ = 2.5;
  // 平滑让行规则粗解使用的额定舒适减速度，车辆会先按该减速度逐步降到巡航让行速度。
  double smooth_yield_deceleration_ = 0.8;
  // 从冲突入口点向后预留的基础停车安全距离。由于 ConflictConstraint 不再携带 stop_s，
  // 停车点由 planner 使用当前轨迹投影结果实时计算。
  double stop_margin_ = 1.0;
  // 在基础停车安全距离之外额外提前的安全距离。
  double stop_buffer_ = 0.0;
  // 如果按 target_entry_time 计算出的让行速度低于该值，则放弃爬行让行，改为停车等待。
  double min_smooth_yield_speed_ = 0.3;
  // 冲突入口/出口地图点投影到当前轨迹时允许的最大横向误差，用于判断旧约束是否仍适用于当前轨迹。
  double projection_max_lateral_error_ = 2.0;
  // 投影后的冲突入口/出口至少要间隔该 s 距离，防止入口出口投到同一点或顺序异常。
  double projection_min_s_gap_ = 0.2;
  // 根据 planning_start_point 在当前轨迹上匹配拼接段末点时允许的最大距离。
  double stitching_start_match_max_distance_ = 1.0;
  // 冲突规则速度修正后，将非拼接段粗轨迹重采样到固定时间间隔，便于后续定时间 QP 使用。
  double fixed_time_coarse_dt_ = 0.1;
  // 固定时间粗轨迹最多保留的非拼接后缀点数，避免低速让行把轨迹采样成数千点。
  int fixed_time_max_points_ = 160;
  // 单次规划循环时间，拼接段末点 planning_start_point 的 relative_time 会被规范为该值。
  double planning_cycle_time_ = 0.1;

  ConflictVelocityOptimizer velocity_optimizer_;
  planning_msgs::ConflictConstraint latest_constraint_;
  // 最近一次完整的让行决策缓存。
  // 用途：冲突预判在避让车减速后可能短暂输出 ROLE_NONE，若 planner 立即释放约束，
  // 车辆会重新加速并再次触发冲突，形成速度上下波动。这里缓存 YIELD 决策，
  // 在很短的释放保持时间内继续让行，具体是否还能使用仍由 cpp 中的空间投影校验决定。
  planning_msgs::ConflictConstraint last_yield_constraint_;
  ros::Time last_yield_update_time_;
  bool have_last_yield_constraint_ = false;
  std::mutex mutex_;
};

}  // namespace conflict_prediction_resolution
