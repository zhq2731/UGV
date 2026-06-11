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
  // 低速等待段截断阈值：相邻点几乎不动且速度低于该值时，认为后续是在原地等待。
  double waiting_truncation_speed_threshold_ = 0.15;
  // 判断等待平台段时使用的最小 s/xy 变化阈值。
  double waiting_truncation_s_gap_ = 0.01;
  double waiting_truncation_xy_gap_ = 0.01;

  ConflictVelocityOptimizer velocity_optimizer_;
  planning_msgs::ConflictConstraint latest_constraint_;
  std::mutex mutex_;
};

}  // namespace conflict_prediction_resolution
