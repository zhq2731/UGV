#pragma once

#include <mutex>

#include <planning_msgs/ConflictConstraint.h>
#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/node_handle.h>

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
  void apply(planning_msgs::TrajectoryPointArray& trajectory);

private:
  bool enabled_ = false;
  bool have_constraint_ = false;

  // 冲突约束超过该时长未更新后，不再按旧让行决策执行，转入保守限速。
  double constraint_timeout_ = 0.5;
  // 冲突约束超时时采用的保守速度上限，避免通信/决策中断后恢复高速。
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

  planning_msgs::ConflictConstraint latest_constraint_;
  std::mutex mutex_;
};

}  // namespace conflict_prediction_resolution
