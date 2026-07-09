#pragma once

#include <mutex>
#include <string>

#include <driver_msgs/ChassisReport.h>
#include <geometry_msgs/Point.h>
#include <localization_msgs/Localization.h>
#include <planning_msgs/ConflictConstraint.h>
#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

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
  struct FollowPeerState
  {
    std::string peer_id;
    geometry_msgs::Point position;
    double speed = 0.0;
    ros::Time pose_stamp;
    ros::Time speed_stamp;
    bool have_pose = false;
    bool have_speed = false;
  };

  bool enabled_ = false;
  bool have_constraint_ = false;

  // 冲突约束超过该时长未更新后，不再按上一帧让行决策执行，转为保守降速。
  double constraint_timeout_ = 2.1;
  // 冲突约束超时时采用的保守速度上限，避免通信/决策中断后继续高速行驶。
  double timeout_max_speed_ = 1.0;
  // 依据当前轨迹投影出的 stop_s 停车兜底时使用的最大减速度，越大越靠近停车点才明显降速。
  double deceleration_limit_ = 1.5;
  // 冲突速度规划和 QP 都无法给出可靠结果时使用的安全兜底减速度，通常高于舒适减速度。
  double emergency_stop_deceleration_ = 2.5;
  // 从冲突入口点向后预留的基础停车安全距离。由于 ConflictConstraint 不再携带 stop_s，
  // 停车点由 planner 使用当前轨迹投影结果实时计算。
  double stop_margin_ = 1.0;
  // 在基础停车安全距离之外额外提前的安全距离。
  double stop_buffer_ = 0.0;
  // 冲突入口/出口地图点投影到当前轨迹时允许的最大横向误差，用于判断约束是否仍适用于当前轨迹。
  double projection_max_lateral_error_ = 2.0;
  // 投影后的冲突入口/出口至少要间隔该 s 距离，防止入口出口投到同一点或顺序异常。
  double projection_min_s_gap_ = 0.2;
  // 根据 planning_start_point 在当前轨迹上匹配拼接段末点时允许的最大距离。
  double stitching_start_match_max_distance_ = 1.0;
  // 单次规划循环时间，拼接段末点 planning_start_point 的 relative_time 会被规范为该值。
  double planning_cycle_time_ = 0.1;
  // FOLLOW 策略：目标间距=max(最小安全距离, 当前速度*最小安全时距)。
  double follow_min_distance_ = 5.0;
  double follow_time_headway_ = 2.0;
  double follow_gap_gain_ = 0.5;
  double follow_relative_speed_gain_ = 0.8;
  double follow_closing_time_ = 3.0;
  double follow_brake_deceleration_ = 1.0;
  double follow_emergency_distance_ = 2.0;
  double follow_state_timeout_ = 0.5;
  double follow_activation_grace_time_ = 0.5;
  double follow_bumper_gap_offset_ = 3.7;
  double follow_projection_max_lateral_error_ = 2.0;

  planning_msgs::ConflictConstraint latest_constraint_;
  FollowPeerState follow_peer_state_;
  ros::Subscriber follow_peer_pose_sub_;
  ros::Subscriber follow_peer_chassis_sub_;
  std::string subscribed_follow_peer_id_;
  bool have_active_follow_ = false;
  std::string active_follow_peer_id_;
  std::string active_follow_conflict_id_;
  ros::Time follow_activation_start_;
  std::mutex mutex_;

  void ensureFollowPeerSubscriptions(const std::string& peer_id);
  void onFollowPeerLocalization(const std::string& peer_id,
                                const localization_msgs::Localization::ConstPtr& msg);
  void onFollowPeerChassis(const std::string& peer_id,
                           const driver_msgs::ChassisReport::ConstPtr& msg);
  FollowPeerState copyFollowPeerState(const std::string& peer_id);
};

}  // namespace conflict_prediction_resolution
