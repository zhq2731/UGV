#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <planning_msgs/TrajectoryPointArray.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "structured_road_conflict_sim/common.hpp"
#include "structured_road_conflict_sim/visualization_builder.hpp"

namespace structured_road_conflict_sim
{

class VehicleKinematicSimNode
{
public:
  VehicleKinematicSimNode();

private:
  // 接收协调器批准后的轨迹，按弧长 s 推动车辆在轨迹上运动。
  void onTrajectory(const planning_msgs::TrajectoryPointArray::ConstPtr& msg);
  void onSpeedLimit(const std_msgs::Float64::ConstPtr& msg);
  void onTimer(const ros::TimerEvent& event);
  void publishState(const ros::Time& stamp);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber speed_limit_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  VisualizationBuilder visualization_;
  planning_msgs::TrajectoryPointArray trajectory_;
  Pose2d current_pose_;

  // 话题默认使用相对名称，配合 launch 中的车辆命名空间形成 /vehicle_N/...。
  std::string vehicle_id_ = "vehicle_1";
  std::string trajectory_topic_ = "trajectory_final";
  std::string speed_limit_topic_;
  std::string pose_topic_ = "current_pose";
  std::string odom_topic_ = "odom";
  std::string marker_topic_ = "vehicle_markers";
  std::string base_frame_id_ = "base_link";
  double simulation_rate_ = 30.0;
  double vehicle_length_ = 3.7;
  double vehicle_width_ = 1.85;
  double acceleration_limit_ = 2.0;
  double deceleration_limit_ = 3.0;
  double speed_limit_ = 100.0;
  // current_s_ 表示车辆沿当前轨迹累计前进的弧长。
  double current_speed_ = 0.0;
  double current_s_ = 0.0;
  double color_r_ = 0.05;
  double color_g_ = 0.32;
  double color_b_ = 0.80;
  bool publish_tf_ = true;
  bool restart_on_new_trajectory_ = false;
  bool have_trajectory_ = false;
  ros::Time last_update_;
};

}  // namespace structured_road_conflict_sim
