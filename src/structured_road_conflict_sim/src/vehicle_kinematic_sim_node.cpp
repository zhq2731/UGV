#include "structured_road_conflict_sim/vehicle_kinematic_sim_node.hpp"

#include <algorithm>

#include <geometry_msgs/TransformStamped.h>

#include "structured_road_conflict_sim/trajectory_utils.hpp"

namespace structured_road_conflict_sim
{

VehicleKinematicSimNode::VehicleKinematicSimNode() : private_nh_("~")
{
  // 从私有命名空间读取车辆仿真参数，便于每辆车独立配置颜色、尺寸和话题名。
  private_nh_.param<std::string>("vehicle_id", vehicle_id_, vehicle_id_);
  private_nh_.param<std::string>("trajectory_topic", trajectory_topic_, trajectory_topic_);
  private_nh_.param<std::string>("speed_limit_topic", speed_limit_topic_, speed_limit_topic_);
  private_nh_.param<std::string>("pose_topic", pose_topic_, pose_topic_);
  private_nh_.param<std::string>("odom_topic", odom_topic_, odom_topic_);
  private_nh_.param<std::string>("marker_topic", marker_topic_, marker_topic_);
  private_nh_.param<std::string>("base_frame_id", base_frame_id_, vehicle_id_ + "/base_link");
  private_nh_.param("simulation_rate", simulation_rate_, simulation_rate_);
  private_nh_.param("vehicle_length", vehicle_length_, vehicle_length_);
  private_nh_.param("vehicle_width", vehicle_width_, vehicle_width_);
  private_nh_.param("acceleration_limit", acceleration_limit_, acceleration_limit_);
  private_nh_.param("deceleration_limit", deceleration_limit_, deceleration_limit_);
  private_nh_.param("color_r", color_r_, color_r_);
  private_nh_.param("color_g", color_g_, color_g_);
  private_nh_.param("color_b", color_b_, color_b_);
  private_nh_.param("publish_tf", publish_tf_, publish_tf_);
  private_nh_.param("restart_on_new_trajectory", restart_on_new_trajectory_, restart_on_new_trajectory_);

  trajectory_sub_ =
      nh_.subscribe(trajectory_topic_, 1, &VehicleKinematicSimNode::onTrajectory, this);
  if (!speed_limit_topic_.empty())
  {
    speed_limit_sub_ =
        nh_.subscribe(speed_limit_topic_, 1, &VehicleKinematicSimNode::onSpeedLimit, this);
  }
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

  last_update_ = ros::Time::now();
  timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, simulation_rate_)),
                           &VehicleKinematicSimNode::onTimer,
                           this);

  ROS_INFO_STREAM("Vehicle kinematic sim started for " << vehicle_id_
                                                       << ", trajectory_topic=" << trajectory_topic_);
}

void VehicleKinematicSimNode::onTrajectory(const planning_msgs::TrajectoryPointArray::ConstPtr& msg)
{
  if (msg->points.empty())
  {
    return;
  }

  // 默认不因重复收到新轨迹而重置车辆位置，避免协调器周期发布导致仿真跳回起点。
  const bool first_trajectory = !have_trajectory_;
  trajectory_ = *msg;
  have_trajectory_ = true;

  if (first_trajectory || restart_on_new_trajectory_)
  {
    current_s_ = 0.0;
    current_speed_ = 0.0;
    double unused_speed = 0.0;
    sampleTrajectoryByS(trajectory_, current_s_, current_pose_, unused_speed);
  }
}

void VehicleKinematicSimNode::onSpeedLimit(const std_msgs::Float64::ConstPtr& msg)
{
  speed_limit_ = std::max(0.0, msg->data);
}

void VehicleKinematicSimNode::onTimer(const ros::TimerEvent&)
{
  if (!have_trajectory_)
  {
    return;
  }

  const ros::Time now = ros::Time::now();
  const double dt = std::max(0.0, (now - last_update_).toSec());
  last_update_ = now;

  double trajectory_speed = 0.0;
  sampleTrajectoryByS(trajectory_, current_s_, current_pose_, trajectory_speed);

  // 车辆执行速度规划节点输出的最终轨迹；speed_limit 仅保留为可选兼容限速接口。
  const double target_speed = std::min(trajectory_speed, speed_limit_);
  if (current_speed_ < target_speed)
  {
    current_speed_ = std::min(target_speed, current_speed_ + acceleration_limit_ * dt);
  }
  else
  {
    current_speed_ = std::max(target_speed, current_speed_ - deceleration_limit_ * dt);
  }

  // 用当前速度积分更新弧长，再反查轨迹得到新的二维位姿。
  current_s_ = std::min(trajectoryLength(trajectory_), current_s_ + current_speed_ * dt);
  sampleTrajectoryByS(trajectory_, current_s_, current_pose_, trajectory_speed);
  if (current_s_ >= trajectoryLength(trajectory_) - 1.0e-3)
  {
    current_speed_ = 0.0;
  }

  publishState(now);
}

void VehicleKinematicSimNode::publishState(const ros::Time& stamp)
{
  const std::string frame_id = trajectory_.header.frame_id.empty() ? "map" : trajectory_.header.frame_id;
  // 同时发布 pose、odom、TF 和 Marker，供协调器、RViz 与外部节点使用。
  pose_pub_.publish(makePoseStamped(frame_id, current_pose_, stamp));

  nav_msgs::Odometry odom;
  odom.header.frame_id = frame_id;
  odom.header.stamp = stamp;
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose = makePose(current_pose_);
  odom.twist.twist.linear.x = current_speed_;
  odom_pub_.publish(odom);

  if (publish_tf_)
  {
    geometry_msgs::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = current_pose_.x;
    transform.transform.translation.y = current_pose_.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);
  }

  marker_pub_.publish(visualization_.makeVehicleMarkers(
      vehicle_id_,
      current_pose_,
      vehicle_length_,
      vehicle_width_,
      makeColor(color_r_, color_g_, color_b_, 0.92),
      frame_id,
      stamp));
}

}  // namespace structured_road_conflict_sim

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicle_kinematic_sim_node");
  structured_road_conflict_sim::VehicleKinematicSimNode node;
  ros::spin();
  return 0;
}
