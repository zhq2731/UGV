#pragma once

#include <ros/ros.h>
#include <planning_msgs/ConflictConstraint.h>
#include <planning_msgs/TrajectoryPointArray.h>

namespace structured_road_conflict_sim
{

class SpeedPlannerNode
{
public:
  SpeedPlannerNode();

private:
  void onCandidateTrajectory(const planning_msgs::TrajectoryPointArray::ConstPtr& msg);
  void onConflictConstraint(const planning_msgs::ConflictConstraint::ConstPtr& msg);
  void onTimer(const ros::TimerEvent& event);

  planning_msgs::TrajectoryPointArray makeFinalTrajectory(const ros::Time& stamp) const;
  void applyYieldConstraint(planning_msgs::TrajectoryPointArray& trajectory) const;
  void recomputeTiming(planning_msgs::TrajectoryPointArray& trajectory) const;
  double relativeTimeAtS(const planning_msgs::TrajectoryPointArray& trajectory, double s) const;
  void enforceTargetEntryTime(planning_msgs::TrajectoryPointArray& trajectory,
                              double stop_s,
                              double ego_s_in,
                              double target_entry_time) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber candidate_sub_;
  ros::Subscriber constraint_sub_;
  ros::Publisher final_pub_;
  ros::Timer timer_;

  planning_msgs::TrajectoryPointArray latest_candidate_;
  planning_msgs::ConflictConstraint latest_constraint_;

  std::string vehicle_id_ = "vehicle_1";
  std::string candidate_topic_ = "trajectory_candidate";
  std::string constraint_topic_ = "conflict_constraint";
  std::string final_topic_ = "trajectory_final";
  double planning_rate_ = 20.0;
  double constraint_timeout_ = 0.8;
  double nominal_speed_limit_ = 100.0;
  double deceleration_limit_ = 2.0;
  double acceleration_limit_ = 1.4;
  double stop_speed_epsilon_ = 0.05;
  bool have_candidate_ = false;
  ros::Time last_constraint_time_;
};

}  // namespace structured_road_conflict_sim
