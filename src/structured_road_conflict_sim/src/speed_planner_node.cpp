#include "structured_road_conflict_sim/speed_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace structured_road_conflict_sim
{

namespace
{

double segmentLength(const planning_msgs::TrajectoryPoint& lhs,
                     const planning_msgs::TrajectoryPoint& rhs)
{
  return std::hypot(rhs.x - lhs.x, rhs.y - lhs.y);
}

}  // namespace

SpeedPlannerNode::SpeedPlannerNode() : private_nh_("~")
{
  private_nh_.param<std::string>("vehicle_id", vehicle_id_, vehicle_id_);
  private_nh_.param<std::string>("candidate_topic", candidate_topic_, candidate_topic_);
  private_nh_.param<std::string>("constraint_topic", constraint_topic_, constraint_topic_);
  private_nh_.param<std::string>("final_topic", final_topic_, final_topic_);
  private_nh_.param("planning_rate", planning_rate_, planning_rate_);
  private_nh_.param("constraint_timeout", constraint_timeout_, constraint_timeout_);
  private_nh_.param("nominal_speed_limit", nominal_speed_limit_, nominal_speed_limit_);
  private_nh_.param("deceleration_limit", deceleration_limit_, deceleration_limit_);
  private_nh_.param("acceleration_limit", acceleration_limit_, acceleration_limit_);
  private_nh_.param("stop_speed_epsilon", stop_speed_epsilon_, stop_speed_epsilon_);

  candidate_sub_ = nh_.subscribe(candidate_topic_, 1, &SpeedPlannerNode::onCandidateTrajectory, this);
  constraint_sub_ = nh_.subscribe(constraint_topic_, 1, &SpeedPlannerNode::onConflictConstraint, this);
  final_pub_ = nh_.advertise<planning_msgs::TrajectoryPointArray>(final_topic_, 1, true);
  last_constraint_time_ = ros::Time(0);
  timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, planning_rate_)),
                           &SpeedPlannerNode::onTimer,
                           this);

  ROS_INFO_STREAM("Speed planner started for " << vehicle_id_
                                                << ", candidate_topic=" << candidate_topic_
                                                << ", constraint_topic=" << constraint_topic_
                                                << ", final_topic=" << final_topic_);
}

void SpeedPlannerNode::onCandidateTrajectory(const planning_msgs::TrajectoryPointArray::ConstPtr& msg)
{
  if (msg->points.empty())
  {
    return;
  }

  latest_candidate_ = *msg;
  have_candidate_ = true;
}

void SpeedPlannerNode::onConflictConstraint(const planning_msgs::ConflictConstraint::ConstPtr& msg)
{
  latest_constraint_ = *msg;
  last_constraint_time_ = ros::Time::now();

  if (msg->role == planning_msgs::ConflictConstraint::ROLE_YIELD)
  {
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "speed planner constraint vehicle=" << vehicle_id_
                                                                  << " role=YIELD peer=" << msg->peer_id
                                                                  << " target_entry_time=" << msg->target_entry_time
                                                                  << " stop_s=" << msg->stop_s
                                                                  << " ego_s_in=" << msg->ego_s_in
                                                                  << " max_speed=" << msg->max_speed
                                                                  << " source=" << msg->decision_source
                                                                  << " reason=" << msg->decision_reason);
  }
}

void SpeedPlannerNode::onTimer(const ros::TimerEvent&)
{
  if (!have_candidate_)
  {
    return;
  }

  final_pub_.publish(makeFinalTrajectory(ros::Time::now()));
}

planning_msgs::TrajectoryPointArray SpeedPlannerNode::makeFinalTrajectory(const ros::Time& stamp) const
{
  planning_msgs::TrajectoryPointArray trajectory = latest_candidate_;
  trajectory.header.stamp = stamp;

  const bool constraint_fresh =
      !last_constraint_time_.isZero() &&
      (stamp - last_constraint_time_).toSec() <= constraint_timeout_;
  if (constraint_fresh &&
      latest_constraint_.role == planning_msgs::ConflictConstraint::ROLE_YIELD)
  {
    applyYieldConstraint(trajectory);
  }
  else
  {
    for (auto& point : trajectory.points)
    {
      point.v = std::min(std::max(0.0, point.v), nominal_speed_limit_);
      point.a = 0.0;
    }
    recomputeTiming(trajectory);
  }

  return trajectory;
}

void SpeedPlannerNode::applyYieldConstraint(planning_msgs::TrajectoryPointArray& trajectory) const
{
  const double stop_s = std::max(0.0, latest_constraint_.stop_s);
  const double ego_s_in = std::max(stop_s, latest_constraint_.ego_s_in);
  const double target_entry_time = std::max(0.0, latest_constraint_.target_entry_time);
  const double max_speed = std::max(0.0, std::min(latest_constraint_.max_speed, nominal_speed_limit_));
  const double decel = std::max(0.1, deceleration_limit_);

  for (auto& point : trajectory.points)
  {
    const double original_speed = std::max(0.0, point.v);
    point.v = std::min(original_speed, max_speed);
    point.a = 0.0;
  }

  recomputeTiming(trajectory);
  const double capped_entry_time = relativeTimeAtS(trajectory, ego_s_in);
  if (!std::isfinite(capped_entry_time) ||
      capped_entry_time + 1.0e-3 >= target_entry_time)
  {
    return;
  }

  for (auto& point : trajectory.points)
  {
    double planned_speed = point.v;

    if (point.s < stop_s)
    {
      const double distance_to_stop = std::max(0.0, stop_s - point.s);
      const double braking_speed_cap = std::sqrt(2.0 * decel * distance_to_stop);
      planned_speed = std::min(planned_speed, braking_speed_cap);
    }
    else if (point.s < ego_s_in)
    {
      planned_speed = 0.0;
    }

    point.v = planned_speed < stop_speed_epsilon_ ? 0.0 : planned_speed;
    point.a = 0.0;
  }

  recomputeTiming(trajectory);
  enforceTargetEntryTime(trajectory, stop_s, ego_s_in, target_entry_time);
}

void SpeedPlannerNode::recomputeTiming(planning_msgs::TrajectoryPointArray& trajectory) const
{
  if (trajectory.points.empty())
  {
    return;
  }

  trajectory.points.front().relative_time = 0.0;
  trajectory.points.front().a = 0.0;
  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    auto& previous = trajectory.points[i - 1];
    auto& current = trajectory.points[i];
    const double ds_from_geometry = segmentLength(previous, current);
    const double ds_from_s = std::max(0.0, current.s - previous.s);
    const double ds = ds_from_s > 1.0e-6 ? ds_from_s : ds_from_geometry;
    const double average_speed = 0.5 * (std::max(0.0, previous.v) + std::max(0.0, current.v));
    const double dt = average_speed > stop_speed_epsilon_ ? ds / average_speed : 0.1;

    current.relative_time = previous.relative_time + dt;
    current.a = dt > 1.0e-6 ? (current.v - previous.v) / dt : 0.0;
    current.a = std::max(-std::abs(deceleration_limit_),
                         std::min(std::abs(acceleration_limit_), current.a));
  }
}

double SpeedPlannerNode::relativeTimeAtS(const planning_msgs::TrajectoryPointArray& trajectory,
                                         const double s) const
{
  if (trajectory.points.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (s <= trajectory.points.front().s)
  {
    return trajectory.points.front().relative_time;
  }

  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    const auto& previous = trajectory.points[i - 1];
    const auto& current = trajectory.points[i];
    if (s > current.s)
    {
      continue;
    }

    const double ds = std::max(1.0e-6, current.s - previous.s);
    const double ratio = std::max(0.0, std::min(1.0, (s - previous.s) / ds));
    return previous.relative_time + (current.relative_time - previous.relative_time) * ratio;
  }

  return trajectory.points.back().relative_time;
}

void SpeedPlannerNode::enforceTargetEntryTime(planning_msgs::TrajectoryPointArray& trajectory,
                                              const double stop_s,
                                              const double ego_s_in,
                                              const double target_entry_time) const
{
  const double entry_time = relativeTimeAtS(trajectory, ego_s_in);
  if (!std::isfinite(entry_time))
  {
    return;
  }

  const double delay = target_entry_time - entry_time;
  if (delay <= 1.0e-3)
  {
    return;
  }

  const double waiting_distance = std::max(1.0e-6, ego_s_in - stop_s);
  for (auto& point : trajectory.points)
  {
    if (point.s <= stop_s)
    {
      continue;
    }

    if (point.s < ego_s_in)
    {
      const double ratio = std::max(0.0, std::min(1.0, (point.s - stop_s) / waiting_distance));
      point.relative_time += delay * ratio;
    }
    else
    {
      point.relative_time += delay;
    }
  }

  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    auto& previous = trajectory.points[i - 1];
    auto& current = trajectory.points[i];
    const double dt = current.relative_time - previous.relative_time;
    current.a = dt > 1.0e-6 ? (current.v - previous.v) / dt : 0.0;
    current.a = std::max(-std::abs(deceleration_limit_),
                         std::min(std::abs(acceleration_limit_), current.a));
  }
}

}  // namespace structured_road_conflict_sim

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speed_planner_node");
  structured_road_conflict_sim::SpeedPlannerNode node;
  ros::spin();
  return 0;
}
