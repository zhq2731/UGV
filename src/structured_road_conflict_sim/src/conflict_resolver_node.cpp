#include "structured_road_conflict_sim/conflict_resolver_node.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <sstream>

#include <boost/bind.hpp>
#include <planning_msgs/ConflictConstraint.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include "structured_road_conflict_sim/common.hpp"

namespace structured_road_conflict_sim
{
namespace
{

std::string defaultVehicleTopic(const std::string& vehicle_id, const std::string& suffix)
{
  // 多车场景统一按 /vehicle_N/suffix 生成默认话题名。
  return "/" + vehicle_id + "/" + suffix;
}

std::string normalizeScenarioName(std::string scenario)
{
  for (char& ch : scenario)
  {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (ch == '-')
    {
      ch = '_';
    }
  }
  return scenario;
}

int defaultVehicleCountForScenario(const std::string& scenario)
{
  if (scenario == "four_way_straight")
  {
    return 4;
  }
  if (scenario == "unprotected_left_turn" || scenario == "left_right_turn_conflict")
  {
    return 2;
  }
  return 2;
}

visualization_msgs::Marker baseMarker(const std::string& frame_id,
                                      const std::string& ns,
                                      const int id,
                                      const int type,
                                      const ros::Time& stamp)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = ros::Duration(1.2);
  return marker;
}

geometry_msgs::Point makeMarkerPoint(const coordination::Pose2d& pose, const double z)
{
  geometry_msgs::Point point;
  point.x = pose.x;
  point.y = pose.y;
  point.z = z;
  return point;
}

geometry_msgs::Point interpolateTrajectoryPointByS(const planning_msgs::TrajectoryPointArray& trajectory,
                                                   const double s,
                                                   const double z)
{
  geometry_msgs::Point point;
  point.z = z;
  if (trajectory.points.empty())
  {
    return point;
  }

  if (s <= trajectory.points.front().s)
  {
    point.x = trajectory.points.front().x;
    point.y = trajectory.points.front().y;
    return point;
  }

  for (size_t i = 1; i < trajectory.points.size(); ++i)
  {
    const auto& prev = trajectory.points[i - 1];
    const auto& next = trajectory.points[i];
    if (s > next.s)
    {
      continue;
    }

    const double ds = std::max(1.0e-6, next.s - prev.s);
    const double ratio = clamp((s - prev.s) / ds, 0.0, 1.0);
    point.x = prev.x + (next.x - prev.x) * ratio;
    point.y = prev.y + (next.y - prev.y) * ratio;
    return point;
  }

  point.x = trajectory.points.back().x;
  point.y = trajectory.points.back().y;
  return point;
}

std::vector<geometry_msgs::Point> makeTrajectorySectionPoints(
    const planning_msgs::TrajectoryPointArray& trajectory,
    const double s_in,
    const double s_out,
    const double z)
{
  std::vector<geometry_msgs::Point> points;
  if (trajectory.points.empty())
  {
    return points;
  }

  const double lower_s = std::min(s_in, s_out);
  const double upper_s = std::max(s_in, s_out);
  points.push_back(interpolateTrajectoryPointByS(trajectory, lower_s, z));
  for (const auto& point : trajectory.points)
  {
    if (point.s <= lower_s || point.s >= upper_s)
    {
      continue;
    }

    geometry_msgs::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = z;
    points.push_back(marker_point);
  }
  points.push_back(interpolateTrajectoryPointByS(trajectory, upper_s, z));
  return points;
}

double sInFor(const coordination::PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_s_in;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_s_in;
  }
  return 0.0;
}

double sOutFor(const coordination::PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_s_out;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_s_out;
  }
  return 0.0;
}

double tInFor(const coordination::PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_t_in;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_t_in;
  }
  return 0.0;
}

double tOutFor(const coordination::PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_t_out;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_t_out;
  }
  return 0.0;
}

coordination::Pose2d entryPoseFor(const coordination::PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_entry_pose;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_entry_pose;
  }
  return coordination::Pose2d{};
}

coordination::Pose2d exitPoseFor(const coordination::PairConflict& conflict, const int vehicle_index)
{
  if (vehicle_index == conflict.first_index)
  {
    return conflict.first_exit_pose;
  }
  if (vehicle_index == conflict.second_index)
  {
    return conflict.second_exit_pose;
  }
  return coordination::Pose2d{};
}

}  // namespace

ConflictResolverNode::ConflictResolverNode()
  : private_nh_("~"), config_(loadCoordinatorConfig()), coordinator_(config_)
{
  readParam<std::string>("frame_id", frame_id_, frame_id_);
  readParam<std::string>("conflict_markers_topic", conflict_markers_topic_, conflict_markers_topic_);
  readParam("decision_rate", decision_rate_, decision_rate_);
  readParam("heading_compensation_degree", heading_compensation_degree_, heading_compensation_degree_);
  private_nh_.param("ego_index", ego_index_, ego_index_);
  private_nh_.param("publish_debug", publish_debug_, publish_debug_);

  loadVehicles();
  if (ego_index_ >= static_cast<int>(vehicles_.size()))
  {
    ROS_WARN_STREAM("ego_index=" << ego_index_ << " is outside active vehicle_count="
                                 << vehicles_.size() << "; this resolver will stay passive");
  }
  setupRosInterfaces();

  timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, decision_rate_)),
                           &ConflictResolverNode::onTimer,
                           this);

  ROS_INFO_STREAM("Distributed conflict resolver started with " << vehicles_.size()
                  << " known vehicles, ego_index=" << ego_index_
                  << ", publish_debug=" << std::boolalpha << publish_debug_);
}

coordination::CoordinatorConfig ConflictResolverNode::loadCoordinatorConfig() const
{
  coordination::CoordinatorConfig config;
  readParam("prediction_horizon", config.prediction_horizon, config.prediction_horizon);
  readParam("footprint_safety_margin", config.footprint_safety_margin, config.footprint_safety_margin);
  readParam("conflict_time_clearance", config.conflict_time_clearance, config.conflict_time_clearance);
  readParam("minimum_yield_speed", config.minimum_yield_speed, config.minimum_yield_speed);
  readParam("yield_stop_time_threshold", config.yield_stop_time_threshold, config.yield_stop_time_threshold);
  readParam("comfortable_deceleration", config.comfortable_deceleration, config.comfortable_deceleration);
  readParam("stop_margin", config.stop_margin, config.stop_margin);
  readParam("retiming_acceleration_limit",
            config.retiming_acceleration_limit,
            config.retiming_acceleration_limit);
  readParam("retiming_deceleration_limit",
            config.retiming_deceleration_limit,
            config.retiming_deceleration_limit);
  readParam("priority_weight", config.priority_weight, config.priority_weight);
  readParam("speed_weight", config.speed_weight, config.speed_weight);
  readParam("progress_weight", config.progress_weight, config.progress_weight);
  readParam("ttc_weight", config.ttc_weight, config.ttc_weight);
  readParam("yield_delay_weight", config.yield_delay_weight, config.yield_delay_weight);
  readParam("score_tie_epsilon", config.score_tie_epsilon, config.score_tie_epsilon);
  readParam("decision_lock_distance", config.decision_lock_distance, config.decision_lock_distance);
  readParam("decision_lock_ttc", config.decision_lock_ttc, config.decision_lock_ttc);
  readParam("decision_unlock_distance", config.decision_unlock_distance, config.decision_unlock_distance);
  readParam("minimum_lock_hold_time", config.minimum_lock_hold_time, config.minimum_lock_hold_time);
  readParam("decision_switch_margin", config.decision_switch_margin, config.decision_switch_margin);
  readParam("enable_conflict_resolution", config.enable_conflict_resolution, config.enable_conflict_resolution);
  readParam("enable_longitudinal_retiming", config.enable_longitudinal_retiming, config.enable_longitudinal_retiming);
  readParam("enable_decision_lock", config.enable_decision_lock, config.enable_decision_lock);
  return config;
}

void ConflictResolverNode::loadVehicles()
{
  int vehicle_count = 2;
  const bool has_private_vehicle_count = private_nh_.getParam("vehicle_count", vehicle_count);
  if (!has_private_vehicle_count && !nh_.getParam("/conflict_resolver_node/vehicle_count", vehicle_count))
  {
    nh_.getParam("/road_network_node/vehicle_count", vehicle_count);
  }
  if (vehicle_count <= 0)
  {
    std::string scenario = "four_way_straight";
    // 车辆数为 0 表示自动模式；此时以 road_network_node 的场景为准，
    // 确保协调器等待的车辆数量和实际发布候选轨迹的车辆数量一致。
    ros::param::get("/road_network_node/scenario", scenario);
    vehicle_count = defaultVehicleCountForScenario(normalizeScenarioName(scenario));
  }
  vehicle_count = std::max(1, vehicle_count);
  vehicles_.resize(static_cast<size_t>(vehicle_count));

  for (int i = 0; i < vehicle_count; ++i)
  {
    // 优先使用 vehicles/i 新格式；未配置时回退到 vehicle_N 默认命名。
    const std::string default_id = "vehicle_" + std::to_string(i + 1);
    const std::string prefix = "vehicles/" + std::to_string(i) + "/";
    VehicleIo& vehicle = vehicles_[static_cast<size_t>(i)];
    vehicle.agent.id = default_id;

    readParam<std::string>(prefix + "id", vehicle.agent.id, vehicle.agent.id);
    readParam(prefix + "priority", vehicle.agent.priority, i);
    readParam(prefix + "nominal_speed", vehicle.agent.nominal_speed, vehicle.agent.nominal_speed);
    readParam(prefix + "vehicle_length", vehicle.agent.length, vehicle.agent.length);
    readParam(prefix + "vehicle_width", vehicle.agent.width, vehicle.agent.width);
  }
}

void ConflictResolverNode::setupRosInterfaces()
{
  if (publish_debug_)
  {
    conflict_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(conflict_markers_topic_, 1);
  }

  for (size_t i = 0; i < vehicles_.size(); ++i)
  {
    VehicleIo& vehicle = vehicles_[i];
    const std::string prefix = "vehicles/" + std::to_string(i) + "/";
    // 每辆车都直接订阅 UGV 原生定位/底盘/候选轨迹，发布本车冲突约束。
    std::string localization_topic = defaultVehicleTopic(vehicle.agent.id, "odomData");
    std::string chassis_topic = defaultVehicleTopic(vehicle.agent.id, "chassis");
    std::string candidate_topic = defaultVehicleTopic(vehicle.agent.id, "trajectory_candidate");
    std::string approved_topic = defaultVehicleTopic(vehicle.agent.id, "approved_trajectory");
    std::string speed_limit_topic = defaultVehicleTopic(vehicle.agent.id, "speed_limit");
    std::string constraint_topic = defaultVehicleTopic(vehicle.agent.id, "conflict_constraint");

    readParam<std::string>(prefix + "localization_topic", localization_topic, localization_topic);
    readParam<std::string>(prefix + "chassis_topic", chassis_topic, chassis_topic);
    readParam<std::string>(prefix + "candidate_topic", candidate_topic, candidate_topic);
    readParam<std::string>(prefix + "approved_topic", approved_topic, approved_topic);
    readParam<std::string>(prefix + "speed_limit_topic", speed_limit_topic, speed_limit_topic);
    readParam<std::string>(prefix + "constraint_topic", constraint_topic, constraint_topic);

    vehicle.localization_sub = nh_.subscribe<localization_msgs::Localization>(
        localization_topic, 1, boost::bind(&ConflictResolverNode::onLocalization, this, i, _1));
    vehicle.chassis_sub = nh_.subscribe<driver_msgs::ChassisReport>(
        chassis_topic, 1, boost::bind(&ConflictResolverNode::onChassis, this, i, _1));
    vehicle.candidate_sub = nh_.subscribe<planning_msgs::TrajectoryPointArray>(
        candidate_topic, 1, boost::bind(&ConflictResolverNode::onCandidate, this, i, _1));
    if (shouldPublishVehicle(i))
    {
      vehicle.approved_pub = nh_.advertise<planning_msgs::TrajectoryPointArray>(approved_topic, 1, true);
      vehicle.speed_limit_pub = nh_.advertise<std_msgs::Float64>(speed_limit_topic, 1, true);
      vehicle.constraint_pub = nh_.advertise<planning_msgs::ConflictConstraint>(constraint_topic, 1, true);
    }
  }
}

void ConflictResolverNode::onLocalization(const size_t index, const localization_msgs::Localization::ConstPtr& msg)
{
  if (index >= vehicles_.size())
  {
    return;
  }
  const auto& pose_msg = msg->location.pose.pose;
  double yaw = tf::getYaw(pose_msg.orientation);
  yaw += M_PI / 2.0;
  yaw += heading_compensation_degree_ * M_PI / 180.0;
  yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  vehicles_[index].agent.pose = coordination::Pose2d{pose_msg.position.x, pose_msg.position.y, yaw};
  vehicles_[index].agent.have_pose = true;
}

void ConflictResolverNode::onChassis(const size_t index, const driver_msgs::ChassisReport::ConstPtr& msg)
{
  if (index >= vehicles_.size())
  {
    return;
  }
  double speed = msg->current_velocity;
  if (msg->gear_location == 7)
  {
    speed = -speed;
  }
  vehicles_[index].agent.speed = std::fabs(speed);
  vehicles_[index].agent.have_speed = true;
}

void ConflictResolverNode::onCandidate(const size_t index,
                                       const planning_msgs::TrajectoryPointArray::ConstPtr& msg)
{
  if (index >= vehicles_.size() || msg->points.empty())
  {
    return;
  }
  vehicles_[index].latest_candidate = *msg;
  vehicles_[index].agent.trajectory = toCoreTrajectory(*msg);
  vehicles_[index].agent.have_trajectory = true;
  vehicles_[index].have_candidate = true;
}

void ConflictResolverNode::onTimer(const ros::TimerEvent&)
{
  std::vector<coordination::VehicleAgent> agents;
  agents.reserve(vehicles_.size());
  // 拷贝一份纯算法状态，避免协调核心直接接触 ROS subscriber/publisher。
  for (const auto& vehicle : vehicles_)
  {
    agents.push_back(vehicle.agent);
  }

  const ros::Time stamp = ros::Time::now();
  publishResult(coordinator_.resolve(agents, ego_index_), stamp);
}

coordination::Trajectory ConflictResolverNode::toCoreTrajectory(
    const planning_msgs::TrajectoryPointArray& msg) const
{
  coordination::Trajectory trajectory;
  trajectory.points.reserve(msg.points.size());
  // 只转换协调算法需要的运动学字段。
  for (const auto& point : msg.points)
  {
    coordination::TrajectoryPoint core_point;
    core_point.relative_time = point.relative_time;
    core_point.x = point.x;
    core_point.y = point.y;
    core_point.yaw = point.theta;
    core_point.s = point.s;
    core_point.v = point.v;
    core_point.a = point.a;
    trajectory.points.push_back(core_point);
  }
  return trajectory;
}

planning_msgs::TrajectoryPointArray ConflictResolverNode::fromCoreTrajectory(
    const coordination::Trajectory& trajectory,
    const planning_msgs::TrajectoryPointArray& original,
    const ros::Time& stamp) const
{
  // 保留原消息中的 header/task_area/shape 等元数据，只覆盖被重定时的轨迹点字段。
  planning_msgs::TrajectoryPointArray msg = original;
  msg.header.stamp = stamp;
  const size_t count = std::min(msg.points.size(), trajectory.points.size());
  for (size_t i = 0; i < count; ++i)
  {
    msg.points[i].relative_time = trajectory.points[i].relative_time;
    msg.points[i].x = trajectory.points[i].x;
    msg.points[i].y = trajectory.points[i].y;
    msg.points[i].theta = trajectory.points[i].yaw;
    msg.points[i].s = trajectory.points[i].s;
    msg.points[i].v = trajectory.points[i].v;
    msg.points[i].a = trajectory.points[i].a;
  }
  return msg;
}

void ConflictResolverNode::publishResult(const coordination::CoordinationResult& result,
                                         const ros::Time& stamp)
{
  const auto make_constraint_msg = [&](const size_t vehicle_index) {
    // 将 pair 决策转成单车视角 ST 约束：每辆车只收到“我是谁、对方是谁、
    // 我应先行还是让行、我的 s/t 冲突窗口、让行目标时间和当前速度上限”。
    planning_msgs::ConflictConstraint msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = stamp;
    msg.role = planning_msgs::ConflictConstraint::ROLE_NONE;
    msg.ego_id = vehicles_[vehicle_index].agent.id;
    msg.max_speed = 100.0;
    msg.stop_s = 0.0;

    if (!result.ready || !result.conflict_active)
    {
      return msg;
    }

    for (const auto& conflict : result.conflicts)
    {
      if (conflict.proceed_index < 0 || conflict.yield_index < 0 ||
          static_cast<size_t>(conflict.proceed_index) >= vehicles_.size() ||
          static_cast<size_t>(conflict.yield_index) >= vehicles_.size())
      {
        continue;
      }
      if (static_cast<int>(vehicle_index) != conflict.proceed_index &&
          static_cast<int>(vehicle_index) != conflict.yield_index)
      {
        continue;
      }

      const int peer_index =
          static_cast<int>(vehicle_index) == conflict.first_index ? conflict.second_index : conflict.first_index;
      if (peer_index < 0 || static_cast<size_t>(peer_index) >= vehicles_.size())
      {
        continue;
      }

      std::ostringstream conflict_id;
      conflict_id.precision(1);
      conflict_id << std::fixed
                  << vehicles_[static_cast<size_t>(conflict.first_index)].agent.id << "_"
                  << vehicles_[static_cast<size_t>(conflict.second_index)].agent.id
                  << "_s" << conflict.first_s_in << "_" << conflict.second_s_in;

      msg.conflict_id = conflict_id.str();
      msg.peer_id = vehicles_[static_cast<size_t>(peer_index)].agent.id;
      msg.role = static_cast<int>(vehicle_index) == conflict.yield_index
                     ? planning_msgs::ConflictConstraint::ROLE_YIELD
                     : planning_msgs::ConflictConstraint::ROLE_PROCEED;
      msg.ego_s_in = sInFor(conflict, static_cast<int>(vehicle_index));
      msg.ego_s_out = sOutFor(conflict, static_cast<int>(vehicle_index));
      msg.ego_t_in = tInFor(conflict, static_cast<int>(vehicle_index));
      msg.ego_t_out = tOutFor(conflict, static_cast<int>(vehicle_index));
      msg.peer_s_in = sInFor(conflict, peer_index);
      msg.peer_s_out = sOutFor(conflict, peer_index);
      msg.peer_t_in = tInFor(conflict, peer_index);
      msg.peer_t_out = tOutFor(conflict, peer_index);
      msg.earliest_entry_time = msg.ego_t_in;
      msg.target_entry_time = msg.role == planning_msgs::ConflictConstraint::ROLE_YIELD
                                  ? tOutFor(conflict, conflict.proceed_index) + config_.conflict_time_clearance
                                  : msg.ego_t_in;
      msg.stop_s = std::max(0.0, msg.ego_s_in - config_.stop_margin);
      msg.max_speed =
          (vehicle_index < result.speed_limits.size() && std::isfinite(result.speed_limits[vehicle_index]))
              ? result.speed_limits[vehicle_index]
              : 100.0;
      msg.decision_locked = conflict.decision_locked;
      msg.decision_source = conflict.decision_source;
      msg.decision_reason = conflict.decision_reason;
      return msg;
    }

    return msg;
  };

  const bool ego_has_runtime_state =
      ego_index_ < 0 ||
      (static_cast<size_t>(ego_index_) < vehicles_.size() &&
       (vehicles_[static_cast<size_t>(ego_index_)].have_candidate ||
        vehicles_[static_cast<size_t>(ego_index_)].agent.have_pose));
  if (publish_debug_ && ego_has_runtime_state)
  {
    conflict_markers_pub_.publish(makeConflictMarkers(result, stamp));
  }

  if (result.ready && result.conflict_active)
  {
    for (const auto& conflict : result.conflicts)
    {
      if (conflict.proceed_index < 0 || conflict.yield_index < 0 ||
          static_cast<size_t>(conflict.proceed_index) >= vehicles_.size() ||
          static_cast<size_t>(conflict.yield_index) >= vehicles_.size())
      {
        continue;
      }
      if (ego_index_ >= 0 && ego_index_ != conflict.proceed_index && ego_index_ != conflict.yield_index)
      {
        continue;
      }

      ROS_INFO_STREAM_THROTTLE(
          1.0,
          "conflict decision ego=" << (ego_index_ >= 0 ? vehicles_[static_cast<size_t>(ego_index_)].agent.id : "central")
                                   << " pair=(" << vehicles_[static_cast<size_t>(conflict.first_index)].agent.id
                                   << "," << vehicles_[static_cast<size_t>(conflict.second_index)].agent.id
                                   << ") proceed=" << vehicles_[static_cast<size_t>(conflict.proceed_index)].agent.id
                                   << " yield=" << vehicles_[static_cast<size_t>(conflict.yield_index)].agent.id
                                   << " source=" << conflict.decision_source
                                   << " reason=" << conflict.decision_reason
                                   << " score=(" << conflict.first_score << "," << conflict.second_score << ")"
                                   << " locked=" << std::boolalpha << conflict.decision_locked);
    }
  }

  if (!result.ready)
  {
    // 启动阶段车辆仿真需要先拿到一条轨迹，才能发布自身位姿；
    // 此时协调器还没等到位姿，所以先透传候选轨迹，打破初始化等待。
    for (size_t i = 0; i < vehicles_.size(); ++i)
    {
      auto& vehicle = vehicles_[i];
      if (!shouldPublishVehicle(i))
      {
        continue;
      }
      if (!vehicle.have_candidate)
      {
        continue;
      }

      planning_msgs::TrajectoryPointArray approved = vehicle.latest_candidate;
      approved.header.stamp = stamp;
      vehicle.approved_pub.publish(approved);

      std_msgs::Float64 speed_limit_msg;
      speed_limit_msg.data = 100.0;
      vehicle.speed_limit_pub.publish(speed_limit_msg);
      vehicle.constraint_pub.publish(make_constraint_msg(i));
    }
    return;
  }

  for (size_t i = 0; i < vehicles_.size() && i < result.approved_trajectories.size(); ++i)
  {
    if (!shouldPublishVehicle(i))
    {
      continue;
    }
    const planning_msgs::TrajectoryPointArray approved =
        fromCoreTrajectory(result.approved_trajectories[i], vehicles_[i].latest_candidate, stamp);
    vehicles_[i].approved_pub.publish(approved);

    std_msgs::Float64 speed_limit_msg;
    speed_limit_msg.data = std::isfinite(result.speed_limits[i]) ? result.speed_limits[i] : 100.0;
    vehicles_[i].speed_limit_pub.publish(speed_limit_msg);
    vehicles_[i].constraint_pub.publish(make_constraint_msg(i));
  }
}

bool ConflictResolverNode::shouldPublishVehicle(const size_t index) const
{
  return ego_index_ < 0 || static_cast<size_t>(ego_index_) == index;
}

visualization_msgs::MarkerArray ConflictResolverNode::makeConflictMarkers(
    const coordination::CoordinationResult& result,
    const ros::Time& stamp) const
{
  visualization_msgs::MarkerArray markers;
  const int marker_id_base = ego_index_ >= 0 ? 1000 * (ego_index_ + 1) : 900000;
  int marker_id = marker_id_base;
  const double status_y_offset = ego_index_ >= 0 ? -1.9 * ego_index_ : 0.0;
  const auto ego_label = [&]() {
    if (ego_index_ >= 0 && static_cast<size_t>(ego_index_) < vehicles_.size())
    {
      return vehicles_[static_cast<size_t>(ego_index_)].agent.id;
    }
    return std::string("central");
  };
  const auto make_status_text = [&]() {
    if (!result.ready)
    {
      return ego_label() + ": WAITING_FOR_TRAJECTORY_OR_POSE";
    }
    if (!result.conflict_active)
    {
      return ego_label() + ": CLEAR";
    }

    std::ostringstream text;
    text.precision(1);
    text << std::fixed << ego_label() << ": CONFLICTS " << result.conflicts.size();
    for (const auto& conflict : result.conflicts)
    {
      if (conflict.proceed_index < 0 || conflict.yield_index < 0 ||
          static_cast<size_t>(conflict.proceed_index) >= vehicles_.size() ||
          static_cast<size_t>(conflict.yield_index) >= vehicles_.size())
      {
        continue;
      }

      if (ego_index_ >= 0 && ego_index_ != conflict.proceed_index && ego_index_ != conflict.yield_index)
      {
        continue;
      }

      const int ego_or_first = ego_index_ >= 0 ? ego_index_ : conflict.first_index;
      const int other_index = ego_index_ >= 0
                                  ? (ego_index_ == conflict.first_index ? conflict.second_index : conflict.first_index)
                                  : conflict.second_index;
      const bool ego_yields = ego_or_first == conflict.yield_index;
      const std::string& other_id = vehicles_[static_cast<size_t>(other_index)].agent.id;
      text << "\n"
           << (ego_yields ? "YIELD vs " : "GO vs ")
           << other_id
           << " t=" << conflict.conflict_time
           << " s[" << sInFor(conflict, ego_or_first)
           << "," << sOutFor(conflict, ego_or_first) << "]"
           << " tw[" << tInFor(conflict, ego_or_first)
           << "," << tOutFor(conflict, ego_or_first) << "]";
      if (conflict.decision_locked)
      {
        text << " LOCK";
      }
    }
    return text.str();
  };

  visualization_msgs::Marker status =
      baseMarker(frame_id_, "conflict_state", marker_id++, visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
  status.pose.position = makePoint(14.5, 13.0 + status_y_offset, 2.4);
  status.scale.z = 0.34;
  status.color = result.conflict_active ? makeColor(0.92, 0.34, 0.04, 1.0)
                                        : makeColor(0.05, 0.45, 0.20, 1.0);
  status.text = make_status_text();
  markers.markers.push_back(status);

  if (!result.ready)
  {
    return markers;
  }

  if (!result.conflict_active)
  {
    return markers;
  }

  for (const auto& conflict : result.conflicts)
  {
    if (conflict.proceed_index < 0 || conflict.yield_index < 0 ||
        static_cast<size_t>(conflict.proceed_index) >= vehicles_.size() ||
        static_cast<size_t>(conflict.yield_index) >= vehicles_.size())
    {
      continue;
    }

    const VehicleIo& proceed_vehicle = vehicles_[static_cast<size_t>(conflict.proceed_index)];
    const VehicleIo& yield_vehicle = vehicles_[static_cast<size_t>(conflict.yield_index)];
    const double yield_limit =
        (static_cast<size_t>(conflict.yield_index) < result.speed_limits.size() &&
         std::isfinite(result.speed_limits[static_cast<size_t>(conflict.yield_index)]))
            ? result.speed_limits[static_cast<size_t>(conflict.yield_index)]
            : 100.0;

    const auto add_section_marker = [&](const planning_msgs::TrajectoryPointArray& trajectory,
                                        const std::string& label,
                                        const double s_in,
                                        const double s_out,
                                        const std_msgs::ColorRGBA& color,
                                        const double z) {
      visualization_msgs::Marker section =
          baseMarker(frame_id_, "conflict_s_intervals", marker_id++, visualization_msgs::Marker::LINE_STRIP, stamp);
      section.points = makeTrajectorySectionPoints(trajectory, s_in, s_out, z);
      if (section.points.size() < 2)
      {
        return;
      }
      section.scale.x = 0.34;
      section.color = color;
      section.text = label;
      markers.markers.push_back(section);
    };

    const auto add_boundary_marker = [&](const coordination::Pose2d& pose,
                                         const std::string& vehicle_id,
                                         const std::string& label,
                                         const double s,
                                         const double time,
                                         const std_msgs::ColorRGBA& color,
                                         const int shape) {
      visualization_msgs::Marker point =
          baseMarker(frame_id_, "conflict_s_boundaries", marker_id++, shape, stamp);
      point.pose.position = makeMarkerPoint(pose, 0.42);
      point.scale.x = 0.46;
      point.scale.y = 0.46;
      point.scale.z = 0.46;
      point.color = color;
      markers.markers.push_back(point);

      visualization_msgs::Marker text =
          baseMarker(frame_id_, "conflict_s_boundaries", marker_id++, visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
      text.pose.position = makeMarkerPoint(pose, 1.05);
      text.scale.z = 0.32;
      text.color = color;
      std::ostringstream boundary_text;
      boundary_text.precision(1);
      boundary_text << std::fixed << vehicle_id << "\n" << label << "=" << s << "\nt=" << time;
      text.text = boundary_text.str();
      markers.markers.push_back(text);
    };

    const auto add_vehicle_interval = [&](const int vehicle_index, const bool yielding) {
      if (vehicle_index < 0 || static_cast<size_t>(vehicle_index) >= vehicles_.size())
      {
        return;
      }

      const VehicleIo& vehicle = vehicles_[static_cast<size_t>(vehicle_index)];
      const double s_in = sInFor(conflict, vehicle_index);
      const double s_out = sOutFor(conflict, vehicle_index);
      const std_msgs::ColorRGBA solid_color = yielding ? makeColor(0.95, 0.10, 0.05, 0.95)
                                                       : makeColor(0.05, 0.70, 0.25, 0.92);
      const std_msgs::ColorRGBA soft_color = yielding ? makeColor(0.95, 0.38, 0.05, 0.70)
                                                      : makeColor(0.05, 0.70, 0.25, 0.70);

      add_section_marker(vehicle.latest_candidate,
                         yielding ? "YIELD_INTERVAL" : "PROCEED_INTERVAL",
                         s_in,
                         s_out,
                         solid_color,
                         yielding ? 0.72 : 0.62);
      add_boundary_marker(entryPoseFor(conflict, vehicle_index),
                          vehicle.agent.id,
                          "s_in",
                          s_in,
                          tInFor(conflict, vehicle_index),
                          solid_color,
                          visualization_msgs::Marker::SPHERE);
      add_boundary_marker(exitPoseFor(conflict, vehicle_index),
                          vehicle.agent.id,
                          "s_out",
                          s_out,
                          tOutFor(conflict, vehicle_index),
                          soft_color,
                          visualization_msgs::Marker::CUBE);
    };

    // 分布式模式下，每个 resolver 只画本车自己的冲突区间；对手车区间由对手车节点生成。
    if (ego_index_ < 0)
    {
      add_vehicle_interval(conflict.proceed_index, false);
      add_vehicle_interval(conflict.yield_index, true);
    }
    else if (ego_index_ == conflict.proceed_index)
    {
      add_vehicle_interval(conflict.proceed_index, false);
    }
    else if (ego_index_ == conflict.yield_index)
    {
      add_vehicle_interval(conflict.yield_index, true);
    }

    visualization_msgs::Marker proceed_label =
        baseMarker(frame_id_, "conflict_state", marker_id++, visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
    proceed_label.pose.position = makePoint(proceed_vehicle.agent.pose.x,
                                            proceed_vehicle.agent.pose.y,
                                            2.15);
    proceed_label.scale.z = 0.42;
    proceed_label.color = makeColor(0.02, 0.45, 0.16, 1.0);
    proceed_label.text = "PROCEED";
    markers.markers.push_back(proceed_label);

    visualization_msgs::Marker yield_label =
        baseMarker(frame_id_, "conflict_state", marker_id++, visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
    yield_label.pose.position = makePoint(yield_vehicle.agent.pose.x,
                                          yield_vehicle.agent.pose.y,
                                          2.15);
    yield_label.scale.z = 0.42;
    yield_label.color = makeColor(0.82, 0.25, 0.02, 1.0);
    std::ostringstream yield_text;
    yield_text.precision(2);
    yield_text << std::fixed << "YIELD\nlimit " << yield_limit << " m/s";
    yield_label.text = yield_text.str();
    markers.markers.push_back(yield_label);
  }
  return markers;
}

}  // namespace structured_road_conflict_sim

int main(int argc, char** argv)
{
  ros::init(argc, argv, "conflict_resolver_node");
  structured_road_conflict_sim::ConflictResolverNode node;
  ros::spin();
  return 0;
}
