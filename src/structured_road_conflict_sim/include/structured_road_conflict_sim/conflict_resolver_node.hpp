#pragma once

#include <string>
#include <vector>

#include <driver_msgs/ChassisReport.h>
#include <localization_msgs/Localization.h>
#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>

#include "structured_road_conflict_sim/coordination_core.hpp"
#include "structured_road_conflict_sim/visualization_builder.hpp"

namespace structured_road_conflict_sim
{

class ConflictResolverNode
{
public:
  ConflictResolverNode();

private:
  struct VehicleIo
  {
    // 单车在协调器中的 ROS 接口和最近一次候选轨迹缓存。
    coordination::VehicleAgent agent;
    planning_msgs::TrajectoryPointArray latest_candidate;
    ros::Subscriber localization_sub;
    ros::Subscriber chassis_sub;
    ros::Subscriber candidate_sub;
    ros::Publisher approved_pub;
    ros::Publisher speed_limit_pub;
    // 发布给速度规划节点的结构化 ST 约束，补充传统标量 speed_limit 难以表达的让行窗口。
    ros::Publisher constraint_pub;
    bool have_candidate = false;
  };

  void loadVehicles();
  void setupRosInterfaces();
  void onLocalization(size_t index, const localization_msgs::Localization::ConstPtr& msg);
  void onChassis(size_t index, const driver_msgs::ChassisReport::ConstPtr& msg);
  void onCandidate(size_t index, const planning_msgs::TrajectoryPointArray::ConstPtr& msg);
  void onTimer(const ros::TimerEvent& event);

  // ROS 消息轨迹和纯算法轨迹之间做轻量转换，保持协调核心不依赖 ROS。
  coordination::CoordinatorConfig loadCoordinatorConfig() const;
  coordination::Trajectory toCoreTrajectory(const planning_msgs::TrajectoryPointArray& msg) const;
  planning_msgs::TrajectoryPointArray fromCoreTrajectory(const coordination::Trajectory& trajectory,
                                                         const planning_msgs::TrajectoryPointArray& original,
                                                         const ros::Time& stamp) const;
  void publishResult(const coordination::CoordinationResult& result, const ros::Time& stamp);
  visualization_msgs::MarkerArray makeConflictMarkers(const coordination::CoordinationResult& result,
                                                       const ros::Time& stamp) const;
  bool shouldPublishVehicle(size_t index) const;

  template <typename T>
  void readParam(const std::string& name, T& value, const T& default_value) const
  {
    if (private_nh_.getParam(name, value))
    {
      return;
    }
    if (nh_.getParam("/conflict_resolver_node/" + name, value))
    {
      return;
    }
    value = default_value;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher conflict_markers_pub_;
  ros::Timer timer_;

  std::vector<VehicleIo> vehicles_;
  // config_ 和 coordinator_ 分离：参数读取在节点层，冲突决策在纯算法层。
  coordination::CoordinatorConfig config_;
  coordination::MultiVehicleCoordinator coordinator_;
  VisualizationBuilder visualization_;

  std::string frame_id_ = "map";
  std::string conflict_markers_topic_ = "/trajectory_conflict/markers";
  double decision_rate_ = 20.0;
  double heading_compensation_degree_ = 0.0;
  int ego_index_ = -1;
  bool publish_debug_ = true;
};

}  // namespace structured_road_conflict_sim
