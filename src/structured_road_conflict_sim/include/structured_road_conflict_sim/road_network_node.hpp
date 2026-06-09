#pragma once

#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "structured_road_conflict_sim/road_network.hpp"
#include "structured_road_conflict_sim/visualization_builder.hpp"

namespace structured_road_conflict_sim
{

class RoadNetworkNode
{
public:
  RoadNetworkNode();

private:
  struct RouteIo
  {
    // 将一条车辆路线和对应的两个发布器绑定在一起，便于定时统一发布。
    VehicleRoute route;
    ros::Publisher candidate_pub;
    ros::Publisher path_pub;
  };

  void loadRoutes();
  void publishAll(const ros::TimerEvent& event);
  void publishVehicleRoute(const VehicleRoute& route,
                           const ros::Publisher& candidate_pub,
                           const ros::Publisher& path_pub,
                           const ros::Time& stamp);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher road_markers_pub_;
  ros::Timer timer_;

  RoadNetwork road_;
  VisualizationBuilder visualization_;
  std::vector<RouteIo> route_ios_;

  std::string road_markers_topic_ = "/structured_road/markers";
  double publish_rate_ = 1.0;
};

}  // namespace structured_road_conflict_sim
