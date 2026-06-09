#include "structured_road_conflict_sim/road_network_node.hpp"

#include <algorithm>

#include "structured_road_conflict_sim/trajectory_utils.hpp"

namespace structured_road_conflict_sim
{

RoadNetworkNode::RoadNetworkNode() : private_nh_("~")
{
  // 私有参数来自 launch/yaml，用于覆盖默认路网和发布频率。
  road_.loadFromRosParams(private_nh_);
  private_nh_.param<std::string>("road_markers_topic", road_markers_topic_, road_markers_topic_);
  private_nh_.param("publish_rate", publish_rate_, publish_rate_);

  loadRoutes();
  road_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(road_markers_topic_, 1, true);

  timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, publish_rate_)),
                           &RoadNetworkNode::publishAll,
                           this);
  publishAll(ros::TimerEvent());

  ROS_INFO_STREAM("Road network node started with " << route_ios_.size() << " vehicle routes");
}

void RoadNetworkNode::loadRoutes()
{
  int vehicle_count = 2;
  private_nh_.param("vehicle_count", vehicle_count, vehicle_count);
  if (vehicle_count <= 0)
  {
    vehicle_count = road_.defaultVehicleCount();
  }
  vehicle_count = std::max(1, vehicle_count);
  route_ios_.clear();
  route_ios_.reserve(static_cast<size_t>(vehicle_count));

  for (int i = 0; i < vehicle_count; ++i)
  {
    RouteIo io;
    // 先生成默认路线，再用 vehicles/i 下的参数覆盖 id、优先级和速度。
    io.route = road_.makeVehicleRoute(i);

    const std::string prefix = "vehicles/" + std::to_string(i) + "/";
    private_nh_.param<std::string>(prefix + "id", io.route.id, io.route.id);
    private_nh_.param(prefix + "priority", io.route.priority, io.route.priority);
    private_nh_.param(prefix + "nominal_speed", io.route.nominal_speed, io.route.nominal_speed);

    std::string candidate_topic = "/" + io.route.id + "/trajectory_candidate";
    std::string path_topic = "/" + io.route.id + "/planned_path";
    // 话题名也允许在配置文件中覆盖，便于和不同仿真节点对接。
    private_nh_.param<std::string>(prefix + "candidate_topic", candidate_topic, candidate_topic);
    private_nh_.param<std::string>(prefix + "path_topic", path_topic, path_topic);

    io.candidate_pub = nh_.advertise<planning_msgs::TrajectoryPointArray>(candidate_topic, 1, true);
    io.path_pub = nh_.advertise<nav_msgs::Path>(path_topic, 1, true);
    route_ios_.push_back(io);
  }
}

void RoadNetworkNode::publishVehicleRoute(const VehicleRoute& route,
                                          const ros::Publisher& candidate_pub,
                                          const ros::Publisher& path_pub,
                                          const ros::Time& stamp)
{
  // 同一条参考线同时发布为轨迹候选和 nav_msgs/Path，分别服务规划和可视化。
  const auto trajectory =
      makeTrajectory(road_.frameId(), route.id, route.reference_points, route.nominal_speed, stamp);
  candidate_pub.publish(trajectory);
  path_pub.publish(makePath(trajectory));
}

void RoadNetworkNode::publishAll(const ros::TimerEvent&)
{
  const ros::Time stamp = ros::Time::now();
  // 所有消息使用同一个时间戳，方便 RViz 和下游节点对齐显示。
  for (const auto& io : route_ios_)
  {
    publishVehicleRoute(io.route, io.candidate_pub, io.path_pub, stamp);
  }
  road_markers_pub_.publish(visualization_.makeRoadMarkers(road_, stamp));
}

}  // namespace structured_road_conflict_sim

int main(int argc, char** argv)
{
  ros::init(argc, argv, "road_network_node");
  structured_road_conflict_sim::RoadNetworkNode node;
  ros::spin();
  return 0;
}
