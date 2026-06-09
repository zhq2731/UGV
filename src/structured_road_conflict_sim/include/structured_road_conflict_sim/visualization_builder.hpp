#pragma once

#include <string>
#include <vector>

#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

#include "structured_road_conflict_sim/common.hpp"
#include "structured_road_conflict_sim/road_network.hpp"

namespace structured_road_conflict_sim
{

class VisualizationBuilder
{
public:
  // 构造静态路网、中心线、停止线和冲突区 Marker。
  visualization_msgs::MarkerArray makeRoadMarkers(const RoadNetwork& road,
                                                  const ros::Time& stamp) const;

  // 构造单车车体、朝向箭头和文字标签。
  visualization_msgs::MarkerArray makeVehicleMarkers(const std::string& vehicle_id,
                                                     const Pose2d& pose,
                                                     double vehicle_length,
                                                     double vehicle_width,
                                                     const std_msgs::ColorRGBA& color,
                                                     const std::string& frame_id,
                                                     const ros::Time& stamp) const;

  // 构造冲突状态区和预测碰撞点，用于 RViz 调试协调逻辑。
  visualization_msgs::MarkerArray makeConflictMarkers(const RoadNetwork& road,
                                                      const std::string& status_text,
                                                      bool conflict_active,
                                                      bool vehicle_holding,
                                                      bool has_collision_point,
                                                      const Pose2d& collision_point,
                                                      double conflict_time,
                                                      const ros::Time& stamp) const;

  // 将预测轨迹上的车辆 footprint 画成线框，便于检查时空占用。
  visualization_msgs::MarkerArray makeTrajectoryFootprintMarkers(const std::string& marker_namespace,
                                                                 const std::vector<Pose2d>& poses,
                                                                 double vehicle_length,
                                                                 double vehicle_width,
                                                                 const std_msgs::ColorRGBA& color,
                                                                 const std::string& frame_id,
                                                                 const ros::Time& stamp) const;
};

}  // namespace structured_road_conflict_sim
