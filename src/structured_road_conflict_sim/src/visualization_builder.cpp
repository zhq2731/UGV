#include "structured_road_conflict_sim/visualization_builder.hpp"

#include <cmath>
#include <sstream>

namespace structured_road_conflict_sim
{
namespace
{

visualization_msgs::Marker baseMarker(const std::string& frame_id,
                                      const std::string& ns,
                                      const int id,
                                      const int type,
                                      const ros::Time& stamp)
{
  // Marker 公共字段集中初始化，避免各类可视化对象重复设置 header/ns/id。
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

void appendLinePoint(visualization_msgs::Marker& marker,
                     const double x,
                     const double y,
                     const double z)
{
  marker.points.push_back(makePoint(x, y, z));
}

std::vector<geometry_msgs::Point> rectangleCorners(const Pose2d& pose,
                                                   const double vehicle_length,
                                                   const double vehicle_width,
                                                   const double z)
{
  // 根据车辆中心位姿计算矩形四角，供车体线框和预测 footprint 使用。
  const double half_length = 0.5 * vehicle_length;
  const double half_width = 0.5 * vehicle_width;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);

  const double local_x[4] = {half_length, half_length, -half_length, -half_length};
  const double local_y[4] = {half_width, -half_width, -half_width, half_width};

  std::vector<geometry_msgs::Point> corners;
  corners.reserve(4);
  for (int i = 0; i < 4; ++i)
  {
    const double x = pose.x + local_x[i] * cos_yaw - local_y[i] * sin_yaw;
    const double y = pose.y + local_x[i] * sin_yaw + local_y[i] * cos_yaw;
    corners.push_back(makePoint(x, y, z));
  }
  return corners;
}

}  // namespace

visualization_msgs::MarkerArray VisualizationBuilder::makeRoadMarkers(const RoadNetwork& road,
                                                                      const ros::Time& stamp) const
{
  visualization_msgs::MarkerArray markers;
  int id = 0;

  // 四车场景使用两条平行横向车道和两条平行纵向车道。
  const double lane_offset = road.laneWidth();
  const double road_center_offset = 0.5 * lane_offset;

  visualization_msgs::Marker east_west =
      baseMarker(road.frameId(), "road_surface", id++, visualization_msgs::Marker::CUBE, stamp);
  east_west.pose.position = makePoint(0.0, road_center_offset, -0.03);
  east_west.scale.x = 2.0 * road.laneLength();
  east_west.scale.y = 2.0 * road.laneWidth();
  east_west.scale.z = 0.04;
  east_west.color = makeColor(0.28, 0.29, 0.31, 0.85);
  markers.markers.push_back(east_west);

  visualization_msgs::Marker south_north =
      baseMarker(road.frameId(), "road_surface", id++, visualization_msgs::Marker::CUBE, stamp);
  south_north.pose.position = makePoint(-road_center_offset, 0.0, -0.02);
  south_north.scale.x = 2.0 * road.laneWidth();
  south_north.scale.y = 2.0 * road.laneLength();
  south_north.scale.z = 0.04;
  south_north.color = makeColor(0.30, 0.31, 0.33, 0.85);
  markers.markers.push_back(south_north);

  const double horizontal_lanes[2] = {0.0, lane_offset};
  for (const double y : horizontal_lanes)
  {
    visualization_msgs::Marker center =
        baseMarker(road.frameId(), "center_lines", id++, visualization_msgs::Marker::LINE_STRIP, stamp);
    center.scale.x = 0.08;
    center.color = makeColor(1.0, 0.88, 0.15, 0.95);
    appendLinePoint(center, -road.laneLength(), y, 0.05);
    appendLinePoint(center, road.laneLength(), y, 0.05);
    markers.markers.push_back(center);
  }

  const double vertical_lanes[2] = {0.0, -lane_offset};
  for (const double x : vertical_lanes)
  {
    visualization_msgs::Marker center =
        baseMarker(road.frameId(), "center_lines", id++, visualization_msgs::Marker::LINE_STRIP, stamp);
    center.scale.x = 0.08;
    center.color = makeColor(1.0, 0.88, 0.15, 0.95);
    appendLinePoint(center, x, -road.laneLength(), 0.05);
    appendLinePoint(center, x, road.laneLength(), 0.05);
    markers.markers.push_back(center);
  }

  return markers;
}

visualization_msgs::MarkerArray VisualizationBuilder::makeVehicleMarkers(
    const std::string& vehicle_id,
    const Pose2d& pose,
    const double vehicle_length,
    const double vehicle_width,
    const std_msgs::ColorRGBA& color,
    const std::string& frame_id,
    const ros::Time& stamp) const
{
  visualization_msgs::MarkerArray markers;

  visualization_msgs::Marker body =
      baseMarker(frame_id, vehicle_id, 0, visualization_msgs::Marker::CUBE, stamp);
  // CUBE 的 pose 使用车辆中心位姿，scale 表示车辆长宽高。
  body.pose = makePose(pose);
  body.pose.position.z = 0.45;
  body.scale.x = vehicle_length;
  body.scale.y = vehicle_width;
  body.scale.z = 1.0;
  body.color = color;
  markers.markers.push_back(body);

  visualization_msgs::Marker heading =
      baseMarker(frame_id, vehicle_id, 1, visualization_msgs::Marker::ARROW, stamp);
  heading.pose = makePose(pose);
  heading.pose.position.z = 1.08;
  heading.scale.x = vehicle_length * 0.58;
  heading.scale.y = 0.14;
  heading.scale.z = 0.14;
  heading.color = makeColor(0.98, 0.98, 0.98, 0.95);
  markers.markers.push_back(heading);

  visualization_msgs::Marker label =
      baseMarker(frame_id, vehicle_id, 2, visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
  label.pose.position = makePoint(pose.x, pose.y, 1.8);
  label.scale.z = 0.55;
  label.color = makeColor(0.02, 0.02, 0.02, 1.0);
  label.text = vehicle_id;
  markers.markers.push_back(label);

  return markers;
}

visualization_msgs::MarkerArray VisualizationBuilder::makeConflictMarkers(
    const RoadNetwork& road,
    const std::string& status_text,
    const bool conflict_active,
    const bool vehicle_holding,
    const bool has_collision_point,
    const Pose2d& collision_point,
    const double conflict_time,
    const ros::Time& stamp) const
{
  visualization_msgs::MarkerArray markers;

  visualization_msgs::Marker zone =
      baseMarker(road.frameId(), "conflict_state", 0, visualization_msgs::Marker::CYLINDER, stamp);
  zone.pose.position = makePoint(0.0, 0.0, 0.12);
  zone.scale.x = 2.0 * road.conflictRadius();
  zone.scale.y = 2.0 * road.conflictRadius();
  zone.scale.z = 0.08;
  if (vehicle_holding)
  {
    // 红色表示已经进入强制等待或停车状态。
    zone.color = makeColor(1.0, 0.08, 0.05, 0.45);
  }
  else if (conflict_active)
  {
    // 橙色表示预测到冲突，但仍在通过限速/重定时处理。
    zone.color = makeColor(1.0, 0.65, 0.03, 0.42);
  }
  else
  {
    // 绿色表示当前预测窗口内未发现冲突。
    zone.color = makeColor(0.10, 0.72, 0.36, 0.28);
  }
  markers.markers.push_back(zone);

  visualization_msgs::Marker text =
      baseMarker(road.frameId(), "conflict_state", 1, visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
  text.pose.position = makePoint(0.0, 4.4, 1.4);
  text.scale.z = 0.7;
  text.color = makeColor(0.02, 0.02, 0.02, 1.0);
  text.text = status_text;
  markers.markers.push_back(text);

  visualization_msgs::Marker collision =
      baseMarker(road.frameId(), "collision_point", 2, visualization_msgs::Marker::SPHERE, stamp);
  collision.pose.position = makePoint(collision_point.x, collision_point.y, 0.45);
  collision.scale.x = 0.65;
  collision.scale.y = 0.65;
  collision.scale.z = 0.65;
  collision.color = has_collision_point ? makeColor(1.0, 0.0, 0.0, 0.95)
                                        : makeColor(1.0, 0.0, 0.0, 0.0);
  markers.markers.push_back(collision);

  visualization_msgs::Marker collision_text =
      baseMarker(road.frameId(), "collision_point", 3, visualization_msgs::Marker::TEXT_VIEW_FACING, stamp);
  collision_text.pose.position = makePoint(collision_point.x, collision_point.y, 1.15);
  collision_text.scale.z = 0.45;
  collision_text.color = has_collision_point ? makeColor(0.80, 0.0, 0.0, 1.0)
                                             : makeColor(0.80, 0.0, 0.0, 0.0);
  std::ostringstream label;
  label.precision(2);
  label << std::fixed << "predicted collision t=" << conflict_time << "s";
  collision_text.text = label.str();
  markers.markers.push_back(collision_text);

  return markers;
}

visualization_msgs::MarkerArray VisualizationBuilder::makeTrajectoryFootprintMarkers(
    const std::string& marker_namespace,
    const std::vector<Pose2d>& poses,
    const double vehicle_length,
    const double vehicle_width,
    const std_msgs::ColorRGBA& color,
    const std::string& frame_id,
    const ros::Time& stamp) const
{
  visualization_msgs::MarkerArray markers;

  visualization_msgs::Marker footprints =
      baseMarker(frame_id, marker_namespace, 0, visualization_msgs::Marker::LINE_LIST, stamp);
  footprints.scale.x = 0.045;
  footprints.color = color;
  footprints.pose.orientation.w = 1.0;

  for (const Pose2d& pose : poses)
  {
    const auto corners = rectangleCorners(pose, vehicle_length, vehicle_width, 0.22);
    // LINE_LIST 每两点组成一条边，四条边闭合成一个 footprint 矩形。
    for (size_t i = 0; i < corners.size(); ++i)
    {
      footprints.points.push_back(corners[i]);
      footprints.points.push_back(corners[(i + 1) % corners.size()]);
    }
  }

  markers.markers.push_back(footprints);
  return markers;
}

}  // namespace structured_road_conflict_sim
