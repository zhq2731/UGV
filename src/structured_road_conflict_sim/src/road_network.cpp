#include "structured_road_conflict_sim/road_network.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>

namespace structured_road_conflict_sim
{
namespace
{

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

void appendWithoutDuplicate(std::vector<Pose2d>& target, const std::vector<Pose2d>& source)
{
  for (const Pose2d& point : source)
  {
    if (!target.empty() && distance2d(target.back().x, target.back().y, point.x, point.y) < 1.0e-6)
    {
      continue;
    }
    target.push_back(point);
  }
}

}  // namespace

void RoadNetwork::loadFromRosParams(const ros::NodeHandle& private_nh)
{
  // 先读取全局道路参数，再读取每辆车自己的速度配置。
  private_nh.param<std::string>("frame_id", frame_id_, frame_id_);
  private_nh.param<std::string>("scenario", scenario_, scenario_);
  scenario_ = normalizeScenarioName(scenario_);
  private_nh.param<std::string>("left_turn_straight_relation",
                                left_turn_straight_relation_,
                                left_turn_straight_relation_);
  left_turn_straight_relation_ = normalizeScenarioName(left_turn_straight_relation_);
  private_nh.param("lane_length", lane_length_, lane_length_);
  private_nh.param("lane_width", lane_width_, lane_width_);
  private_nh.param("sample_spacing", sample_spacing_, sample_spacing_);
  private_nh.param("conflict_radius", conflict_radius_, conflict_radius_);
  private_nh.param("stop_line_distance", stop_line_distance_, stop_line_distance_);
  private_nh.param("vehicle_1/nominal_speed", vehicle_1_speed_, vehicle_1_speed_);
  private_nh.param("vehicle_2/nominal_speed", vehicle_2_speed_, vehicle_2_speed_);

  int vehicle_count = 2;
  private_nh.param("vehicle_count", vehicle_count, vehicle_count);
  if (vehicle_count <= 0)
  {
    vehicle_count = defaultVehicleCount();
  }
  vehicle_count = std::max(1, vehicle_count);
  vehicle_speeds_.assign(static_cast<size_t>(vehicle_count), 2.0);
  for (int i = 0; i < vehicle_count; ++i)
  {
    const std::string prefix = "vehicles/" + std::to_string(i) + "/";
    // 兼容旧参数 vehicle_1/vehicle_2，同时支持 vehicles/N 新格式。
    const double fallback_speed = (i == 0) ? vehicle_1_speed_ : vehicle_2_speed_;
    vehicle_speeds_[static_cast<size_t>(i)] = fallback_speed;
    private_nh.param(prefix + "nominal_speed",
                     vehicle_speeds_[static_cast<size_t>(i)],
                     vehicle_speeds_[static_cast<size_t>(i)]);
  }
}

const std::string& RoadNetwork::frameId() const
{
  return frame_id_;
}

const std::string& RoadNetwork::scenario() const
{
  return scenario_;
}

int RoadNetwork::defaultVehicleCount() const
{
  if (scenario_ == "four_way_straight")
  {
    return 4;
  }
  if (scenario_ == "unprotected_left_turn" || scenario_ == "left_right_turn_conflict")
  {
    return 2;
  }
  return 2;
}

double RoadNetwork::laneLength() const
{
  return lane_length_;
}

double RoadNetwork::laneWidth() const
{
  return lane_width_;
}

double RoadNetwork::sampleSpacing() const
{
  return sample_spacing_;
}

double RoadNetwork::conflictRadius() const
{
  return conflict_radius_;
}

double RoadNetwork::stopLineDistance() const
{
  return stop_line_distance_;
}

Pose2d RoadNetwork::conflictCenter() const
{
  return Pose2d{0.0, 0.0, 0.0};
}

VehicleRoute RoadNetwork::makeVehicle1Route() const
{
  VehicleRoute route;
  route.id = "vehicle_1";
  route.priority = 0;
  route.nominal_speed = vehicle_1_speed_;
  route.reference_points =
      sampleStraightRoute(Pose2d{-lane_length_, 0.0, 0.0}, Pose2d{lane_length_, 0.0, 0.0});
  return route;
}

VehicleRoute RoadNetwork::makeVehicle2Route() const
{
  VehicleRoute route;
  route.id = "vehicle_2";
  route.priority = 1;
  route.nominal_speed = vehicle_2_speed_;
  route.reference_points = sampleStraightRoute(Pose2d{0.0, -lane_length_, M_PI_2},
                                               Pose2d{0.0, lane_length_, M_PI_2});
  return route;
}

VehicleRoute RoadNetwork::makeVehicleRoute(const int index) const
{
  if (scenario_ == "unprotected_left_turn")
  {
    return makeUnprotectedLeftTurnRoute(index);
  }
  if (scenario_ == "left_right_turn_conflict")
  {
    return makeLeftRightTurnConflictRoute(index);
  }
  return makeFourWayStraightRoute(index);
}

VehicleRoute RoadNetwork::makeFourWayStraightRoute(const int index) const
{
  VehicleRoute route;
  route.id = "vehicle_" + std::to_string(index + 1);
  route.priority = index;
  if (index >= 0 && static_cast<size_t>(index) < vehicle_speeds_.size())
  {
    route.nominal_speed = vehicle_speeds_[static_cast<size_t>(index)];
  }

  // 按车辆编号循环分配四个进入方向，形成可扩展的十字路口冲突场景。
  switch (index % 4)
  {
    case 0:
      route.reference_points =
          sampleStraightRoute(Pose2d{-lane_length_, 0.0, 0.0}, Pose2d{lane_length_, 0.0, 0.0});
      break;
    case 1:
      route.reference_points = sampleStraightRoute(Pose2d{0.0, -lane_length_, M_PI_2},
                                                   Pose2d{0.0, lane_length_, M_PI_2});
      break;
    case 2:
      route.reference_points =
          sampleStraightRoute(Pose2d{lane_length_, lane_width_, M_PI}, Pose2d{-lane_length_, lane_width_, M_PI});
      break;
    default:
      route.reference_points = sampleStraightRoute(Pose2d{-lane_width_, lane_length_, -M_PI_2},
                                                   Pose2d{-lane_width_, -lane_length_, -M_PI_2});
      break;
  }
  return route;
}

VehicleRoute RoadNetwork::makeUnprotectedLeftTurnRoute(const int index) const
{
  VehicleRoute route;
  route.id = "vehicle_" + std::to_string(index + 1);
  route.priority = index;
  if (index >= 0 && static_cast<size_t>(index) < vehicle_speeds_.size())
  {
    route.nominal_speed = vehicle_speeds_[static_cast<size_t>(index)];
  }

  if (index == 0)
  {
    route.reference_points = makeUnprotectedLeftTurnStraightPath();
    return route;
  }

  if (index == 1)
  {
    route.reference_points = makeUnprotectedLeftTurnPath();
    return route;
  }

  return makeFourWayStraightRoute(index);
}

VehicleRoute RoadNetwork::makeLeftRightTurnConflictRoute(const int index) const
{
  VehicleRoute route;
  route.id = "vehicle_" + std::to_string(index + 1);
  route.priority = index;
  if (index >= 0 && static_cast<size_t>(index) < vehicle_speeds_.size())
  {
    route.nominal_speed = vehicle_speeds_[static_cast<size_t>(index)];
  }

  if (index == 0)
  {
    // 右转车：从西侧进入，右转后汇入南向外侧车道。
    route.reference_points = makeRightTurnToSouthPath();
    return route;
  }

  if (index == 1)
  {
    // 左转车起点略靠近交叉口，使其与右转车接近同时汇入出口车道。
    const double radius = lane_width_;
    const double start_x = std::max(2.0 * radius, lane_length_ - 8.0);
    appendWithoutDuplicate(route.reference_points,
                           sampleStraightRoute(Pose2d{start_x, lane_width_, M_PI},
                                               Pose2d{0.0, lane_width_, M_PI}));
    appendWithoutDuplicate(route.reference_points,
                           sampleCircularArc(0.0, 0.0, radius, M_PI_2, M_PI));
    appendWithoutDuplicate(route.reference_points,
                           sampleStraightRoute(Pose2d{-lane_width_, 0.0, -M_PI_2},
                                               Pose2d{-lane_width_, -lane_length_, -M_PI_2}));
    return route;
  }

  return makeFourWayStraightRoute(index);
}

std::vector<Pose2d> RoadNetwork::makeUnprotectedLeftTurnStraightPath() const
{
  if (left_turn_straight_relation_ == "same_after_turn")
  {
    // 与左转车转弯完成后的方向相同：北向南直行，进入同一条南向外侧车道。
    return sampleStraightRoute(Pose2d{-lane_width_, lane_length_, -M_PI_2},
                               Pose2d{-lane_width_, -lane_length_, -M_PI_2});
  }

  if (left_turn_straight_relation_ == "opposite_after_turn")
  {
    // 与左转车转弯完成后的方向相反：南向北直行，位于相邻对向车道。
    return sampleStraightRoute(Pose2d{0.0, -lane_length_, M_PI_2},
                               Pose2d{0.0, lane_length_, M_PI_2});
  }

  // 默认：与左转车原始进入方向对向而行，形成典型无保护左转冲突。
  return sampleStraightRoute(Pose2d{-lane_length_, 0.0, 0.0},
                             Pose2d{lane_length_, 0.0, 0.0});
}

std::vector<Pose2d> RoadNetwork::makeUnprotectedLeftTurnPath() const
{
  std::vector<Pose2d> points;
  // 左转车：从东侧进入，左转后驶入南向外侧车道 x=-lane_width。
  const double radius = lane_width_;
  appendWithoutDuplicate(points,
                         sampleStraightRoute(Pose2d{lane_length_, lane_width_, M_PI},
                                             Pose2d{0.0, lane_width_, M_PI}));
  appendWithoutDuplicate(points,
                         sampleCircularArc(0.0, 0.0, radius, M_PI_2, M_PI));
  appendWithoutDuplicate(points,
                         sampleStraightRoute(Pose2d{-lane_width_, 0.0, -M_PI_2},
                                             Pose2d{-lane_width_, -lane_length_, -M_PI_2}));
  return points;
}

std::vector<Pose2d> RoadNetwork::makeRightTurnToSouthPath() const
{
  std::vector<Pose2d> points;
  const double radius = lane_width_;
  // 右转车从西向东驶来，经西南象限右转，最终进入 x=-lane_width 的南向外侧车道。
  appendWithoutDuplicate(points,
                         sampleStraightRoute(Pose2d{-lane_length_, 0.0, 0.0},
                                             Pose2d{-2.0 * radius, 0.0, 0.0}));
  appendWithoutDuplicate(points,
                         sampleCircularArc(-2.0 * radius, -radius, radius, M_PI_2, 0.0));
  appendWithoutDuplicate(points,
                         sampleStraightRoute(Pose2d{-radius, -radius, -M_PI_2},
                                             Pose2d{-radius, -lane_length_, -M_PI_2}));
  return points;
}

std::vector<Pose2d> RoadNetwork::sampleStraightRoute(const Pose2d& start, const Pose2d& end) const
{
  const double length = distance2d(start.x, start.y, end.x, end.y);
  const int count = std::max(1, static_cast<int>(std::ceil(length / sample_spacing_)));
  const double yaw = std::atan2(end.y - start.y, end.x - start.x);

  // 沿直线按 sample_spacing 均匀采样，保留起点和终点。
  std::vector<Pose2d> points;
  points.reserve(static_cast<size_t>(count + 1));
  for (int i = 0; i <= count; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(count);
    Pose2d point;
    point.x = start.x + (end.x - start.x) * ratio;
    point.y = start.y + (end.y - start.y) * ratio;
    point.yaw = yaw;
    points.push_back(point);
  }
  return points;
}

std::vector<Pose2d> RoadNetwork::sampleCircularArc(const double center_x,
                                                   const double center_y,
                                                   const double radius,
                                                   const double start_angle,
                                                   const double end_angle) const
{
  const double arc_length = std::abs(end_angle - start_angle) * radius;
  const int count = std::max(1, static_cast<int>(std::ceil(arc_length / sample_spacing_)));
  const double direction = end_angle >= start_angle ? 1.0 : -1.0;

  std::vector<Pose2d> points;
  points.reserve(static_cast<size_t>(count + 1));
  for (int i = 0; i <= count; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(count);
    const double angle = start_angle + (end_angle - start_angle) * ratio;
    Pose2d point;
    point.x = center_x + radius * std::cos(angle);
    point.y = center_y + radius * std::sin(angle);
    point.yaw = normalizeAngle(angle + direction * M_PI_2);
    points.push_back(point);
  }
  return points;
}

}  // namespace structured_road_conflict_sim
