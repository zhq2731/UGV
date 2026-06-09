#pragma once

#include <string>
#include <vector>

#include <ros/node_handle.h>

#include "structured_road_conflict_sim/common.hpp"

namespace structured_road_conflict_sim
{

struct VehicleRoute
{
  // 单车的参考路线：priority 数值越小，默认优先级越高。
  std::string id;
  int priority = 0;
  double nominal_speed = 2.0;
  std::vector<Pose2d> reference_points;
};

class RoadNetwork
{
public:
  void loadFromRosParams(const ros::NodeHandle& private_nh);

  const std::string& frameId() const;
  const std::string& scenario() const;
  int defaultVehicleCount() const;
  double laneLength() const;
  double laneWidth() const;
  double sampleSpacing() const;
  double conflictRadius() const;
  double stopLineDistance() const;
  Pose2d conflictCenter() const;

  VehicleRoute makeVehicle1Route() const;
  VehicleRoute makeVehicle2Route() const;
  VehicleRoute makeVehicleRoute(int index) const;
  std::vector<Pose2d> sampleStraightRoute(const Pose2d& start, const Pose2d& end) const;

private:
  VehicleRoute makeFourWayStraightRoute(int index) const;
  VehicleRoute makeUnprotectedLeftTurnRoute(int index) const;
  VehicleRoute makeLeftRightTurnConflictRoute(int index) const;
  std::vector<Pose2d> makeUnprotectedLeftTurnStraightPath() const;
  std::vector<Pose2d> makeUnprotectedLeftTurnPath() const;
  std::vector<Pose2d> makeRightTurnToSouthPath() const;
  std::vector<Pose2d> sampleCircularArc(double center_x,
                                        double center_y,
                                        double radius,
                                        double start_angle,
                                        double end_angle) const;

  // 默认构造一个十字路口场景，参数可由 launch/yaml 覆盖。
  std::string frame_id_ = "map";
  std::string scenario_ = "four_way_straight";
  std::string left_turn_straight_relation_ = "opposing_approach";
  double lane_length_ = 26.0;
  double lane_width_ = 3.5;
  double sample_spacing_ = 0.25;
  double conflict_radius_ = 2.6;
  double stop_line_distance_ = 4.5;
  double vehicle_1_speed_ = 2.5;
  double vehicle_2_speed_ = 2.0;
  std::vector<double> vehicle_speeds_;
};

}  // namespace structured_road_conflict_sim
