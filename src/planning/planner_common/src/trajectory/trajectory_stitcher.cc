/******************************************************************************
 * Copyright 2017 The ugv Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include <algorithm>

#include "trajectory/trajectory_stitcher.h"

namespace ugv {
namespace planning {
	
using namespace ugv::common;
using namespace ugv::common::math;

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
    TrajectoryPoint point;
	point.mutable_path_point()->set_s(0.0);
	point.mutable_path_point()->set_x(vehicle_state.x);
	point.mutable_path_point()->set_y(vehicle_state.y);
	point.mutable_path_point()->set_theta(vehicle_state.heading);
	point.set_v(vehicle_state.v);
	point.set_relative_time(planning_cycle_time);
    return point;
}


std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  if (std::abs(vehicle_state.v < kEpsilon_v)/* &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a*/) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);
  }

  return std::vector<TrajectoryPoint>(1, reinit_point);
}


/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, const DiscretizedTrajectory* prev_trajectory,
    std::string* replan_reason) {
   
  PlanningConfig *planning_config = PlanningConfig::get_instance();
  /*
  if (!FLAGS_enable_trajectory_stitcher) {
    *replan_reason = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  */

  
  if (0 == prev_trajectory->size()) {
        *replan_reason = "replan for no previous trajectory.";
        return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  /*
  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    *replan_reason = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  */


  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    std::cout << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();
  //std::cout <<"veh_rel_time "<<veh_rel_time<<std::endl;
  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  //for (int i  = 0 ;i < prev_trajectory->size();i++){
  	   // std::cout <<prev_trajectory->TrajectoryPointAt(i).relative_time()<<std::endl;
       // std::cout <<"s "<<prev_trajectory->TrajectoryPointAt(i).path_point().s()<<std::endl;
  //}

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    std::cout << "current time smaller than the previous trajectory's first time"<<std::endl;
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
	  
  if (time_matched_index + 1 >= prev_trajectory_size) {
    std::cout << "current time beyond the previous trajectory's last time"<<std::endl;
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  //std::cout <<"time_matched_index "<<time_matched_index<<std::endl;
  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));
  /*
  if (!time_matched_point.has_path_point()) {
    *replan_reason = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  */
  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
      {vehicle_state.x, vehicle_state.y}, 1.0e-6);
  //std::cout <<"position_matched_index "<<position_matched_index<<std::endl;

  //计算车辆当前位置在上一次规划路径上的frenet坐标
  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x, vehicle_state.y,
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));
  //std::cout <<frenet_sd.first  <<"  "<<frenet_sd.second<<std::endl;
  //std::cout <<"time match s "<<time_matched_point.path_point().s() <<std::endl;
  
  if (replan_by_offset) {
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
    auto lat_diff = frenet_sd.second;

    //std::cout << "Control lateral diff: " << lat_diff
          // << ", longitudinal diff: " << lon_diff<<std::endl;

    if (std::fabs(lat_diff) > planning_config->replan_lateral_distance_threshold) {
	  const std::string msg =
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = " + to_string(lat_diff);
     // std::cout << msg<<std::endl;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }

    if (std::fabs(lon_diff) > planning_config->replan_longitudinal_distance_threshold) {
	  const std::string msg =  "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = " + to_string(lon_diff);
      //std::cout << msg<<std::endl;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
  } else {
    std::cout << "replan according to certain amount of lat and lon offset is "
              "disabled"<<std::endl;
  }

  //上一次规划时间到现在的时间差+规划周期
  double forward_rel_time = veh_rel_time + planning_cycle_time;

  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  //std::cout << "Position matched index:\t" << position_matched_index<<std::endl;
  //std::cout << "Time matched index:\t" << time_matched_index<<std::endl;

  auto matched_index = std::min(time_matched_index, position_matched_index);
  
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),
      prev_trajectory->begin() + forward_time_index + 1);


  //std::cout << "stitching_trajectory size: " << stitching_trajectory.size()<<std::endl;
  /*
  const double zero_s = stitching_trajectory.back().path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  */
  
  return stitching_trajectory;
}


std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace planning
}  // namespace ugv
