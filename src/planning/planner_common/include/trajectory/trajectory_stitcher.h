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

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "common/vehicle_state.h"
#include "common/pnc_point.h"
#include "common/vehcile_model.h"
#include "common/planning_config.h"
#include "trajectory/discretized_trajectory.h"
#include "math/vec2d.h"


using namespace ugv::common;

namespace ugv {
namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  static std::vector<TrajectoryPoint> ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, const DiscretizedTrajectory* prev_trajectory,
    std::string* replan_reason);

  static std::vector<TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const double planning_cycle_time,
      const VehicleState& vehicle_state);

 private:
  static std::pair<double, double> ComputePositionProjection(
      const double x, const double y,
      const TrajectoryPoint& matched_trajectory_point);

  static TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      const double planning_cycle_time,
      const VehicleState& vehicle_state);
};

}  // namespace planning
}  // namespace ugv
