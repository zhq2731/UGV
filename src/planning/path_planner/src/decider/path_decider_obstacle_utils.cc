/******************************************************************************
 * Copyright 2019 The ugv Authors. All Rights Reserved.
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

#include "decider/path_decider_obstacle_utils.h"

#include <limits>

#include "math/linear_interpolation.h"

namespace ugv {
namespace planning {

bool IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle) {
	
  PlanningConfig *planning_config = PlanningConfig::get_instance();
  // Obstacle should be non-virtual.
  if (obstacle.IsVirtual()) {
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle.IsIgnore()) {
    return false;
  }

  /* lianbin
  // Obstacle should not be moving obstacle.
  if (!obstacle.IsStatic() ||
      obstacle.speed() > planning_config->static_obstacle_speed_threshold) {
    return false;
  }
   */ 
  // TODO(jiacheng):
  // Some obstacles are not moving, but only because they are waiting for
  // red light (traffic rule) or because they are blocked by others (social).
  // These obstacles will almost certainly move in the near future and we
  // should not side-pass such obstacles.

  return true;
}

bool ComputeSLBoundaryIntersection(const SLBoundary& sl_boundary,
                                   const double s, double* ptr_l_min,
                                   double* ptr_l_max) {
  *ptr_l_min = std::numeric_limits<double>::max();
  *ptr_l_max = -std::numeric_limits<double>::max();

  // invalid polygon
  if (sl_boundary.boundary_point_size() < 3) {
    return false;
  }

  bool has_intersection = false;
  for (auto i = 0; i < sl_boundary.boundary_point_size(); ++i) {
    auto j = (i + 1) % sl_boundary.boundary_point_size();
    const auto& p0 = sl_boundary.boundary_point[i];
    const auto& p1 = sl_boundary.boundary_point[j];

    if (WithinBound<double>(std::fmin(p0.s(), p1.s()),
                                          std::fmax(p0.s(), p1.s()), s)) {
      has_intersection = true;
      auto l = common::math::lerp<double>(p0.l(), p0.s(), p1.l(), p1.s(), s);
      if (l < *ptr_l_min) {
        *ptr_l_min = l;
      }
      if (l > *ptr_l_max) {
        *ptr_l_max = l;
      }
    }
  }
  return has_intersection;
}

}  // namespace planning
}  // namespace ugv
