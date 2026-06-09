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
 * @file reference_point.cc
 **/

#include "trajectory/reference_point.h"

#include "common/point_factory.h"

namespace ugv {
namespace planning {
using namespace ugv::planning;
using namespace ugv::common::util;


namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
}  // namespace

//ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,
                               //const double kappa, const double dkappa)
  //  : hdmap::MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa) {}


PathPoint ReferencePoint::ToPathPoint(double s) const {
  return PointFactory::ToPathPoint(x(), y(), 0.0, s, heading(),
                                                 kappa(), dkappa());
}

/*
std::string ReferencePoint::DebugString() const {
  return absl::StrCat("{x: ", x(), ", y: ", y(), ", theta: ", heading(),
                      ", kappa: ", kappa(), ", dkappa: ", dkappa(), "}");
}
*/

double ReferencePoint::DistanceSquareTo(const ReferencePoint &other) const {
    const double dx = x_ - other.x();
    const double dy = y_ - other.y();
    return dx * dx + dy * dy;
}


void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {
  //CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } 
	//else {
      //(*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    //}
  }
  points->resize(count);
}

}  // namespace planning
}  // namespace ugv
