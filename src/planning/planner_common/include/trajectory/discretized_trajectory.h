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

#include <vector>
#include <algorithm>
#include <assert.h>
#include "math/vec2d.h"
#include "common/pnc_point.h"

namespace ugv {
namespace planning {
	
using namespace ugv::common;
using namespace ugv::planning;

class DiscretizedTrajectory : public std::vector<TrajectoryPoint> {
 public:
  DiscretizedTrajectory() = default;
  
  double header_time_ = 0.0;
  
  double header_time() const{ return header_time_; }
  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  //explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);

  explicit DiscretizedTrajectory(
      const std::vector<TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<TrajectoryPoint>& trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  virtual TrajectoryPoint StartPoint() const;

  virtual double GetTemporalLength() const;

  virtual double GetSpatialLength() const;

  virtual TrajectoryPoint Evaluate(const double relative_time) const;

  virtual size_t QueryLowerBoundPoint(const double relative_time,
                                      const double epsilon = 1.0e-5) const;

  virtual size_t QueryNearestPoint(const math::Vec2d& position) const;

  size_t QueryNearestPointWithBuffer(const math::Vec2d& position,
                                     const double buffer) const;

  virtual void AppendTrajectoryPoint(
      const TrajectoryPoint& trajectory_point);


  void PrependTrajectoryPoints(
      const std::vector<TrajectoryPoint>& trajectory_points) {
    if (!empty() && trajectory_points.size() > 1) {
      assert(trajectory_points.back().relative_time() <
             front().relative_time());
    }
    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  const TrajectoryPoint& TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const;

  virtual void Clear();

  
};

inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

inline void DiscretizedTrajectory::Clear() { clear(); }

}  // namespace planning
}  // namespace ugv
