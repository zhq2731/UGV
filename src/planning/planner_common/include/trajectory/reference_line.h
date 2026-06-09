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
 * @file reference_line.h
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "common/pnc_point.h"
#include "trajectory/reference_point.h"
#include "math/path_matcher.h"
#include "amathutils_lib/geometry.hpp"
#include "math/linear_interpolation.h"
#include "math/cartesian_frenet_conversion.h"
#include "math/vec2d.h"
#include "math/box2d.h"
#include "math/polygon2d.h"
#include "math/math_utils.h"
#include "math/math_utils.h"

#include "common/planning_config.h"

namespace ugv {
namespace planning {
	
using namespace ugv::planning;
using namespace ugv::common::math;

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;

  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  
  const std::vector<ReferencePoint>& reference_points() const;

  std::pair<std::array<double, 3>, std::array<double, 3>> ToFrenetFrame(
      const TrajectoryPoint& traj_point) const;

 std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                               double end_s) const;

  size_t GetNearestReferenceIndex(const double s) const;

  ReferencePoint GetNearestReferencePoint(const Vec2d& xy) const;

  size_t GetNearestReferenceIndex(const Vec2d& xy) const ;

  ReferencePoint GetNearestReferencePoint(const double s) const;


  std::array<double,3> XYToS_SD_L(std::array<double,4> trajectoryPoint) const;


  /*
  bool GetApproximateSLBoundary(const math::Box2d& box,
                                const double start_s, const double end_s,
                                SLBoundary* const sl_boundary) const;
  */
  bool GetSLBoundary(const Box2d& box,
                     SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const Polygon2d &polygon,
									SLBoundary* const sl_boundary) const ;

  bool XYToSL(const Vec2d& xy_point,
              SLPoint* const sl_point) const;
  
  bool XYToS_SD_L(const Vec2d& xy_point,
              SLPoint* const sl_point,double &s_d) const;
  
  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, SLPoint* const sl_point) const {
    return XYToSL(Vec2d(xy.x(), xy.y()), sl_point);
  }
  
  
  bool SLToXY(const SLPoint& sl_point,
              common::math::Vec2d* const xy_point) const;

  bool GetLaneWidth(const double s, double* const lane_left_width,
                    double* const lane_right_width) const;

  double GetDrivingWidth(const SLBoundary& sl_boundary) const;

  /**
   * @brief: check if a box/point is on lane along reference line
   */
  bool IsOnLane(const SLPoint& sl_point) const;
  bool IsOnLane(const Vec2d& vec2d_point) const;
  template <class XYPoint>
  bool IsOnLane(const XYPoint& xy) const {
    return IsOnLane(Vec2d(xy.x(), xy.y()));
  }
  
  bool IsOnLane(const SLBoundary& sl_boundary) const;

  void AddSpeedLimit(double start_s, double end_s, double speed_limit);

  uint32_t GetPriority() const { return priority_; }

  void SetPriority(uint32_t priority) { priority_ = priority; }

  std::vector<PathPoint> getDiscretizedPathPoint() const {return discrete_path_points;}
  double Length() const { return length_; }

 std::vector<PathPoint> ToDiscretizedReferenceLine (
	 const std::vector<ReferencePoint>& ref_points)  const ;



 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
   
  std::vector<PathPoint> discrete_path_points;
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  //hdmap::Path map_path_;
  uint32_t priority_ = 0;
  double length_;
  std::vector <double> accumulated_s_;

  PlanningConfig *planning_config;
};

}  // namespace planning
}  // namespace ugv
