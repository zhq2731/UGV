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

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <utility>

#include "math/linear_interpolation.h"
#include "math/math_utils.h"

#include "math/box2d.h"
#include "math/vec2d.h"
#include "math/polygon2d.h"
#include "common/indexed_list.h"
#include "trajectory/reference_line.h"
#include "common/perception_obstacle.h"
#include "common/planning_config.h"
#include "amathutils_lib/amathutils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

namespace ugv {
namespace planning {
using namespace ugv::perception;
using namespace ugv::common::math;

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const bool is_static);
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const Trajectory& trajectory,
           const bool is_static);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

  /*
   * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
   */
  bool IsLaneBlocking() const { return is_lane_blocking_; }
  bool IsValidObstacle(
      const perception::PerceptionObstacle& perception_obstacle);
  void CheckLaneBlocking(const ReferenceLine& reference_line);
  bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

  double speed() const { return speed_; }

  int32_t PerceptionId() const { return perception_id_; }

  bool IsStatic() const { return is_static_; }
  bool IsVirtual() const { return is_virtual_; }

  TrajectoryPoint GetPointAtTime(const double time) const;

  common::math::Box2d GetBoundingBox(
      const TrajectoryPoint& point) const;

  const common::math::Box2d& PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }
  const common::math::Polygon2d& PerceptionPolygon() const {
    return perception_polygon_;
  }
  
  const Trajectory& getTrajectory() const { return trajectory_; }
  
  void AddIgnoreDecision(){is_ignore_ = true;}


  bool HasTrajectory() const {
      return !(trajectory_.trajectory_point.empty());
  }

  const perception::PerceptionObstacle& Perception() const {
    return perception_obstacle_;
  }

  static bool IsValidPerceptionObstacle(
      const perception::PerceptionObstacle& obstacle);

  static bool IsValidTrajectoryPoint(const TrajectoryPoint& point);


  const SLBoundary& PerceptionSLBoundary() const;
  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {
	sl_boundary_ = sl_boundary;
  }

  bool IsIgnore() const;

  /*
  TrajectoryPoint* AddTrajectoryPoint() {
    return trajectory_.add_trajectory_point();
  }

  // * @brief This is a helper function that can create obstacles from prediction
 //  * data.  The original prediction may have multiple trajectories for each
  // * obstacle. But this function will create one obstacle for each trajectory.
  // * @param predictions The prediction results
  // * @return obstacles The output obstacles saved in a list of unique_ptr.
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles& predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string& id, const common::math::Box2d& obstacle_box);


  inline bool IsCautionLevelObstacle() const {
    return is_caution_level_obstacle_;
  }


  // const Obstacle* obstacle() const;
   // return the merged lateral decision
  //  Lateral decision is one of {Nudge, Ignore}
  const ObjectDecisionType& LateralDecision() const;

   //@brief return the merged longitudinal decision
  // Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
  const ObjectDecisionType& LongitudinalDecision() const;

  std::string DebugString() const;


  const STBoundary& reference_line_st_boundary() const;

  const STBoundary& path_st_boundary() const;

  const std::vector<std::string>& decider_tags() const;

  const std::vector<ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision);

  void AddLateralDecision(const std::string& decider_tag,
                          const ObjectDecisionType& decision);

  void AddIgnoreDecision(){is_ignore_ = true;};
  bool HasLateralDecision() const;

  void set_path_st_boundary(const STBoundary& boundary);

  bool is_path_st_boundary_initialized() {
    return path_st_boundary_initialized_;
  }

  void SetStBoundaryType(const STBoundary::BoundaryType type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const STBoundary& boundary);

  void SetReferenceLineStBoundaryType(const STBoundary::BoundaryType type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;


    //@brief Calculate stop distance with the obstacle using the ADC's minimum
   // turning radius
   
  double MinRadiusStopDistance(const common::VehicleParam& vehicle_param) const;


  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                    const double adc_start_s);

  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

  static bool IsLongitudinalDecision(const ObjectDecisionType& decision);

  static bool IsLateralDecision(const ObjectDecisionType& decision);

  void SetLaneChangeBlocking(const bool is_distance_clear);

 private:
  FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);
  static ObjectDecisionType MergeLongitudinalDecision(
      const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);
  FRIEND_TEST(MergeLateralDecision, AllDecisions);
  static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType& lhs,
                                                 const ObjectDecisionType& rhs);

  bool BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                 const double adc_start_s,
                                 STBoundary* const st_boundary);
   */

 private:
  std::string id_;
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  bool is_ignore_ = false;
  double speed_ = 0.0;

  bool path_st_boundary_initialized_ = false;

  Trajectory trajectory_;
  perception::PerceptionObstacle perception_obstacle_;
  common::math::Box2d perception_bounding_box_;
  common::math::Polygon2d perception_polygon_;
  SLBoundary sl_boundary_;
  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;
  /*
  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;

  STBoundary reference_line_st_boundary_;
  STBoundary path_st_boundary_;

  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;

  struct ObjectTagCaseHash {
    size_t operator()(
        const planning::ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<size_t>(tag);
    }
  };

  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;

	*/  
};

typedef IndexedList<std::string, Obstacle> IndexedObstacles;
typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

}  // namespace planning
}  // namespace ugv
