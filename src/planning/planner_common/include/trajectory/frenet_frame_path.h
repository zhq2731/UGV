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
 * @file frenet_frame_path.h
 **/

#pragma once

#include <utility>
#include <vector>
#include <assert.h>
#include "common/pnc_point.h"

namespace ugv {
namespace planning {

class FrenetFramePath : public std::vector<FrenetFramePoint> {
 public:
  FrenetFramePath() = default;
  explicit FrenetFramePath(std::vector<FrenetFramePoint> points);

  double Length() const;
  FrenetFramePoint EvaluateByS(const double s) const;

  /**
   * @brief Get the FrenetFramePoint that is within SLBoundary, or the one with
   * smallest l() in SLBoundary's s range [start_s(), end_s()]
   */
  FrenetFramePoint GetNearestPoint(const SLBoundary &sl) const;

 private:
  static bool LowerBoundComparator(const FrenetFramePoint &p,
                                   const double s) {
    return p.s() < s;
  }
  static bool UpperBoundComparator(const double s,const FrenetFramePoint &p) {
    return s < p.s();
  }
};

}  // namespace planning
}  // namespace ugv
