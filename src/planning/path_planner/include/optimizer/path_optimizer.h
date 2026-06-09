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
 * @file path_optimizer.h
 **/

#pragma once

#include <memory>

#include "common/pnc_point.h"
#include "trajectory/reference_line.h"
#include "common/frame.h"
#include "reference_line_info.h"

namespace ugv {
namespace planning {

class PathOptimizer  {
 public:
  explicit PathOptimizer() = default;
  virtual ~PathOptimizer() = default;
  bool Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) ;
 virtual bool Process( ReferenceLineInfo* reference_line_info_,
   const TrajectoryPoint& init_point, const bool path_reusable) = 0;

};

}  // namespace planning
}  // namespace ugv
