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

#include "optimizer/path_optimizer.h"

#include <memory>


namespace ugv {
namespace planning {


bool PathOptimizer::Execute(Frame* frame,
                              ReferenceLineInfo* const reference_line_info) {
 // Task::Execute(frame, reference_line_info);
  auto ret = Process( reference_line_info,frame->PlanningStartPoint(),false);
  if (ret != true) {
    reference_line_info->SetDrivable(false);
    std::cout << "Reference Line " <<  " is not drivable after "<<std::endl ;
  }
  return ret;
}



}  // namespace planning
}  // namespace ugv
