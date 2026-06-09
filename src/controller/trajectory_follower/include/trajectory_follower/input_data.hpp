// Copyright 2022 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAJECTORY_FOLLOWER__INPUT_DATA_HPP_
#define TRAJECTORY_FOLLOWER__INPUT_DATA_HPP_

#include "autoware_msgs/TrajectoryPointArray.h"
#include "autoware_msgs/SteeringReport.h"
#include "nav_msgs/Odometry.h"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
struct InputData
{
    autoware_msgs::TrajectoryPointArray::Ptr current_trajectory_ptr;
    nav_msgs::Odometry::Ptr current_odometry_ptr;
    autoware_msgs::SteeringReport::Ptr current_steering_ptr;
    //geometry_msgs::AccelWithCovarianceStamped::SharedPtr current_accel_ptr;
   double vel;
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__INPUT_DATA_HPP_
