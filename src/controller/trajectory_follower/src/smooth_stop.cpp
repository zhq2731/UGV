// Copyright 2021 Tier IV, Inc.
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

#include "trajectory_follower/smooth_stop.hpp"

#include <experimental/optional>  // NOLINT

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
void SmoothStop::init(const double pred_vel_in_target, const double pred_stop_dist)
{
  m_weak_acc_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  // when distance to stopline is near the car
  if (pred_stop_dist < std::numeric_limits<double>::epsilon()) {
    m_strong_acc = m_params.min_strong_acc;
    return;
  }

  m_strong_acc = -std::pow(pred_vel_in_target, 2) / (2 * pred_stop_dist);
  m_strong_acc = std::max(std::min(m_strong_acc, m_params.max_strong_acc), m_params.min_strong_acc);
}

void SmoothStop::setParams(
  double max_strong_acc, double min_strong_acc, double weak_acc, double weak_stop_acc,
  double strong_stop_acc, double min_fast_vel, double min_running_vel, double min_running_acc,
  double weak_stop_time, double weak_stop_dist, double strong_stop_dist)
{
  m_params.max_strong_acc = max_strong_acc;
  m_params.min_strong_acc = min_strong_acc;
  m_params.weak_acc = weak_acc;
  m_params.weak_stop_acc = weak_stop_acc;
  m_params.strong_stop_acc = strong_stop_acc;

  m_params.min_fast_vel = min_fast_vel;
  m_params.min_running_vel = min_running_vel;
  m_params.min_running_acc = min_running_acc;
  m_params.weak_stop_time = weak_stop_time;

  m_params.weak_stop_dist = weak_stop_dist;
  m_params.strong_stop_dist = strong_stop_dist;

  m_is_set_params = true;
}




}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
