/******************************************************************************
 * Copyright 2017 The car Authors. All Rights Reserved.
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

#include "common/vehicle_state/vehicle_state.h"
#include "common/math/quaternion.h"
// #include "common/util/log.h"
#include <cmath>
#include <iostream>

namespace car {
namespace common {
namespace vehicle_state {

namespace control_msgs = ::car::control::msgs;
namespace common_msgs = ::car::common::msgs;
namespace math = ::car::common::math;

bool FLAGS_enable_map_reference_unify = false;

VehicleState::VehicleState(
    const control_msgs::LocalizationEstimate &localization) {
  ConstructExceptLinearVelocity(&localization);
  linear_v_ = localization.pose.linear_velocity.y;
}

VehicleState::VehicleState(
    const control_msgs::LocalizationEstimate *localization,
    const control_msgs::Chassis *chassis) {
  ConstructExceptLinearVelocity(localization);
  if (chassis != nullptr) {
    linear_v_ = chassis->speed_mps;
  }
}

void VehicleState::ConstructExceptLinearVelocity(
    const control_msgs::LocalizationEstimate *localization) {
  if (localization == nullptr) {
    // AWARN("Invalid localization input.");
    return;
  }
  // localization_ptr_ = localization;
  x_ = localization->pose.position.x;
  y_ = localization->pose.position.y;
  z_ = localization->pose.position.z;

  heading_ = localization->pose.heading;
  pitch_ = localization->pose.euler_angles.x;

  if (FLAGS_enable_map_reference_unify) {
    angular_v_ = localization->pose.angular_velocity_vrf.z;
    linear_a_ = localization->pose.linear_acceleration_vrf.y;
  } else {
    angular_v_ = localization->pose.angular_velocity.z;
    linear_a_ = localization->pose.linear_acceleration.y;
  }
}

double VehicleState::x() const { return x_; }

double VehicleState::y() const { return y_; }

double VehicleState::z() const { return z_; }

double VehicleState::heading() const { return heading_; }

double VehicleState::pitch() const { return pitch_; }

double VehicleState::linear_velocity() const { return linear_v_; }

double VehicleState::angular_velocity() const { return angular_v_; }

double VehicleState::linear_acceleration() const { return linear_a_; }

void VehicleState::set_x(const double x) { x_ = x; }

void VehicleState::set_y(const double y) { y_ = y; }

void VehicleState::set_z(const double z) { z_ = z; }

void VehicleState::set_heading(const double heading) { heading_ = heading; }
void VehicleState::set_pitch(const double pitch) { pitch_ = pitch; }

void VehicleState::set_linear_velocity(const double linear_velocity) {
  linear_v_ = linear_velocity;
}

void VehicleState::set_angular_velocity(const double angular_velocity) {
  angular_v_ = angular_velocity;
}
} // namespace vehicle_state
} // namespace common
} // namespace car
