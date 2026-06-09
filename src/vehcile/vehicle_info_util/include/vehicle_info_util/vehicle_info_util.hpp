// Copyright 2015-2021 Autoware Foundation
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

#ifndef VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_
#define VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_

#include "vehicle_info_util/vehicle_info.hpp"

#include <ros/ros.h>

namespace vehicle_info_util
{
/// This is a convenience class for saving you from declaring all parameters
/// manually and calculating derived parameters.
/// This class supposes that necessary parameters are set when the node is launched.
class VehicleInfoUtil
{
public:
	
  VehicleInfoUtil(const VehicleInfoUtil&)=delete;
  VehicleInfoUtil& operator=(const VehicleInfoUtil&)=delete;
  
  static VehicleInfoUtil* get_instance(){
	  static VehicleInfoUtil instance;
	  return &instance;
  }

  //wheel rad
  double wheelToSteer(double wheel)
  {
      double square = wheel*wheel;
      return   square*wheel* w2s_cubic_coeff+
	  	square*w2s_quadratic_coeff+wheel*w2s_primary_coeff+w2s_constant_term_coeff;
  }
  
  //wheel rad
  double steerToWheel(double steer) 
  {
	  double square = steer*steer;
	  return   square*steer* s2w_cubic_coeff+
		square*s2w_quadratic_coeff+
		steer*s2w_primary_coeff+s2w_constant_term_coeff;
  }
  void loadVehicleingParam(ros::NodeHandle &private_nh_);
  
  /// Buffer for base parameters
  double wheel_base_m;
  
  double front_overhang_m;
  double rear_overhang_m;
  double left_overhang_m;
  double right_overhang_m;
  
  double vehicle_length_m;
  double vehicle_width_m;
  double vehicle_height_m;
  
  double min_turn_radius;
  
  double w2s_cubic_coeff;
  double w2s_quadratic_coeff;
  double w2s_primary_coeff;
  double w2s_constant_term_coeff;
  
  double s2w_cubic_coeff;
  double s2w_quadratic_coeff;
  double s2w_primary_coeff;
  double s2w_constant_term_coeff;
  
  double steer_rate_lim_dps;
  double max_steer_angle_rad;
  
  double long_expansion_distance_m;
  double lat_expansion_distance_m;
  
   VehicleInfoUtil(){}
private:
};

}  // namespace vehicle_info_util

#endif  // VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_
