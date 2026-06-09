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

#include "vehicle_info_util/vehicle_info_util.hpp"

#include <string>

namespace vehicle_info_util
{
	
void VehicleInfoUtil::loadVehicleingParam(ros::NodeHandle &private_nh_)
{
	private_nh_.param<double>("wheel_base_m", wheel_base_m, 1.0);
	private_nh_.param<double>("front_overhang_m", front_overhang_m, 1.0);
	private_nh_.param<double>("rear_overhang_m",  rear_overhang_m, 1.0);
	private_nh_.param<double>("left_overhang_m",  left_overhang_m, 1.0);
	private_nh_.param<double>("right_overhang_m",  right_overhang_m, 1.0);
	private_nh_.param<double>("vehicle_width_m",  vehicle_width_m, 1.0);
	private_nh_.param<double>("vehicle_height_m", vehicle_height_m, 1.0);
	private_nh_.param<double>("min_turn_radius", min_turn_radius, 1.0);


	private_nh_.param<double>("w2s_cubic_coeff", w2s_cubic_coeff, 1.0);
	private_nh_.param<double>("w2s_quadratic_coeff",w2s_quadratic_coeff, 1.0);
	private_nh_.param<double>("w2s_primary_coeff",w2s_primary_coeff, 1.0);
	private_nh_.param<double>("w2s_constant_term_coeff", w2s_constant_term_coeff, 1.0);

	private_nh_.param<double>("s2w_cubic_coeff",s2w_cubic_coeff, 1.0);
	private_nh_.param<double>("s2w_quadratic_coeff", s2w_quadratic_coeff, 1.0);
	private_nh_.param<double>("s2w_primary_coeff", s2w_primary_coeff, 1.0);
	private_nh_.param<double>("s2w_constant_term_coeff", s2w_constant_term_coeff, 1.0);

	private_nh_.param<double>("steer_rate_lim_dps", steer_rate_lim_dps, 1.0);
	private_nh_.param<double>("max_steer_angle_rad", max_steer_angle_rad, 1.0);

	private_nh_.param<double>("long_expansion_distance_m", long_expansion_distance_m, 1.0);
	private_nh_.param<double>("lat_expansion_distance_m", lat_expansion_distance_m, 1.0);

	vehicle_length_m = front_overhang_m + rear_overhang_m + wheel_base_m;
}

}  // namespace vehicle_info_util
