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

#ifndef VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_
#define VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_

namespace vehicle_info_util
{

/// Data class for vehicle info
struct VehicleInfo
{
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
};


}  // namespace vehicle_info_util

#endif  // VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_
