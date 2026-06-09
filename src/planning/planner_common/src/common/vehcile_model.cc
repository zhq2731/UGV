
#include "common/vehcile_model.h"
namespace ugv {
namespace common {


VehicleState VehicleModel::Predict(const double predicted_time_horizon,const VehicleState& cur_vehicle_state)
{
	VehicleState prediced_state = cur_vehicle_state;
	
	vehicle_info_util::VehicleInfoUtil *vehicle_util = vehicle_info_util::VehicleInfoUtil::get_instance();
	double dt = predicted_time_horizon;
	double cur_v  = cur_vehicle_state.v;
	double cur_x  = cur_vehicle_state.x;
	double cur_y  = cur_vehicle_state.y;
	double cur_delta = cur_vehicle_state.steering;
	double cur_heading = cur_vehicle_state.heading;
	
	double middleTheta =  cur_heading + 0.5*cur_v/vehicle_util->wheel_base_m * std::tan(cur_delta)*dt;
	prediced_state.x = cur_x + cur_v * std::cos(middleTheta)*dt;
	prediced_state.y = cur_y + cur_v * std::sin(middleTheta)*dt;
	prediced_state.heading = cur_heading + cur_v/vehicle_util->wheel_base_m * std::tan(cur_delta)*dt;
	return prediced_state;
}



}
}

