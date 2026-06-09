#include "common/vehicle_state.h"
#include "vehicle_info_util/vehicle_info_util.hpp"

namespace ugv {
namespace common {

	
class VehicleModel {
 public:
  VehicleModel() = delete;

  static VehicleState Predict(const double predicted_time_horizon,const VehicleState& cur_vehicle_state);
};

}  // namespace common
}  // namespace ugv


