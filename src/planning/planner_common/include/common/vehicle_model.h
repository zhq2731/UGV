#include "common/vehicle_state.h"

namespace ugv {
namespace common {

using namespace ugv::planning;
	
class VehicleModel {
 public:
  VehicleModel() = delete;

  static VehicleState Predict(const double predicted_time_horizon,const VehicleState& cur_vehicle_state);
};

}  // namespace common
}  // namespace ugv


