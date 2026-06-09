#ifndef CONTROL_CONTROLLER_H_
#define CONTROL_CONTROLLER_H_

#include <cmath>
#include <string>

#include "common/comm_msgs.h"
#include "common/status/status.h"
#include "control/common/mdl_msgs.h"
// #include "control/proto/control_conf.pb.h"
#include "control/common/config_parser.h"

namespace car {
namespace control {

class Controller {
public:
  Controller() = default;
  virtual ~Controller() = default;
  virtual car::common::Status Init(const ControlConfig *control_conf) = 0;

  virtual car::common::Status ComputeControlCommand(
      const car::control::msgs::LocalizationEstimate *localization,
      const car::control::msgs::Chassis *chassis,
      const car::control::msgs::Trajectory *trajectory,
      car::control::msgs::ControlCommand *cmd) = 0;

  virtual car::common::Status Reset() = 0;

  virtual std::string Name() const = 0;

  virtual void Stop() = 0;
};

} // namespace control
} // namespace car

#endif // CONTROL_CONTROLLER_CONTROLLER_H_