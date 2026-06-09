#ifndef CONTROL_CONTROLLER_AGENT_H_
#define CONTROL_CONTROLLER_AGENT_H_

#include "common/util/factory.h"
#include "control/controller/controller.h"
#include <cstdio>
#include <memory>
#include <vector>

namespace car {
namespace control {

class ControllerAgent {
public:
  car::common::Status Init(const ControlConfig *control_conf);
  car::common::Status ComputeControlCommand(
      const car::control::msgs::LocalizationEstimate *localization,
      const car::control::msgs::Chassis *chassis,
      const car::control::msgs::Trajectory *trajectory,
      car::control::msgs::ControlCommand *cmd);
  car::common::Status Reset();

private:
  void RegisterControllers(const ControlConfig *control_conf);

  car::common::Status InitializeConf(const ControlConfig *control_conf);
  const ControlConfig *control_conf_ = nullptr;
  car::common::util::Factory<ControllerType, Controller>
      controller_factory_;
  std::vector<std::unique_ptr<Controller>> controller_list_;
};
} // namespace control
} // namespace car

#endif