#include "control/controller/controller_agent.h"
#include "common/time/time.h"
#include "control/controller/tank_controller.h"

#include <utility>

#include "common/util/log.h"

namespace car {
namespace control {

using ::car::common::Status;
using ::car::common::time::Clock;
namespace control_msgs = ::car::control::msgs;
namespace common_msgs = ::car::common::msgs;
namespace common_time = ::car::common::time;

void ControllerAgent::RegisterControllers(const ControlConfig *control_conf) 
{
  AINFO("Only support MPC controller or Lat + Lon controllers as of now");
  for (auto active_controller : control_conf->active_controllers()) 
  {
    // if ("howo_controller" == active_controller) 
    // {
      // controller_factory_.Register(HOWO_CONTROLLER, []() -> Controller * {
      //   return new HowoController();
      // });
    // }

    if ("tank_controller" == active_controller) 
    {
      controller_factory_.Register(TANK_CONTROLLER, []() -> Controller * {
        return new TankController();
      });
    }

    // if ("semi_controller" == active_controller) 
    // {
      // controller_factory_.Register(SEMI_CONTROLLER, []() -> Controller * {
      //   return new SemiController();
      // });
    // }
  }
}

Status ControllerAgent::InitializeConf(const ControlConfig *control_conf) 
{
  if (!control_conf) {
    AERROR("control_conf is null.");
    return Status(common_msgs::CONTROL_INIT_ERROR, "Failed to load config");
  }
  control_conf_ = control_conf;
  ControllerType controller_type = DEFAULT_CONTROLLER;
  for (auto active_controller : control_conf_->active_controllers()) 
  {
    if ("howo_controller" == active_controller) {
      controller_type = HOWO_CONTROLLER;
    }
    if ("tank_controller" == active_controller) {
      controller_type = TANK_CONTROLLER;
    }
    if ("semi_controller" == active_controller) {
      controller_type = SEMI_CONTROLLER;
    }
    auto controller = controller_factory_.CreateObject(
        static_cast<ControllerType>(controller_type));
    if (controller) {
      controller_list_.emplace_back(std::move(controller));
    } else {
      AERROR("Controller: %s is not supported.", active_controller.c_str());
      return Status(common_msgs::CONTROL_INIT_ERROR,
                    "Invalid controller type:" + active_controller);
    }
  }
  return Status::OK();
}

Status ControllerAgent::Init(const ControlConfig *control_conf) 
{
  RegisterControllers(control_conf);
  
  if (!InitializeConf(control_conf).ok()) {
    return Status(common_msgs::CONTROL_INIT_ERROR, "Failed to load config");
  }
  
  for (auto &controller : controller_list_) {
    if (controller == nullptr || !controller->Init(control_conf_).ok()) {
      if (controller != nullptr) {
        AERROR("Controller <%s> init failed!", controller->Name().c_str());
        return Status(common_msgs::CONTROL_INIT_ERROR,
                      "Failed to init Controller:" + controller->Name());
      } else {
        return Status(common_msgs::CONTROL_INIT_ERROR,
                      "Failed to init Controller");
      }
    }
    AINFO("Controller <%s> init done!", controller->Name().c_str());
  }
  return Status::OK();
}

Status ControllerAgent::ComputeControlCommand(
    const control_msgs::LocalizationEstimate *localization,
    const control_msgs::Chassis *chassis,
    const control_msgs::Trajectory *trajectory,
    control_msgs::ControlCommand *cmd) 
{
  for (auto &controller : controller_list_) {
    double start_timestamp = common_time::ToSecond(Clock::Now());
    controller->ComputeControlCommand(localization, chassis, trajectory, cmd);
    double end_timestamp = common_time::ToSecond(Clock::Now());
    cmd->latency_stats.controller_time_ms.emplace_back(
        (end_timestamp - start_timestamp) * 1000);
  }
  return Status::OK();
}

Status ControllerAgent::Reset() 
{
  for (auto &controller : controller_list_) {
    controller->Reset();
  }
  return Status::OK();
}

} // namespace control
} // namespace car