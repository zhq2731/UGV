/**
 * @file pid_controller.h
 * @brief Defines the PIDController class.
 */

#ifndef CONTROL_PID_CONTROLLER_H_
#define CONTROL_PID_CONTROLLER_H_

#include "control/common/config_parser.h"

/**
 * @namespace car::control
 * @brief car::control
 */
namespace car {
namespace control {

/**
 * @class PIDController
 * @brief A proportional–integral–derivative controller for speed and steering
 */
class PIDController {
 public:
  /**
   * @brief constructor
   */
  PIDController() = default;

  /**
   * @brief initialize pid controller
   * @param pid_conf configuration for pid controller
   */
  void Init(const PidConf &pid_conf);

  /**
   * @brief set pid controller coefficients for the proportional,
   * integral, and derivative
   * @param pid_conf configuration for pid controller
   */
  void SetPID(const PidConf &pid_conf);

  /**
   * @brief reset variables for pid controller
   */
  void Reset();

  /**
   * @brief compute control value based on the error
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  double Control(const double error, const double dt);

  /**
   * @brief get saturation status
   * @return saturation status
   */
  int saturation_status() const;

  /**
   * @brief get status that if integrator is hold
   * @return if integrator is hold return true
   */
  bool integrator_hold() const;

 private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  double saturation_high_ = 0.0;
  double saturation_low_ = 0.0;
  bool first_hit_ = false;
  bool integrator_enabled_ = false;
  bool integrator_hold_ = false;
  int saturation_status_ = 0;
};

}  // namespace control
}  // namespace car

#endif  // CONTROL_COMMON_PID_CONTROLLER_H_
