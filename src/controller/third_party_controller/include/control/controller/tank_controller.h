/**
 * @file
 * @brief Defines the TankController class.
 */

#ifndef CONTROL_PID_TANK_CONTROLLER_H_
#define CONTROL_PID_TANK_CONTROLLER_H_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <fstream> 
#include <sstream>
#include <ros/ros.h>

#include "control/controller/controller.h"

#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/math/math_utils.h"
#include "common/vehicle_state/vehicle_state.h"

#include "control/common/interpolation_1d.h"
#include "control/common/trajectory_analyzer.h"

#include "control/controller/pid.h"
/**
 * @namespace car::control
 * @brief car::control
 */
namespace car {
namespace control {

class TankController : public Controller 
{
public:
  TankController();

  virtual ~TankController();

  common::Status Init(const ControlConfig *control_conf) override;

  common::Status ComputeControlCommand(
      const car::control::msgs::LocalizationEstimate *localization,
      const car::control::msgs::Chassis *chassis,
      const car::control::msgs::Trajectory *trajectory,
      car::control::msgs::ControlCommand *cmd) override;

  common::Status Reset() override;

  void Stop() override;

  std::string Name() const override;

protected:
  void ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory,
                                 const double preview_time,
                                 car::control::msgs::SimpleLongitudinalDebug *debug,
                                 const bool sign);
  void SetpolationThrottle(const TrajectoryAnalyzer *trajectory,
                                 const double preview_time,
                                 car::control::msgs::SimpleLongitudinalDebug *debug);
  double InterpolationThrottle(double v,double a);
  void MakePolationThrottle(double throttle);            
  void SaveLonTable(double throttle);    
  void SaveData();  
  void DataRecord(car::control::msgs::SimpleLongitudinalDebug *debug);             
                                                         


private:
  void SetDigitalFilterPitchAngle(const LongitudinalControllerConfig &lon_controller_conf);
  void LoadControlCalibrationTable(const LongitudinalControllerConfig &lon_controller_conf);

  void SetDigitalFilter(double ts, double cutoff_freq,DigitalFilter *digital_filter);
  double EscapeCompensate(double current_speed, double target_speed);

  void GetPathRemain(const TrajectoryAnalyzer *trajectory_analyzer,car::control::msgs::SimpleLongitudinalDebug *debug);
  void StopVehicle(car::control::msgs::SimpleLongitudinalDebug *debug,double l,car::control::msgs::ControlCommand *cmd);
  bool ReadLonTableTxt(std::string fileName);

  void CloseLogFile();
  void OpenLogFile();

  const car::control::msgs::LocalizationEstimate *localization_ = nullptr;
  const car::control::msgs::Chassis *chassis_ = nullptr;
  const car::control::msgs::Trajectory *trajectory_message_ = nullptr;

  car::common::vehicle_state::VehicleState vehicle_state_;

  std::unique_ptr<Interpolation1D> control_interpolation_;
  std::shared_ptr<Interpolation1D> lon_interpolation_;
  std::shared_ptr<Interpolation1D> throttle_interpolation_;
  std::vector<std::shared_ptr<Interpolation1D>> lon_interpolation_list;

  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;
  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_1;

  std::string name_;
  bool controller_initialized_ = false;
  bool ready_stop = false;

  PIDController speed_pid_controller_;  //油门控制pid
  
  PIDController acc_pid_controller_;    //调控减速度

  DigitalFilter digital_filter_pitch_angle_;

  const ControlConfig *control_conf_ = nullptr;
  std::deque<double> history_speed_;

  FILE *speed_log_file_ = nullptr;
  bool is_csv_debug_open_ = false;

  std::vector<std::vector<double>> lon_record;
  std::ofstream f_lon_data_out;

  std::vector<std::vector<double>> lon_table;
  
  ros::Time start_time;
  bool cal_sign = true;
  bool save_sign=true;
  double v_1 = 0.0; 

  /*lon_table*/
  std::vector<std::vector<double>> lon_txt;
  std::vector<std::vector<double>> throttle_acc;

  std::vector<int> throttle_list = {40};    /*标定的油门在这后面列出*/

  // uint8_t motion_start_ = 2;
};
} // namespace control
} // namespace car
#endif // CONTROL_PID_TANK_CONTROLLER_H_
