
#include "control/common/config_parser.h"
#include "common/util/log.h"
#include "yaml-cpp/yaml.h"
#include <fstream>

template <typename T> void operator>>(const YAML::Node &node, T &i) {
  i = node.as<T>();
}

void operator>>(const YAML::Node &node, PidConf &pid_conf) 
{
  node["integrator_enable"] >> pid_conf.integrator_enable_;
  node["integrator_saturation_level"] >> pid_conf.integrator_saturation_level_;
  node["kp"] >> pid_conf.kp_;
  node["ki"] >> pid_conf.ki_;
  node["kd"] >> pid_conf.kd_;
}

void operator>>(const YAML::Node &node, FilterConf &pitch_angle_filter_conf) 
{
  node["cutoff_freq"] >> pitch_angle_filter_conf.cutoff_freq_;
}

void operator>>(const YAML::Node &node, Calibration &calibration) {
  node["speed"] >> calibration.speed_;
  node["command"] >> calibration.command_;
}

void operator>>(const YAML::Node &node, Deceleration &deceleration) {
  node["deceleration"] >> deceleration.deceleration_;
  node["command"] >> deceleration.command_;
}


void operator>>(const YAML::Node &node,
                LongitudinalControllerConfig &lon_controller_conf) 
{
  node["ts"] >> lon_controller_conf.ts_;
  node["preview_window"] >> lon_controller_conf.preview_window_;
  node["throttle_deadzone"] >> lon_controller_conf.throttle_deadzone_;
  node["brake_deadzone"] >> lon_controller_conf.brake_deadzone_;
  node["calibration_throttle"]>>lon_controller_conf.calibration_throttle_;
  node["calibration_speed_to_stop"] >>lon_controller_conf.calibration_speed_to_stop_;
  node["speed_controller_input_limit"] >>lon_controller_conf.speed_controller_input_limit_;
  node["standstill_brake"] >> lon_controller_conf.standstill_brake_;
  node["off_throttle_acc"] >> lon_controller_conf.off_throttle_acc_;
  node["point_size_to_end"]>>lon_controller_conf.point_size_to_end_;
  node["path_to_stop"]>>lon_controller_conf.path_to_stop_;
  node["is_simulation"]>>lon_controller_conf.is_simulation_;
  node["is_calibration"]>>lon_controller_conf.is_calibration_;
  node["overspeed_light_brake"] >> lon_controller_conf.overspeed_light_brake_;
  node["max_speed_error_for_brake"] >>lon_controller_conf.max_speed_error_for_brake_;
  node["max_reverse_speed_error_for_brake"] >>lon_controller_conf.max_reverse_speed_error_for_brake_;
  node["deceleration_deadzone"] >> lon_controller_conf.deceleration_deadzone_;
  node["switch_speed"] >> lon_controller_conf.switch_speed_;
  node["enable_escape_mode"] >> lon_controller_conf.enable_escape_mode_;
  node["escape_determine_times"] >> lon_controller_conf.escape_determine_times_;
  node["escape_mean_speed_threhold"] >>lon_controller_conf.escape_mean_speed_threhold_;
  node["escape_stdev_threhold"] >> lon_controller_conf.escape_stdev_threhold_;
  node["escape_compensate"] >> lon_controller_conf.escape_compensate_;
  node["low_speed_pid_conf"] >> lon_controller_conf.low_speed_pid_conf_;
  node["back_pid_conf"] >> lon_controller_conf.back_pid_conf_;
  node["acc_pid_conf"] >> lon_controller_conf.acc_pid_conf_;            
  node["high_speed_pid_conf"] >> lon_controller_conf.high_speed_pid_conf_;
  node["pitch_angle_filter_conf"] >>lon_controller_conf.pitch_angle_filter_conf_;
  node["overspeed_acceleration"] >>lon_controller_conf.overspeed_acceleration_;
  
  const YAML::Node &calibrationTableNode = node["calibration_table"];
  for (const auto &calibrationNode : calibrationTableNode) {
    Calibration calibration;
    calibrationNode >> calibration;
    lon_controller_conf.calibration_table_.push_back(calibration);
  }

  const YAML::Node &decelerationTableNode = node["deceleration_table"];
  for (const auto &decelerationNode : decelerationTableNode) {
    Deceleration deceleration;
    decelerationNode >> deceleration;
    lon_controller_conf.deceleration_table_.push_back(deceleration);
  }
}

void operator>>(const YAML::Node &node, VehicleParams &vehicle_params) {
  node["weight"] >> vehicle_params.weight_;
  node["wheel_rolling_radius"] >> vehicle_params.wheel_rolling_radius_;
}

void operator>>(const YAML::Node &node, ControlConfig &control_config) 
{
  node["name"] >> control_config.name_;

  const YAML::Node &activeControllersNode = node["active_controllers"];
  for (const auto &controllerNode : activeControllersNode) 
  {
    std::string controller;
    controllerNode >> controller;
    control_config.active_controllers_.push_back(controller);
  }

  node["enable_csv_debug"] >> control_config.enable_csv_debug_;
  node["enable_info_terminal"] >> control_config.enable_info_terminal_;
  node["enable_slope_offset"] >> control_config.enable_slope_offset_;
  node["is_deceleration_to_brake"] >> control_config.is_deceleration_to_brake_;
  node["slope_offset_coefficient"] >> control_config.slope_offset_coefficient_;
  node["vehicle_params"] >> control_config.vehicle_params_;
  node["lon_controller_conf"] >> control_config.lon_controller_conf_;
}

bool ParserConfig(const std::string fpath, ControlConfig &config) {
      
  std::ifstream fin(fpath.c_str());
  if (!fin.is_open()) {
    AERROR("Failed to open file: %s", fpath.c_str());
    return false;
  }
  // The document loading process changed in yaml-cpp 0.5.

  YAML::Node doc = YAML::Load(fin);

  doc >> config;
  fin.close();
  return true;
}
void ControlConfigDebug(const ControlConfig &config) {

  AINFO("name: %s", config.name().c_str());

  AINFO("active_controller:");
  for (const auto &controller : config.active_controllers()) {
    AINFO(" - %s", controller.c_str());
  }

  AINFO("enable_csv_debug: %s", config.enable_csv_debug() ? "true" : "false");
  AINFO("enable_info_terminal: %s",
        config.enable_info_terminal() ? "true" : "false");
  AINFO("enable_slope_offset: %s",
        config.enable_slope_offset() ? "true" : "false");
  AINFO("slope_offset_coefficient: %lf", config.slope_offset_coefficient());

  auto vehicle_params = config.vehicle_params();
  AINFO("vehicle_params:");
  AINFO(" weight: %lf", vehicle_params.weight());
  AINFO(" wheel_rolling_radius: %lf", vehicle_params.wheel_rolling_radius());

  auto lon_controller_conf = config.lon_controller_conf();
  AINFO("lon_controller_conf:");

  AINFO(" ts: %lf", lon_controller_conf.ts());
  AINFO(" preview_window: %lf", lon_controller_conf.preview_window());
  AINFO(" throttle_deadzone: %lf", lon_controller_conf.throttle_deadzone());
  AINFO(" brake_deadzone: %lf", lon_controller_conf.brake_deadzone());
  AINFO(" speed_controller_input_limit: %lf",
        lon_controller_conf.speed_controller_input_limit());
  AINFO(" standstill_brake: %lf", lon_controller_conf.standstill_brake());
  AINFO(" overspeed_light_brake: %lf",
        lon_controller_conf.overspeed_light_brake());
  AINFO(" max_speed_error_for_brake: %lf",
        lon_controller_conf.max_speed_error_for_brake());
  AINFO(" max_reverse_speed_error_for_brake: %lf",
        lon_controller_conf.max_reverse_speed_error_for_brake());
  AINFO(" deceleration_deadzone: %lf",
        lon_controller_conf.deceleration_deadzone());
  AINFO(" switch_speed: %lf", lon_controller_conf.switch_speed());
  AINFO(" enable_escape_mode: %s",
        lon_controller_conf.enable_escape_mode() ? "true" : "false");
  AINFO(" escape_determine_times: %lf",
        lon_controller_conf.escape_determine_times());
  AINFO(" escape_mean_speed_threhold: %lf",
        lon_controller_conf.escape_mean_speed_threhold());
  AINFO(" escape_stdev_threhold: %lf",
        lon_controller_conf.escape_stdev_threhold());
  AINFO(" escape_compensate: %lf", lon_controller_conf.escape_compensate());

  auto low_speed_pid_conf = lon_controller_conf.low_speed_pid_conf();
  AINFO(" low_speed_pid_conf:");

  AINFO("  integrator_enable: %s",
        low_speed_pid_conf.integrator_enable() ? "true" : "false");
  AINFO("  integrator_saturation_level: %lf",
        low_speed_pid_conf.integrator_saturation_level());
  AINFO("  kp: %lf", low_speed_pid_conf.kp());
  AINFO("  ki: %lf", low_speed_pid_conf.ki());
  AINFO("  kd: %lf", low_speed_pid_conf.kd());

  auto high_speed_pid_conf = lon_controller_conf.high_speed_pid_conf();
  AINFO(" high_speed_pid_conf:");

  AINFO("  integrator_enable: %s",
        high_speed_pid_conf.integrator_enable() ? "true" : "false");
  AINFO("  integrator_saturation_level: %lf",
        high_speed_pid_conf.integrator_saturation_level());
  AINFO("  kp: %lf", high_speed_pid_conf.kp());
  AINFO("  ki: %lf", high_speed_pid_conf.ki());
  AINFO("  kd: %lf", high_speed_pid_conf.kd());
  auto back_pid_conf = lon_controller_conf.back_pid_conf();
  auto acc_pid_conf = lon_controller_conf.acc_pid_conf();




  auto pitch_angle_filter_conf = lon_controller_conf.pitch_angle_filter_conf();
  AINFO(" pitch_angle_filter_conf:");
  AINFO("  cutoff_freq: %d", pitch_angle_filter_conf.cutoff_freq());

  auto calibration_table = lon_controller_conf.calibration_table();
  AINFO(" calibration_table:");
  for (const auto &calibration : calibration_table) {
    AINFO("  - speed: %lf", calibration.speed());
    AINFO("    command: %lf", calibration.command());
  }
}
