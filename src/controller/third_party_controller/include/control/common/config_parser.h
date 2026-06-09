
#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <string>
#include <vector>

enum ControllerType {
  DEFAULT_CONTROLLER,
  HOWO_CONTROLLER,
  TANK_CONTROLLER,
  SEMI_CONTROLLER
};

struct PidConf {
  bool integrator_enable_;
  double integrator_saturation_level_;
  double kp_;
  double ki_;
  double kd_;

public:
  bool integrator_enable() const { return integrator_enable_; }
  double integrator_saturation_level() const {
    return integrator_saturation_level_;
  }
  double kp() const { return kp_; }
  double ki() const { return ki_; }
  double kd() const { return kd_; }
};

struct FilterConf {
  int cutoff_freq_;

public:
  int cutoff_freq() const { return cutoff_freq_; }
};

struct Calibration {
  double speed_;
  double command_;

public:
  double speed() const { return speed_; }
  double command() const { return command_; }
};

struct Deceleration{
  double deceleration_;
  double command_;

public:
  double deceleration() const { return deceleration_; }
  double command() const { return command_; }
};


struct LongitudinalControllerConfig {
  double ts_;
  double preview_window_;
  double throttle_deadzone_;
  double brake_deadzone_;
  double calibration_throttle_;
  double calibration_speed_to_stop_;
  double speed_controller_input_limit_;
  double standstill_brake_;
  double overspeed_light_brake_;
  double off_throttle_acc_;
  double point_size_to_end_;
  double path_to_stop_;
  bool is_simulation_;
  bool is_calibration_;
  double overspeed_acceleration_;
  double max_speed_error_for_brake_;
  double max_reverse_speed_error_for_brake_;
  double deceleration_deadzone_;
  double switch_speed_;
  bool enable_escape_mode_;
  double escape_determine_times_;
  double escape_mean_speed_threhold_;
  double escape_stdev_threhold_;
  double escape_compensate_;
  PidConf low_speed_pid_conf_;
  PidConf high_speed_pid_conf_;
  PidConf back_pid_conf_;
  PidConf acc_pid_conf_;
  FilterConf pitch_angle_filter_conf_;
  std::vector<Calibration> calibration_table_;
  std::vector<Deceleration> deceleration_table_;

public:
  double ts() const { return ts_; }
  double preview_window() const { return preview_window_; }
  double throttle_deadzone() const { return throttle_deadzone_; }
  double brake_deadzone() const { return brake_deadzone_; }
  double calibration_throttle() const{return calibration_throttle_;}
  double calibration_speed_to_stop() const{return calibration_speed_to_stop_;}
  double speed_controller_input_limit() const {
    return speed_controller_input_limit_;
  }
  double standstill_brake() const { return standstill_brake_; }
  double point_size_to_end() const{return point_size_to_end_;}
  double path_to_stop() const{return path_to_stop_;}
  bool is_simulation() const{return is_simulation_;}
  bool is_calibration() const{return is_calibration_;}
  double overspeed_light_brake() const { return overspeed_light_brake_; }
  double off_throttle_acc() const { return off_throttle_acc_; }
  double overspeed_acceleration() const { return overspeed_acceleration_; }

  double max_speed_error_for_brake() const {
    return max_speed_error_for_brake_;
  }
  double max_reverse_speed_error_for_brake() const {
    return max_reverse_speed_error_for_brake_;
  }
  double deceleration_deadzone() const { return deceleration_deadzone_; }
  double switch_speed() const { return switch_speed_; }
  bool enable_escape_mode() const { return enable_escape_mode_; }
  double escape_determine_times() const { return escape_determine_times_; }
  double escape_mean_speed_threhold() const {
    return escape_mean_speed_threhold_;
  }
  double escape_stdev_threhold() const { return escape_stdev_threhold_; }
  double escape_compensate() const { return escape_compensate_; }

  PidConf low_speed_pid_conf() const { return low_speed_pid_conf_; }
  PidConf back_pid_conf() const { return back_pid_conf_; }
  PidConf acc_pid_conf() const { return acc_pid_conf_; }
  PidConf high_speed_pid_conf() const { return high_speed_pid_conf_; }
  FilterConf pitch_angle_filter_conf() const {
    return pitch_angle_filter_conf_;
  }
  std::vector<Calibration> calibration_table() const {
    return calibration_table_;
  }
	std::vector<Deceleration> deceleration_table() const {
	  return deceleration_table_;
	}

};
struct VehicleParams {
  double weight_;
  double wheel_rolling_radius_;



public:
  double weight() const { return weight_; }
  double wheel_rolling_radius() const { return wheel_rolling_radius_; }
};

struct ControlConfig 
{
  std::string name_;
  std::vector<std::string> active_controllers_;
  bool enable_csv_debug_;
  bool enable_info_terminal_;
  bool enable_slope_offset_;
  bool is_deceleration_to_brake_;
  double slope_offset_coefficient_;
  VehicleParams vehicle_params_;
  LongitudinalControllerConfig lon_controller_conf_;

 public:
  std::string name() const { return name_; }
  std::vector<std::string> active_controllers() const {
    return active_controllers_;
  }
  bool enable_csv_debug() const { return enable_csv_debug_; }
  bool enable_info_terminal() const { return enable_info_terminal_; }
  bool enable_slope_offset() const { return enable_slope_offset_; }
  bool is_deceleration_to_brake() const { return is_deceleration_to_brake_; }
  double slope_offset_coefficient() const { return slope_offset_coefficient_; }
  VehicleParams vehicle_params() const { return vehicle_params_; }
  LongitudinalControllerConfig lon_controller_conf() const {
    return lon_controller_conf_;
  }
};

bool ParserConfig(const std::string fpath, ControlConfig &config);
void ControlConfigDebug(const ControlConfig &config);
#endif
