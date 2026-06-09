
#ifndef CONTROL_COMMON_MSGS_H_
#define CONTROL_COMMON_MSGS_H_

#include <string>
#include <vector>

#include "common/comm_msgs.h"

namespace car {
namespace control {
namespace msgs {
// using namespace std;
using namespace ::car::common::msgs;

struct LocalizationEstimate {
  /* data */
  Header header;
  Pose pose;
  // Header header() { return header; }
};

struct Chassis {
  /* data */
  Header header;
  float speed_mps;
  GearPosition gear_location;
  DrivingMode driving_mode;
  MotionCmd motion_start_cmd;

  void Reset() {
    speed_mps = 0.0;
    gear_location = GEAR_NEUTRAL;
    driving_mode = COMPLETE_MANUAL;
    motion_start_cmd = MOTION_INVALID;
  }

  std::string GearString(const GearPosition &gear_location) {
    std::string str;
    switch (gear_location) {
    case GEAR_NEUTRAL:
      str = std::string("neutral");
      break;
    case GEAR_DRIVE:
      str = std::string("forward");
      break;
    case GEAR_REVERSE:
      str = std::string("reverse");
      break;
    case GEAR_PARKING:
      str = std::string("parking");
      break;
    default:
      str = std::string("invalid");
      break;
    }
    return str;
  }
  std::string DrivingModeString(const DrivingMode &driving_mode) {
    std::string str;
    switch (driving_mode) {
    case COMPLETE_MANUAL:
      str = std::string("manual");
      break;
    case COMPLETE_AUTO_DRIVE:
      str = std::string("auto");
      break;
    case EMERGENCY_MODE:
      str = std::string("emergency");
      break;
    // case AUTO_STEER_ONLY:
    //   str = std::string("only auto steer");
    //   break;
    // case AUTO_SPEED_ONLY:
    //   str = std::string("only auto speed");
    //   break;
    default:
      str = std::string("invalid");
      break;
    }
    return str;
  }
  std::string MotionCmdString(const MotionCmd &motion_start_cmd) {
    std::string str;
    switch (motion_start_cmd) {
    case MOTION_STOP:
      str = std::string("motion stop");
      break;
    case MOTION_START:
      str = std::string("motion start");
      break;
    default:
      str = std::string("invalid");
      break;
    }
    return str;
  }
};

struct ADCTrajectoryPoint {
  double x; // in meters.
  double y; // in meters.
  double z; // height in meters.

  double speed;                 // speed, in meters / second
  double acceleration_s;        // acceleration in s direction
  double curvature;             // curvature (k = 1/r), unit: (1/meters)
  double curvature_change_rate; // change of curvature in unit s (dk/ds)
  double relative_time;         // in seconds relative time (relative_time =
                                // time_of_this_state - timestamp_in_header)
  double theta;                 // relative to absolute coordinate system
  double accumulated_s; // calculated from the first point in this trajectory
};

struct TrajectoryPoint {
  PathPoint path_point;

  // linear velocity
  double v; // in [m/s]
  // linear acceleration
  double a;
  // relative time from beginning of the trajectory
  double relative_time;
};

struct Trajectory {
  /* data */
  Header header;
  // std::vector<ADCTrajectoryPoint> adc_trajectory_point;
  GearPosition gear; // Specify trajectory gear
  std::vector<TrajectoryPoint> trajectory_point;
  void Clear() {
    trajectory_point.clear();
    gear = GEAR_NONE;
  }
  std::string GearString(const GearPosition &gear_location) {
    std::string str;
    switch (gear_location) {
    case GEAR_NEUTRAL:
      str = std::string("neutral");
      break;
    case GEAR_DRIVE:
      str = std::string("forward");
      break;
    case GEAR_REVERSE:
      str = std::string("reverse");
      break;
    case GEAR_PARKING:
      str = std::string("parking");
      break;
    default:
      str = std::string("invalid");
      break;
    }
    return str;
  }
};

struct LatencyStats {
  /* data */
  double total_time_ms = 1;
  std::vector<double> controller_time_ms;
  bool total_time_exceeded;
};

struct SimpleLongitudinalDebug {
  /* data */
  double station_reference;
  double station_error;
  double station_error_limited;
  double speed_reference;
  double speed_error;
  double acceleration_reference;
  double acceleration_error;
  double speed_controller_input_limited;
  double preview_station_error;
  double preview_speed_reference;
  double preview_speed_error;
  double preview_acceleration_reference;
  double acceleration_cmd_closeloop;
  double acceleration_cmd;
  double acceleration_lookup;
  double speed_lookup;
  double calibration_value;
  double throttle_cmd;
  double brake_cmd;
  bool is_full_stop;
  bool is_throttle;
  double slope_offset_compensation;
  double current_station;
  double path_remain;
  PathPoint current_point;
  PathPoint matched_point;
  TrajectoryPoint reference_point;

  void Clear() {}
};

struct Debug {
  /* data */
  SimpleLongitudinalDebug simple_lon_debug;
  //   SimpleLateralDebug simple_lat_debug;
  //   InputDebug input_debug;
};

struct ControlCommand {
  /* data */
  Header header;
  // target throttle in percentage [0, 100]
  double throttle;

  // target brake in percentage [0, 100]
  double brake;

  // target non-directional steering rate, in percentage of full scale per
  // second [0, 100]
  double steering_rate;

  // target steering angle, in percentage of full scale [-100, 100]
  double steering_target;

  // parking brake engage. true: engaged
  bool parking_brake;

  // target speed, in m/s
  double speed;

  // target acceleration in m`s^-2
  double acceleration;

  // model reset
  bool reset_model;
  // engine on/off, true: engine on
  bool engine_on_off;
  // completion percentage of trajectory planned in last cycle
  double trajectory_fraction;
  DrivingMode driving_mode;
  GearPosition gear_location;

  Debug debug;
  VehicleSignal signal;
  LatencyStats latency_stats;

  bool is_in_safe_mode;
};
} // namespace msgs
} // namespace control
} // namespace car
#endif // CONTROL_COMMON_MSGS_H_
