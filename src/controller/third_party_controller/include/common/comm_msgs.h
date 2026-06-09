
#ifndef COMMON_MSGS_H_
#define COMMON_MSGS_H_

#include <math.h>
#include <string>
#include <vector>

namespace car {
namespace common {
namespace msgs {
// using namespace std;

enum ErrorCode {
  OK = 0,
  CONTROL_ERROR,
  CONTROL_INIT_ERROR,
  CONTROL_COMPUTE_ERROR,
  LOCALIZATION_ERROR
};
struct StatusPb {
  ErrorCode error_code;
  std::string msg;
};
struct Header {
  /* data */
  // Message publishing time in seconds. It is recommended to obtain
  // timestamp_sec from ros::Time::now(), right before calling
  // SerializeToString() and publish().
  double timestamp_sec;
  // Module name.
  std::string module_name;
  // Sequence number for each message. Each module maintains its own counter for
  // sequence_num, always starting from 1 on boot.
  uint32_t sequence_num;
  void Reset() {
    timestamp_sec = 0.0;
    sequence_num = 0;
    module_name = "";
  }
};

struct PointENU {
  /* data */
  double x; // East from the origin, in meters.
  double y; // North from the origin, in meters.
  double z; // Up from the WGS-84 ellipsoid, in
};

struct Point3D {
  double x;
  double y;
  double z;
};
struct Quaternion {
  /* data */
  double qx;
  double qy;
  double qz;
  double qw;
};

enum MotionCmd {
  MOTION_STOP = 0,   // 停止
  MOTION_START,      // 开始
  MOTION_INVALID     // 无效
};

enum GearPosition {
  GEAR_NEUTRAL = 0,
  GEAR_DRIVE,
  GEAR_REVERSE,
  GEAR_PARKING,
  GEAR_LOW,
  GEAR_INVALID,
  GEAR_NONE
};

enum DrivingMode {
  COMPLETE_MANUAL = 0, // human drive
  COMPLETE_AUTO_DRIVE,
  INTERVENE,
  REMOTE_CONTROL,
  AUTO_STEER_ONLY, // only steer
  AUTO_SPEED_ONLY, // include throttle and brake

  // security mode when manual intervention happens, only response status
  EMERGENCY_MODE
};

struct PathPoint {
  // coordinates
  double x;
  double y;
  double z;

  // direction on the x-y plane
  double theta;
  // curvature on the x-y planning
  double kappa;
  // accumulated distance from beginning of the path
  double s;

  // derivative of kappa w.r.t s.
  double dkappa;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa;
  // The lane ID where the path point is on
  // std::string lane_id;
};

enum TurnSignal {
  TURN_NONE = 0,
  TURN_LEFT,
  TURN_RIGHT,
};

struct VehicleSignal {
  TurnSignal turn_signal;
  // lights enable command
  bool high_beam;
  bool low_beam;
  bool horn;
  bool emergency_light;
};

struct Pose {
  /* data */
  // Position of the vehicle reference point (VRP) in the map reference frame.
  // The VRP is the center of rear axle.
  PointENU position;

  // A quaternion that represents the rotation from the map coordinate
  // (East/North/Up) to the
  // vehicle coordinate (Right/Forward/Up).
  Quaternion orientation;

  // Linear velocity of the VRP in the map reference frame.
  // East/north/up in meters per second.
  Point3D linear_velocity;

  // Linear acceleration of the VRP in the map reference frame.
  // East/north/up in meters per second.
  Point3D linear_acceleration;

  // Angular velocity of the vehicle in the map reference frame.
  // Around east/north/up axes in radians per second.
  Point3D angular_velocity;

  // heading
  double heading;

  // Linear acceleration of the VRP in the vehicle reference frame.
  // Right/forward/up in meters per square second.
  Point3D linear_acceleration_vrf;

  // Angular velocity of the vehicle in the vehicle reference frame.
  // Around right/forward/up axes in radians per second.
  Point3D angular_velocity_vrf;
  Point3D euler_angles;
};

} // namespace msgs
} // namespace common
} // namespace car
#endif // COMMON_MSGS_H_
