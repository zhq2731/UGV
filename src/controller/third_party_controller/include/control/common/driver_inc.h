#ifndef CONTROL_COMMON_DRIVER_INC_H_
#define CONTROL_COMMON_DRIVER_INC_H_

#include "driver_msgs/ChassisReport.h"
#include "driver_msgs/DriveCmd.h"
#include "driver_msgs/GearCmd.h"
#include "driver_msgs/MotionStartCmd.h"
#include "localization_msgs/Localization.h"
#include "planning_msgs/TrajectoryPointArray.h"
#include "driver_msgs/ParkingBrakeCmd.h"

namespace car {
namespace control {
namespace driver {

/**************longititude control**************/
struct LocalData {
  driver_msgs::ChassisReport chassis_data;
  localization_msgs::Localization location_data;
  planning_msgs::TrajectoryPointArray trajectory_data;
  driver_msgs::DriveCmd lon_drive_cmd;
  driver_msgs::GearCmd gear_cmd;
  driver_msgs::MotionStartCmd motion_start_cmd;
  driver_msgs::ParkingBrakeCmd parking_brake_cmd;
};

// driver msgs消息中底盘的档位
enum DriverMsgsGear {
  DRIVER_GEAR_NEUTRAL = 0,
  DRIVER_GEAR_FORWARD,
  DRIVER_GEAR_PARKING,
  DRIVER_GEAR_REVERSE = 7,
  DRIVER_GEAR_INVALID
};
enum DriverMsgsMode {
  DRIVER_COMPLETE_MANUAL_MODE = 0, // 人工
  DRIVER_COMPLETE_AUTO_DRIVE_MODE, // 自动
  DRIVER_INTERVENE_MODE,           // 干预
  DRIVER_REMOTE_CONTROL_MODE,      // 遥控
  DRIVER_INVALID_MODE              // 无效
};
enum DriverMsgsMotion {
  DRIVER_MOTION_STOP = 0,          // 停止
  DRIVER_MOTION_START,             // 开始
  DRIVER_MOTION_INVALID            // 无效
};

enum DriverMsgsRemote {
  DRIVER_REMOTE_INVALID = 0,       // 无效
  DRIVER_REMOTE_START,             // 开始
  DRIVER_REMOTE_STOP               // 停止
};


/**********************************************/
} // namespace driver
} // namespace control
} // namespace car
#endif
