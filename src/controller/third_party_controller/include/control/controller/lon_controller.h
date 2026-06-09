#ifndef LON_CONTROLLER_H
#define LON_CONTROLLER_H


#include <string>

#include "control/common/mdl_msgs.h"
#include "control/controller/controller_agent.h"
#include "control/common/driver_inc.h"
#include "control/common/interpolation_1d.h"
#include "control/common/config_parser.h"


namespace car {
namespace control {


struct InputData {
  driver_msgs::ChassisReport chassis_data;
  localization_msgs::Localization location_data;
  planning_msgs::TrajectoryPointArray trajectory_data;
  driver_msgs::MotionStartCmd motion_start_cmd;
};

class LonController {
public:
  explicit LonController(std::string yaml_conf_path,std::string &platform_);
  bool Process(driver_msgs::GearCmd &gear_cmd,
                                driver_msgs::DriveCmd &lon_drive_cmd, driver_msgs::ParkingBrakeCmd &parking_brake_cmd);
  bool SetLocalData(const InputData &input);
  // bool getDesirdVel(double &vel){}
  double GetDesiredVelocity();
  ~LonController();

private:
  void
  ConvTrajectoryMsg(const planning_msgs::TrajectoryPointArray &trajectory_data);
  void ConvChassisMsg(const driver_msgs::ChassisReport &chassis_data,
                      const driver_msgs::MotionStartCmd &motion_start_cmd);
  void ConvLocationMsg(const localization_msgs::Localization &location_data);
  bool platformCmd(car::control::msgs::ControlCommand &controller_cmd,driver_msgs::DriveCmd &cmd);

private:
  car::control::ControllerAgent mControllerAgent;

  ControlConfig mControlConfig;

  std::string mControlConfPath; // 配置文件路径

  car::control::msgs::LocalizationEstimate mLocalizationEstimate; /**内部接口*/
  car::control::msgs::Chassis mChassis;       /**内部接口*/
  car::control::msgs::Trajectory mTrajectory; /**内部接口*/

  car::control::driver::LocalData mLocalData; /**本地数据*/

  double mDesiredVelocity;                    /*期望速度(预瞄点)*/

  car::common::msgs::MotionCmd mLstTermMotionCmd; /** 上一时刻终端启动命令*/
  std::string platform;
  std::unique_ptr<Interpolation1D> brakepedal_interpolation;
  // const ControlConfig *loncmd_conf_ = nullptr;
};
} // namespace control
} // namespace car

#endif
