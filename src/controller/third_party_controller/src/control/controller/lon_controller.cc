#include "control/controller/lon_controller.h"
#include "common/util/log.h"

/* ROS记录日志*/
#include "type_traits"
#include <fstream>
#include <iomanip>
#include <sstream>

#include "common/comm_msgs.h"
#include "common/math/euler_angles_zxy.h"

// #include "control/controller/controller_agent.h"

using std::placeholders::_1;
namespace car
{
  namespace control
  {

    /*********** longititude control**************/
    const double kMsgDelayThreshold = 1.0;
    const double kEpsilon = 1.0e-6;
    constexpr double DEGREE_PER_RAD = (180 / M_PI);

    namespace common_math = car::common::math;
    namespace common_msgs = car::common::msgs;
    namespace control_msgs = car::control::msgs;
    namespace common_util = ::car::common::util;
    namespace control_driver = car::control::driver;

    /**********************************************/

    LonController::LonController(std::string yaml_conf_path,std::string &platform_)
        : mControlConfPath(yaml_conf_path),platform(platform_)
    {
      // std::cout << "xuyixuan"<<std::endl;
      if (!ParserConfig(mControlConfPath, mControlConfig))
      {
        AERROR("get conf file from yaml failed, path: %s.",
               mControlConfPath.c_str());
      }
      ControlConfigDebug(mControlConfig);

      if (!mControllerAgent.Init(&mControlConfig).ok())
      {
        AERROR("get conf file Init failed.");
      }
	  
      mDesiredVelocity = 0.0;
      // mChassis.motion_start_cmd = common_msgs::MOTION_INVALID;
      mLstTermMotionCmd = common_msgs::MOTION_INVALID;
      mChassis.Reset();
    }

    LonController::~LonController() {}

    // 定位接口数据转换
    void LonController::ConvLocationMsg(
        const localization_msgs::Localization &location_data)
    {

      mLocalizationEstimate.header.timestamp_sec = location_data.timestamp;

      // vehicle reference point (VRP)
      common_math::EulerAnglesZXY<double> euler_angle(
          location_data.location.pose.pose.orientation.w,
          location_data.location.pose.pose.orientation.x,
          location_data.location.pose.pose.orientation.y,
          location_data.location.pose.pose.orientation.z);

      // East/north/up
      mLocalizationEstimate.pose.position.x =
          location_data.location.pose.pose.position.x;
      mLocalizationEstimate.pose.position.y =
          location_data.location.pose.pose.position.y;
      mLocalizationEstimate.pose.position.z =
          location_data.location.pose.pose.position.z;

	  mLocalizationEstimate.pose.heading = euler_angle.yaw();

	  mLocalizationEstimate.pose.angular_velocity.z =
          location_data.location.twist.twist.angular.z;

		  /**定位未给出x/y方向的速度,只给出地面速度,此处这么处理是兼容howo底盘速度不准，采用定位速度当做车速*/
      mLocalizationEstimate.pose.linear_velocity.y =
          location_data.original_ins.ground_speed;
      mLocalizationEstimate.pose.linear_velocity.x =
          location_data.location.twist.twist.linear.x;
      mLocalizationEstimate.pose.linear_acceleration.y =
          location_data.original_ins.acc_y;
      mLocalizationEstimate.pose.linear_acceleration.x =
          location_data.original_ins.acc_x;

      // ROS_INFO("orig yaw: %lf, heading: %lf, normalize yaw: %lf.",
      //          location_data.original_ins.yaw,
      //          mLocalizationEstimate.pose.heading,
      //          common_math::NormalizeAngle(location_data.original_ins.yaw));

      if (0)
      {
        /**howo
         * 惯导原始数据(单位为度),yaw是与正北方向夹角,heading是车头与正东方向夹角*/
        mLocalizationEstimate.pose.euler_angles.z = common_math::NormalizeAngle(
            location_data.original_ins.yaw / DEGREE_PER_RAD); // 航向角 -PI~PI
        mLocalizationEstimate.pose.euler_angles.y = common_math::NormalizeAngle(
            location_data.original_ins.roll / DEGREE_PER_RAD); // 翻滚角 -PI~PI
        mLocalizationEstimate.pose.euler_angles.x = common_math::NormalizeAngle(
            location_data.original_ins.pitch / DEGREE_PER_RAD); // 俯仰角 -PI/2~PI/2
      }
      else
      { /**tank*/
        mLocalizationEstimate.pose.euler_angles.z = common_math::NormalizeAngle(
            location_data.original_ins.yaw); // 航向角 -PI~PI
        mLocalizationEstimate.pose.euler_angles.y = common_math::NormalizeAngle(
            location_data.original_ins.roll); // 翻滚角 -PI~PI
        mLocalizationEstimate.pose.euler_angles.x = common_math::NormalizeAngle(
            location_data.original_ins.pitch); // 俯仰角 -PI/2~PI/2
      }
    }
    void LonController::ConvChassisMsg(
        const driver_msgs::ChassisReport &chassis_data,
        const driver_msgs::MotionStartCmd &motion_start_cmd)
    {
      /**底盘接口数据转换*/
      mChassis.header.timestamp_sec = chassis_data.header.stamp.sec;
      mChassis.speed_mps = chassis_data.current_velocity;

      /**运动开始指令*/
      // switch (motion_start_cmd.motion_start) {
      // case control_driver::DRIVER_MOTION_STOP:
      //   mChassis.motion_start_cmd = common_msgs::MOTION_STOP;
      //   break;
      // case control_driver::DRIVER_MOTION_START:
      //   mChassis.motion_start_cmd = common_msgs::MOTION_START;
      //   break;
      // default:
      //   mChassis.motion_start_cmd = common_msgs::MOTION_INVALID;
      //   break;
      // }

      // if (control_driver::DRIVER_MOTION_STOP == motion_start_cmd.motion_start ||
      //     control_driver::DRIVER_REMOTE_STOP == chassis_data.remote_button_status)
      // {
      //   mChassis.motion_start_cmd = common_msgs::MOTION_STOP;
      // }
      // if (control_driver::DRIVER_MOTION_START == motion_start_cmd.motion_start ||
      //     control_driver::DRIVER_REMOTE_START == chassis_data.remote_button_status)
      // {
      //   mChassis.motion_start_cmd = common_msgs::MOTION_START;
      // }

      /*运动开始指令*/
      /*当终端状态有效,并且状态发生改变时,说明产生一个终端指令,将响应终端指令,同时更新终端上一个状态*/
      if (control_driver::DRIVER_MOTION_INVALID != motion_start_cmd.motion_start) {
        /**上一个状态与终端状态不一致*/
        if ((control_driver::DRIVER_MOTION_STOP == motion_start_cmd.motion_start &&
            common_msgs::MOTION_STOP != mLstTermMotionCmd) ||
            (control_driver::DRIVER_MOTION_START == motion_start_cmd.motion_start &&
            common_msgs::MOTION_START != mLstTermMotionCmd)) {
          mChassis.motion_start_cmd =
              (motion_start_cmd.motion_start == control_driver::DRIVER_MOTION_STOP)
                  ? common_msgs::MOTION_STOP
                  : common_msgs::MOTION_START; /** 内部响应终端指令*/
          mLstTermMotionCmd = mChassis.motion_start_cmd; /** 更新状态*/
        }
      }
      /*遥控发送指令逻辑为:遥控按一下就发一个指令,发完下次接着发无效*/
      /*当遥控状态有效时,说明下发了一个遥控指令,将优先响应遥控指令*/
      if (control_driver::DRIVER_REMOTE_INVALID !=
          chassis_data.remote_button_status) {
        mChassis.motion_start_cmd =
            (chassis_data.remote_button_status ==
            control_driver::DRIVER_REMOTE_STOP)
                ? common_msgs::MOTION_STOP
                : common_msgs::MOTION_START; /**内部响应遥控指令*/
      }

      if (mControlConfig.enable_info_terminal_) {
        AWARN("motion[0-stop, 1-start, 2-invalid] = %d, for [motion_start_cmd, "
              "remote_button_status] = [%d, %d]",
              mChassis.motion_start_cmd, motion_start_cmd.motion_start,
              chassis_data.remote_button_status);
      }

      /*档位*/
      switch (chassis_data.gear_location)
      {
      case control_driver::DRIVER_GEAR_NEUTRAL:
        mChassis.gear_location = common_msgs::GEAR_NEUTRAL;
        break;
      case control_driver::DRIVER_GEAR_FORWARD:
        mChassis.gear_location = common_msgs::GEAR_DRIVE;
        break;
      case control_driver::DRIVER_GEAR_PARKING:
        mChassis.gear_location = common_msgs::GEAR_PARKING;
        break;
      case control_driver::DRIVER_GEAR_REVERSE:
        mChassis.gear_location = common_msgs::GEAR_REVERSE;
        break;
      default:
        mChassis.gear_location = common_msgs::GEAR_INVALID;
        break;
      }
	  
      /*驾驶模式*/
      switch (chassis_data.driving_mode)
      {
      case control_driver::DRIVER_COMPLETE_MANUAL_MODE:
        mChassis.driving_mode = common_msgs::COMPLETE_MANUAL;
        break;
      case control_driver::DRIVER_COMPLETE_AUTO_DRIVE_MODE:
        mChassis.driving_mode = common_msgs::COMPLETE_AUTO_DRIVE;
        break;
      case control_driver::DRIVER_INTERVENE_MODE:
        mChassis.driving_mode = common_msgs::EMERGENCY_MODE;
        break;
      case control_driver::DRIVER_REMOTE_CONTROL_MODE:
        mChassis.driving_mode = common_msgs::REMOTE_CONTROL;
        break;
      default:
        mChassis.driving_mode = common_msgs::EMERGENCY_MODE;
        break;
      }
    }

    void LonController::ConvTrajectoryMsg(
        const planning_msgs::TrajectoryPointArray &trajectory_data)
    {
      auto trajectory_points = trajectory_data.points;

      mTrajectory.Clear();

      // 规划接口数据转换
      mTrajectory.header.timestamp_sec = trajectory_data.header.stamp.sec;
      mTrajectory.header.sequence_num = trajectory_data.header.seq;
      mTrajectory.gear = trajectory_data.is_forward_shift
                             ? common_msgs::GEAR_DRIVE
                             : common_msgs::GEAR_REVERSE;

      for_each(trajectory_points.begin(), trajectory_points.end(),
               [this](auto trajectory_point)
               {
                 car::control::msgs::TrajectoryPoint trajectoryPoint;
                 trajectoryPoint.path_point.x = trajectory_point.x;
                 trajectoryPoint.path_point.y = trajectory_point.y;
                 trajectoryPoint.path_point.z = trajectory_point.z;
                 trajectoryPoint.path_point.theta = trajectory_point.theta;
                 trajectoryPoint.path_point.kappa = trajectory_point.kappa;
                 trajectoryPoint.path_point.dkappa = trajectory_point.dkappa;
                 trajectoryPoint.relative_time = trajectory_point.relative_time;
                 trajectoryPoint.a = trajectory_point.a;
                 trajectoryPoint.path_point.s = trajectory_point.s;

                 if (common_msgs::GEAR_DRIVE == mTrajectory.gear)
                 {
                       trajectoryPoint.v = trajectory_point.v;
                 }
                 else
                 {
                       trajectoryPoint.v = std::abs(trajectory_point.v);
                 }
                 mTrajectory.trajectory_point.push_back(trajectoryPoint);
               });
    }
    /*************************************************************/

    bool LonController::SetLocalData(const InputData &input)
    {
      mLocalData.chassis_data = input.chassis_data;
      mLocalData.location_data = input.location_data;
      mLocalData.trajectory_data = input.trajectory_data;
      mLocalData.motion_start_cmd = input.motion_start_cmd;
      
      ConvTrajectoryMsg(mLocalData.trajectory_data);
      ConvChassisMsg(mLocalData.chassis_data, mLocalData.motion_start_cmd);
      ConvLocationMsg(mLocalData.location_data);

      // const auto now = ros::Time::now().toSec();
      // if (kMsgDelayThreshold < now - mTrajectory.header.timestamp_sec) {
      //   ROS_WARN("Waiting for trajectory data update.");
      //   return false;
      // }

      // if (kMsgDelayThreshold < now - mLocalizationEstimate.header.timestamp_sec)
      // {
      //   ROS_WARN("Waiting for current odometry data update.");
      //   return false;
      // }

      // if (kMsgDelayThreshold < now - mChassis.header.timestamp_sec) {
      //   ROS_WARN("Waiting for current chassis data update.");
      //   return false;
      // }

      std::string strDrivingMode =
          mChassis.DrivingModeString(mChassis.driving_mode);
      std::string strGear = mChassis.GearString(mChassis.gear_location);

      if (mControlConfig.enable_info_terminal_)
      {
        AINFO(
            "chassis[v, a, driving_mode, gear] = [%lf, %lf, %d-%d(%s), %d-%d(%s)]",
            mChassis.speed_mps, 0.0, mLocalData.chassis_data.driving_mode,
            mChassis.driving_mode, strDrivingMode.c_str(),
            mLocalData.chassis_data.gear_location, mChassis.gear_location,
            strGear.c_str());
        AINFO("location[x, y, heading, pitch, v, a] = "
              "[%lf, %lf, %lf, %lf, %lf, %lf]",
              mLocalizationEstimate.pose.position.x,
              mLocalizationEstimate.pose.position.y,
              mLocalizationEstimate.pose.heading,
              mLocalizationEstimate.pose.euler_angles.x,
              mLocalizationEstimate.pose.linear_velocity.y,
              mLocalizationEstimate.pose.linear_acceleration.y);
      }

      strGear = mTrajectory.GearString(mTrajectory.gear);
      if (mControlConfig.enable_info_terminal_)
      {
        AINFO("trajectory[size, gear] = [%d, %d(%s)]",
              (int)mTrajectory.trajectory_point.size(), mTrajectory.gear,
              strGear.c_str());
      }
      return true;
    }
	
	  bool LonController::platformCmd(car::control::msgs::ControlCommand &controller_cmd,driver_msgs::DriveCmd &cmd)
    {
      /*如果底盘使用减速度进行制动，则在此添加*/
		  // if (platform == std::string("zhito") || platform == std::string("x6000"))
      // std::cout << loncmd_conf_->is_deceleration_to_brake() << std::endl;

      // std::cout << mControlConfig.is_deceleration_to_brake() <<"sssss"<< std::endl;
      
      if (mControlConfig.is_deceleration_to_brake())
		  {
        cmd.throttle_pedal = controller_cmd.throttle;
        cmd.acc_target = controller_cmd.acceleration;
        //std::cout << "-----at last acc is "<< cmd.acc_target<< "-----------------" << std::endl;
        //std::cout << "-----at last throttle is "<< cmd.throttle_pedal<< "-----------------" << std::endl;
        cmd.brake_pedal  = 0.0;
			  return  true; 
		  }

		  // auto func = [](const Deceleration& deceleration_,const double computed_deceleration_) 
      // {
      //   return deceleration_.deceleration() < fabs(computed_deceleration_);
		  // };

		  // cmd.brake_pedal = 0;
		  // if (controller_cmd.acceleration < 0)
      // {
      //   LongitudinalControllerConfig lon_controller_conf_ =  mControlConfig.lon_controller_conf();
      //   auto it_lower = std::lower_bound(lon_controller_conf_.deceleration_table().begin(),lon_controller_conf_.deceleration_table().end(), controller_cmd.acceleration, func);
      //   cmd.brake_pedal = it_lower->command();
      //   cmd.brake_pedal = car::common::math::Clamp(cmd.brake_pedal, lon_controller_conf_.throttle_deadzone(), 100.0);
		  // }
      // cmd.acc_target = 0.0;
      // cmd.throttle_pedal = controller_cmd.throttle;

      /*刹车插值*/
      LongitudinalControllerConfig lon_controller_conf_ = mControlConfig.lon_controller_conf();
      const auto &deceleration_table = lon_controller_conf_.deceleration_table();
      int brake_table_size = deceleration_table.size();
      Interpolation1D::DataType brake_xy;

      for (const auto &deceleration : deceleration_table)
      {
        brake_xy.push_back(std::make_pair(deceleration.deceleration(), deceleration.command()));
      }
      
      brakepedal_interpolation.reset(new Interpolation1D);
      brakepedal_interpolation->Init(brake_xy);

      cmd.brake_pedal = 0.0;
      
      if (controller_cmd.acceleration <= 0)
      {
        cmd.brake_pedal = brakepedal_interpolation->Interpolate(controller_cmd.acceleration);
      }
      cmd.acc_target = 0.0;
      cmd.throttle_pedal = std::round(controller_cmd.throttle);

      return true;
    }
	
    bool LonController::Process(driver_msgs::GearCmd &gear_cmd,
                                driver_msgs::DriveCmd &lon_drive_cmd, driver_msgs::ParkingBrakeCmd &parking_brake_cmd)
    {
      // std::cout << "----control calc start----" << std::endl;
      control_msgs::ControlCommand control_command;
      //计算控制量！！！
      if (mControllerAgent
              .ComputeControlCommand(&mLocalizationEstimate, &mChassis,
                                     &mTrajectory, &control_command)
              .ok())
      {
        // std::cout << "ComputeControlCommand success." << std::endl;
      }
      else
      {
        AERROR("ComputeControlCommand failed.");
        return false;
      }

	  platformCmd(control_command,lon_drive_cmd);
	  
      mLocalData.lon_drive_cmd.throttle_pedal = lon_drive_cmd.throttle_pedal;
      mLocalData.lon_drive_cmd.brake_pedal = lon_drive_cmd.brake_pedal;
      mLocalData.lon_drive_cmd.acc_target = lon_drive_cmd.acc_target;
      switch (control_command.gear_location)
      {
	      case common_msgs::GEAR_NEUTRAL:
	        mLocalData.gear_cmd.gear_location = control_driver::DRIVER_GEAR_NEUTRAL;
	        mLocalData.parking_brake_cmd.parking_brake = 1;//驻车
	        break;
	      case common_msgs::GEAR_DRIVE:
	        mLocalData.gear_cmd.gear_location = control_driver::DRIVER_GEAR_FORWARD;
	        mLocalData.parking_brake_cmd.parking_brake = 0;
	        break;
	      case common_msgs::GEAR_REVERSE:
	        mLocalData.gear_cmd.gear_location = control_driver::DRIVER_GEAR_REVERSE;
	        mLocalData.parking_brake_cmd.parking_brake = 0;
	        break;
	      case common_msgs::GEAR_PARKING:
	        mLocalData.gear_cmd.gear_location = control_driver::DRIVER_GEAR_PARKING;
	        mLocalData.parking_brake_cmd.parking_brake = 1;
	        break;
	      default:
	        mLocalData.gear_cmd.gear_location = control_driver::DRIVER_GEAR_INVALID;
	        break;
      }
      //AINFO("cmd before pub is -------%d in lon_controller.cc", mLocalData.gear_cmd.gear_location);

      gear_cmd = mLocalData.gear_cmd;
      parking_brake_cmd  = mLocalData.parking_brake_cmd;
      mDesiredVelocity = control_command.debug.simple_lon_debug.preview_speed_reference;
      return true;
    }

    double LonController::GetDesiredVelocity() { return mDesiredVelocity; }

  } // namespace control
} // namespace car
