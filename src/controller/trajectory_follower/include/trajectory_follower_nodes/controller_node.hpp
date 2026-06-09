// Copyright 2021 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <ros/ros.h>
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower/lateral_controller_base.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "autoware_msgs/TrajectoryPointArray.h"
#include "autoware_msgs/TrajectoryPoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include  "std_msgs/Float32.h"
#include  "std_msgs/Int32.h"

#include "planning_msgs/TrajectoryPointArray.h"
#include "ins_msgs/Ins.h"
#include  "localization_msgs/Localization.h"

#include "driver_msgs/DriveCmd.h"
#include "driver_msgs/ChassisReport.h"
#include "driver_msgs/SteeringWheelCmd.h"
#include "driver_msgs/GearCmd.h"
#include "driver_msgs/MotionStartCmd.h"

#include "vehicle_info_util/vehicle_info_util.hpp"
#include "amathutils_lib/amathutils.hpp"
#include "amathutils_lib/geometry.hpp"

#include "control/controller/lon_controller.h"
#include "display/display.h"
#include "visualization_msgs/MarkerArray.h"
#include <heartbeat_msgs/Heartbeat.h>
#include "platoon_msgs/PlatoonLog.h"

#include <pwd.h>
#include <fstream> 
#include "common/common.h"
#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"
#include "platoon_msgs/PlatoonLog.h"
#include "platoon_common/platoon_common.h"

namespace autoware
{
namespace motion
{
namespace control
{
using trajectory_follower::LateralOutput;
namespace trajectory_follower_nodes
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// \classController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)

struct PlatformParam
{
	std::string vehicle_type;// "zhitong"  # 车辆类型  geometry_c  sanzhou tank500 zhito zw zhitong
	std::string ins_type;// "cgi610" # cgi610 cgi1010 ht33  570d shengda
	std::string id; // "vehicle_1"  # vehicle_1 vehicle_2...
	int num; // # 1 2 3 4...
};


struct ControlLog
{ 
	geometry_msgs::Pose pose;
    double yaw;
	geometry_msgs::Pose nearestPose;
	double current_v;
	double deired_v;
	double v_error;
	double lat_error;
	double yaw_error;
	double driving_s;
	double throttle;
	double brake_pedal;
	double deired_acc;
	geometry_msgs::Pose lastLogPose;
};


class TRAJECTORY_FOLLOWER_PUBLIC Controller
{
public:
  explicit Controller(ros::NodeHandle &nh);
  virtual ~Controller() {logfile.close();}
  void run();
  void sendHeart(unsigned char flag);

private:
	nav_msgs::Odometry current_pose_;
	::common::PlatformParam platform_param;
	double steer_compensation;

	planning_msgs::TrajectoryPointArray sim_trajectory;
	autoware_msgs::TrajectoryPointArray current_waypoints_;
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh;
	ros::Timer timer;
    int driving_mode;
	std::ofstream   logfile;
	ros::Publisher  pub2_;
	ros::Subscriber sub_pose_; 
	ros::Subscriber sub_platoon_log; 

	ros::Subscriber chassis_sub_, motion_start_sub_;
	
	ros::Subscriber steer_compensation_sub;
	ros::Publisher pub_diplay;
	ros::Publisher pub_heart;
	
	ros::Subscriber steering_sub_;

	trajectory_follower::InputData input_data_;
	double timeout_thr_sec_;
	double steer_compensation_degree;
	boost::optional<LateralOutput> lateral_output_{boost::none};

	std::shared_ptr<trajectory_follower::LateralControllerBase> lateral_controller_;

	/**************longititude control**************/
	std::shared_ptr<car::control::LonController> longitudinal_controller_;
	std_msgs::Header mHeader;
	ros::Publisher gearcmd_pub_; /**档位发布者*/
	ros::Publisher parking_brake_pub_; /**档位发布者*/

    
    ros::Subscriber platoonMission_sub_;
	ros::Subscriber platoonConfig_self_sub_;
	ros::Subscriber sub_ref_path_;
	ros::Subscriber sub_chassis_;
	ros::Subscriber sub_odometry_;
	ros::Subscriber sub_steering_;
	ros::Subscriber sub_accel_;
	ros::Publisher control_cmd_pub_;

	ros::Publisher latcmd_pub_;
	ros::Publisher loncmd_pub_;
	ros::Publisher latcontrol_debug;
	bool is_forward_shift;
	int  driving_mode_feedback = 0;//0：人工 1：自动
	bool debug;
	bool open_lat_controller;
	bool open_lon_controller;
	bool open_simulate;
	double simulate_velocity;
	double heading_compensation_degree;
	bool lat_use_current_velocity_only;
	bool enable_log;
	bool is_pose_set_;
	driver_msgs::SteeringWheelCmd lastLatCmd;
	driver_msgs::DriveCmd lastLonCmd;
	vehicle_info_util::VehicleInfoUtil  *vehcileInfo;
    car::control::InputData lon_input;
	platoon_msgs::PlatoonLog platoonLog;
	unsigned char  trajectoryType = PlatoonType::NONE;
	unsigned char  inputTrajectoryType = PlatoonType::NONE;
	ControlLog controlLog;
	bool platoonBuild = false;
	enum class LateralControllerMode {
	INVALID = 0,
	MPC = 1,
	PURE_PURSUIT = 2,
	};
	enum class LongitudinalControllerMode {
	INVALID = 0,
	PID = 1,
	};

	/**
	* @brief compute control command, and publish periodically
	*/
	void callbackTimerControl(const ros::TimerEvent &event);
	void onTrajectory(const planning_msgs::TrajectoryPointArray::Ptr);
	//void onChassis(const driver_msgs::msg::Chassis::SharedPtr msg);
	//void onOdometry(const gps_info_msgs::msg::GpsInfoData::SharedPtr msg);
	//void onSteering(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
	//void onAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
	bool isTimeOut();
	LateralControllerMode getLateralControllerMode(const std::string & algorithm_name) const;
	LongitudinalControllerMode getLongitudinalControllerMode(
	const std::string & algorithm_name) const;

	void chassisCallback(const driver_msgs::ChassisReport::ConstPtr &msg);
	void platoonLogCallback(const platoon_msgs::PlatoonLog::ConstPtr &msg);

    
	void steerCompensationCallback(const std_msgs::Float32::ConstPtr &msg);
	void motionStartCallback(const driver_msgs::MotionStartCmd::ConstPtr &msg);
	void callbackPose(const localization_msgs::Localization::ConstPtr &msg);
	double distance2D(geometry_msgs::Point &p1 ,geometry_msgs::Point &p2) ;
	double	closestPointVel() const ;
	double normalized_angle(double angle) ;
	void lonControl();
	void latControl();
	void diplayDesireVelocity();
	size_t QueryLowerBoundPoint(const double relative_time,
                                                   planning_msgs::TrajectoryPointArray &trajectory,const double epsilon = 1.0e-5) const ;
	void computeLonSim(driver_msgs::DriveCmd &lonCmd);
	void callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg) ;

};
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_
