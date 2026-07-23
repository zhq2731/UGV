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

#include "trajectory_follower_nodes/controller_node.hpp"
#include "trajectory_follower/mpc_lateral_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include <unistd.h>


namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{


// Keep the angle within [-pi, pi]
double normalized_angle(double angle) {
  if (angle > 180.0) {
      angle -= 2 * 180.0;
  } else if (angle < -180.0) {
      angle += 2 * 180.0;
  }
  return angle;
}


//
void Controller::callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg) {

	unsigned char   optType = msg->command_type;
	assert(optType >= PlatoonType::BUILD);
	assert(optType <= PlatoonType::RUNNING);
	
    if ((optType != PlatoonType::BUILD) && (!platoonBuild))
		return;

    if ((optType == PlatoonType::BUILD) && (platoonBuild))
		return;	

    trajectoryType = optType;
	if (optType == PlatoonType::BUILD){
		platoonBuild = true;
	}

	if (optType == PlatoonType::DISSOLVE){
		platoonBuild = false;
	}	
	
	return;
}



Controller::Controller(ros::NodeHandle &nh): nh_(nh),private_nh("~")
{

	steer_compensation = 0.0;
	private_nh.param<double>("steer_compensation_degree",  steer_compensation_degree, 0.0);

	private_nh.param<bool>("open_lon_controller",  open_lon_controller, true);
	private_nh.param<bool>("open_lat_controller",  open_lat_controller, true);
	private_nh.param<bool>("open_simulate",  open_simulate, false);
	private_nh.param<bool>("open_space_execution_mode", open_space_execution_mode, false);
	private_nh.param<double>("open_space_lookahead_distance", open_space_lookahead_distance, 0.5);
	private_nh.param<double>("open_space_speed_kp", open_space_speed_kp, 1.5);
	private_nh.param<double>("open_space_max_acceleration", open_space_max_acceleration, 0.5);
	private_nh.param<double>("open_space_max_deceleration", open_space_max_deceleration, 1.0);
	private_nh.param<double>("open_space_steering_prepare_tolerance",
		open_space_steering_prepare_tolerance, 0.03);
	private_nh.param<double>("open_space_stop_speed_tolerance",
		open_space_stop_speed_tolerance, 0.05);
	private_nh.param<double>("open_space_hold_deceleration",
		open_space_hold_deceleration, 1.0);
	private_nh.param<double>("open_space_hold_brake_pedal",
		open_space_hold_brake_pedal, 20.0);
	private_nh.param<double>("open_space_segment_end_remaining_distance",
		open_space_segment_end_remaining_distance, 0.30);
	private_nh.param<double>("open_space_segment_end_stable_duration",
		open_space_segment_end_stable_duration, 1.0);
	private_nh.param<int>("open_space_steering_prepare_stable_cycles",
		open_space_steering_prepare_stable_cycles, 3);
	private_nh.param<double>("heading_compensation_degree",  heading_compensation_degree, 0.0);
	

	private_nh.param<bool>("lat_use_current_velocity_only",  lat_use_current_velocity_only,false);
	private_nh.param<bool>("enable_control_log",  enable_log,true);
	
	vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();
	vehcileInfo->loadVehicleingParam(private_nh);

	lon_input.motion_start_cmd.motion_start = 2; // invalid
	timeout_thr_sec_ = 0.5;//declare_parameter<double>("timeout_thr_sec", 0.5);

	lateral_controller_ = std::make_shared<trajectory_follower::MpcLateralController>(private_nh);

	std::string vehicle_platform_file;
	private_nh.param<std::string>("vehicle_platform_file", vehicle_platform_file, "vehicle_platform.yaml");
    ::common::getPlatformParam(vehicle_platform_file,platform_param);


	// 开放空间仿真使用独立的低速纵向控制，不需要构造道路纵向控制器。
	// 道路模式开启 open_lon_controller 时仍按原流程加载参数并创建控制器。
	if (open_lon_controller) {
		std::string param_node_dir = ros::package::getPath("launch_node");
		std::string lon_config_yaml_file = param_node_dir +
			std::string("/param/control/") + platform_param.vehicle_type +
			std::string("/longitudinal_controller_defaults.param.yaml");
		longitudinal_controller_ = std::make_shared<car::control::LonController>(
			lon_config_yaml_file, platform_param.vehicle_type);
	}
	sub_ref_path_ = nh_.subscribe("trajectory",1, &Controller::onTrajectory, this);

	latcmd_pub_  = nh_.advertise<driver_msgs::SteeringWheelCmd>("auto_chassis_steeringwheel_cmd", 1);

	loncmd_pub_  = nh_.advertise<driver_msgs::DriveCmd>("auto_chassis_drive_cmd", 10);

	latcontrol_debug = nh_.advertise<std_msgs::Float32>("lat_error", 10);	  
	if (open_space_execution_mode) {
		open_space_mpc_yaw_error_pub_ = nh_.advertise<std_msgs::Float64>(
			"control/open_space_mpc/yaw_error", 10);
		open_space_mpc_desired_tire_angle_pub_ = nh_.advertise<std_msgs::Float64>(
			"control/open_space_mpc/desired_tire_angle", 10);
		open_space_mpc_actual_tire_angle_pub_ = nh_.advertise<std_msgs::Float64>(
			"control/open_space_mpc/actual_tire_angle", 10);
		open_space_mpc_signed_speed_pub_ = nh_.advertise<std_msgs::Float64>(
			"control/open_space_mpc/signed_speed", 10);
		open_space_mpc_reference_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
			"control/open_space_mpc/reference_pose", 10);
		open_space_execution_status_pub_ =
			nh_.advertise<planning_msgs::OpenSpaceExecutionStatus>(
				"open_space_execution_status", 1, false);
	}

	sub_pose_ = nh_.subscribe("odomData", 1, &Controller::callbackPose, this);

	chassis_sub_ = nh_.subscribe("chassis", 1, &Controller::chassisCallback, this);
    
	sub_platoon_log = nh_.subscribe("platoon_log", 1, &Controller::platoonLogCallback, this);

	
	motion_start_sub_ = nh_.subscribe("chassis_motion_start_cmd", 1, &Controller::motionStartCallback, this);
	if (open_space_execution_mode) {
		open_space_task_reset_sub_ = nh_.subscribe(
			"/open_space_task_reset", 1,
			&Controller::openSpaceTaskResetCallback, this);
	}
	steer_compensation_sub = nh_.subscribe("/steering_offset_estimator", 1, &Controller::steerCompensationCallback, this);

	gearcmd_pub_ = nh_.advertise<driver_msgs::GearCmd>("auto_chassis_gear_cmd", 1);

	parking_brake_pub_ = nh_.advertise<driver_msgs::ParkingBrakeCmd>("auto_chassis_parking_brake_cmd", 1);

	platoonMission_sub_ = nh.subscribe("/PlatoonMission", 10, &Controller::callbackPlatoonMission, this); 
    platoonConfig_self_sub_  = nh.subscribe("PlatoonMission_self", 10, &Controller::callbackPlatoonMission, this);

	pub_diplay = nh_.advertise<visualization_msgs::MarkerArray>(
		"control/desired_velocity", 1);
    pub_heart = nh_.advertise<heartbeat_msgs::Heartbeat>("heartbeat", 1);

	if (enable_log){
		std::string logDir  = ros::package::getPath("launch_node");
		logDir = logDir + std::string("/log/");
		time_t now = time(0);
		tm *now_time = localtime(&now);
		std::string strTime = std::to_string(now_time->tm_year + 1900) + "-" + std::to_string(now_time->tm_mon + 1) + "-" + std::to_string(now_time->tm_mday);
		std::string logFile =logDir + strTime + std::string("_control_log_")+platform_param.id+std::string(".txt");
		int access_ok = access(logFile.c_str(),F_OK);
		logfile = std::ofstream(logFile,std::ios::app);
		logfile<< std::fixed;
		logfile.precision(8); //设置输出精度
		controlLog.lastLogPose.position.x = 0.0;
		controlLog.lastLogPose.position.y = 0.0;

		if (access_ok < 0)
		{
			     ROS_DEBUG_STREAM("[control] create control log: " << logFile);
			 logfile <<"时间         行驶距离(m)    当前位置(x)    当前位置(y)    当前朝向(rad)  期望位置(x)    期望位置(y)     期望朝向(rad)  当前速度(km/h) 期望速度(km/h) 速度误差(km/h)  横向误差(m)  航向误差(rad)   期望间距(m)    实际间距(m)    间距误差(m)    油门(%)    刹车(%)    期望加速度"<<std::endl;
		}
		else
				ROS_DEBUG_STREAM("[control] append control log: " << logFile);
		
	}
     driving_mode = 0;
 }



void Controller::onTrajectory(const planning_msgs::TrajectoryPointArray::Ptr msg)
{
	if (open_space_execution_mode) {
		stageOpenSpaceTrajectory(*msg);
		return;
	}
	activateTrajectory(*msg);
}

void Controller::activateTrajectory(const planning_msgs::TrajectoryPointArray &trajectory)
{
	if (trajectory.points.empty()) {
		return;
	}
	inputTrajectoryType = trajectory.type;
	sim_trajectory = trajectory;
	open_space_progress_index_ = 0;
	open_space_segment_end_candidate_start_ = ros::Time(0);
	if (!open_space_execution_mode) {
		// 道路时间索引仿真沿用原有虚拟尾点；泊车以规划段真实末点为停车点。
		planning_msgs::TrajectoryPoint lastPoint = sim_trajectory.points.back();
		lastPoint.x += std::cos(lastPoint.theta) * 0.5;
		lastPoint.y += std::sin(lastPoint.theta) * 0.5;
		lastPoint.relative_time += 100.0;
		sim_trajectory.points.push_back(lastPoint);
	}

	boost::shared_ptr<autoware_msgs::TrajectoryPointArray> trajectoryPtr(new autoware_msgs::TrajectoryPointArray);
	for (const auto &p : trajectory.points) {
        autoware_msgs::TrajectoryPoint point;

	    point.pose.position.x = p.x;
		point.pose.position.y = p.y;

		point.pose.orientation.w = std::cos(p.theta * 0.5);
		point.pose.orientation.z = std::sin(p.theta * 0.5);
		point.pose.orientation.y = 0;
		point.pose.orientation.x = 0;
		point.longitudinal_velocity_mps = lat_use_current_velocity_only?input_data_.vel:p.v;
		trajectoryPtr->points.push_back(point);
    }
	
	lon_input.trajectory_data = trajectory;

	current_waypoints_ = *trajectoryPtr;
    input_data_.current_trajectory_ptr = trajectoryPtr;
	is_forward_shift = trajectory.is_forward_shift;
}

const char *Controller::openSpaceExecutionStateName(OpenSpaceExecutionState state) const
{
	switch (state) {
	case OpenSpaceExecutionState::IDLE:
		return "IDLE";
	case OpenSpaceExecutionState::HOLD_STOP:
		return "HOLD_STOP";
	case OpenSpaceExecutionState::PREPARE_STEERING:
		return "PREPARE_STEERING";
	case OpenSpaceExecutionState::EXECUTING:
		return "EXECUTING";
	case OpenSpaceExecutionState::SEGMENT_END_HOLD:
		return "SEGMENT_END_HOLD";
	}
	return "UNKNOWN";
}

void Controller::setOpenSpaceExecutionState(OpenSpaceExecutionState state)
{
	if (open_space_execution_state_ == state) {
		return;
	}
	ROS_INFO_STREAM("[open_space_control] state "
		<< openSpaceExecutionStateName(open_space_execution_state_)
		<< " -> " << openSpaceExecutionStateName(state));
	open_space_execution_state_ = state;
}

bool Controller::isOpenSpaceStopTrajectory(
	const planning_msgs::TrajectoryPointArray &trajectory) const
{
	if (trajectory.points.size() < 2) {
		return true;
	}
	for (size_t index = 1; index < trajectory.points.size(); ++index) {
		if (std::fabs(trajectory.points[index].v) > 1.0e-3) {
			return false;
		}
	}
	return true;
}

double Controller::calculateOpenSpaceSteeringPrepareTarget(
	const planning_msgs::TrajectoryPointArray &trajectory) const
{
	if (trajectory.points.empty()) {
		return 0.0;
	}
	const double wheel_base = vehcileInfo->wheel_base_m;
	double steering_limit = std::numeric_limits<double>::infinity();
	if (std::fabs(vehcileInfo->w2s_primary_coeff) > 1.0e-3) {
		steering_limit = std::fabs(
			vehcileInfo->max_steer_angle_rad / vehcileInfo->w2s_primary_coeff);
	}
	// kappa 按轨迹点的排列方向计算。倒车时轨迹切向与车头方向相反，
	// 因此前轮转角需要对几何曲率反号，保持与 MPC 的倒车曲率处理一致。
	const double direction_sign = trajectory.is_forward_shift ? 1.0 : -1.0;
	const double target = std::atan(
		wheel_base * direction_sign * trajectory.points.front().kappa);
	return std::max(-steering_limit, std::min(target, steering_limit));
}

void Controller::stageOpenSpaceTrajectory(const planning_msgs::TrajectoryPointArray &trajectory)
{
	if (trajectory.points.empty()) {
		ROS_WARN("[open_space_control] ignore empty trajectory segment");
		return;
	}
	if (isOpenSpaceStopTrajectory(trajectory)) {
		has_pending_open_space_trajectory_ = false;
		activateTrajectory(trajectory);
		setOpenSpaceExecutionState(OpenSpaceExecutionState::EXECUTING);
		ROS_WARN("[open_space_control] activate stop segment immediately");
		return;
	}

	pending_open_space_trajectory_ = trajectory;
	has_pending_open_space_trajectory_ = true;
	open_space_steering_prepare_target_ =
		calculateOpenSpaceSteeringPrepareTarget(pending_open_space_trajectory_);
	open_space_steering_ready_cycles_ = 0;
	setOpenSpaceExecutionState(OpenSpaceExecutionState::HOLD_STOP);
	ROS_INFO_STREAM("[open_space_control] receive pending "
		<< (trajectory.is_forward_shift ? "forward" : "reverse")
		<< " segment, steering target=" << open_space_steering_prepare_target_ << " rad");
}



void Controller::chassisCallback(const driver_msgs::ChassisReport::ConstPtr &msg)
{
   
	input_data_.vel = (7 == msg->gear_location)?(-msg->current_velocity):msg->current_velocity;
	double steer  = (double)(msg->steering_wheel_angle -steer_compensation_degree)/vehcileInfo->w2s_primary_coeff;
	steer = steer / 180.0 * M_PI;
    
    boost::shared_ptr<autoware_msgs::SteeringReport> steer_ptr(new autoware_msgs::SteeringReport);
    //autoware_msgs::SteeringReport::Ptr steer_ptr = std::make_shared<autoware_msgs::SteeringReport>();
    steer_ptr->steering_tire_angle = steer;
	input_data_.current_steering_ptr = steer_ptr;
	lon_input.chassis_data = *msg;
	driving_mode = msg->driving_mode;

}
void Controller::motionStartCallback(const driver_msgs::MotionStartCmd::ConstPtr &msg)
{
	lon_input.motion_start_cmd = *msg;
}

void Controller::openSpaceTaskResetCallback(const std_msgs::Empty::ConstPtr &msg)
{
	(void)msg;
	// 清除上一任务尚未接管或正在执行的轨迹，防止新起点/新终点设置后
	// 继续沿旧轨迹输出 MPC 和纵向控制命令。
	pending_open_space_trajectory_ = planning_msgs::TrajectoryPointArray();
	has_pending_open_space_trajectory_ = false;
	sim_trajectory = planning_msgs::TrajectoryPointArray();
	current_waypoints_.points.clear();
	input_data_.current_trajectory_ptr.reset();
	lon_input.trajectory_data = planning_msgs::TrajectoryPointArray();
	lateral_output_ = boost::none;
	open_space_steering_ready_cycles_ = 0;
	open_space_steering_prepare_target_ = 0.0;
	open_space_progress_index_ = 0;
	open_space_segment_end_candidate_start_ = ros::Time(0);
	lon_input.motion_start_cmd.motion_start = 0;
	const auto mpc_controller = std::dynamic_pointer_cast<
		trajectory_follower::MpcLateralController>(lateral_controller_);
	if (mpc_controller && input_data_.current_steering_ptr) {
		mpc_controller->resetForOpenSpaceTrajectory(
			*input_data_.current_steering_ptr);
	}
	setOpenSpaceExecutionState(OpenSpaceExecutionState::IDLE);
	publishOpenSpaceHoldCommand();
	ROS_INFO("[open_space_control] previous parking task cleared");
}

void Controller::steerCompensationCallback(const std_msgs::Float32::ConstPtr &msg)

{
	steer_compensation = msg->data;
	ROS_DEBUG_STREAM("[control] steering compensation updated: "
		<< steer_compensation);
}
void Controller::platoonLogCallback(const platoon_msgs::PlatoonLog::ConstPtr &msg)
{
    platoonLog = *msg;
}


double Controller::distance2D(geometry_msgs::Point &p1 ,geometry_msgs::Point &p2) {
 
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}


// Keep the angle within [-pi, pi]
double Controller::normalized_angle(double angle) {
  if (angle > 180.0) {
      angle -= 2 * 180.0;
  } else if (angle < -180.0) {
      angle += 2 * 180.0;
  }
  return angle;
}

void Controller::callbackPose(const localization_msgs::Localization::ConstPtr &msg)
{
		/*
	//nav_msgs::Odometry::SharedPtr odom_ptr = std::make_shared<nav_msgs::nav_msgs>();
	odom_ptr->pose.pose.position.x = msg->utm_x - init_x;
	odom_ptr->pose.pose.position.y = msg->utm_y- init_y;

	double norm_yaw = normalized_angle(msg->yaw);
    //std::cout <<" odom_ptr->pose.pose.position.x "<<odom_ptr->pose.pose.position.x<<std::endl;
	//std::cout <<" odom_ptr->pose.pose.position.y "<<odom_ptr->pose.pose.position.y<<std::endl;
	//std::cout <<" norm_yaw "<<norm_yaw<<std::endl;
	norm_yaw = norm_yaw * 3.141592653589793 / 180.0;

	odom_ptr->pose.pose.orientation.w = std::cos(norm_yaw * 0.5);
	odom_ptr->pose.pose.orientation.z = std::sin(norm_yaw * 0.5);
	odom_ptr->pose.pose.orientation.y = 0;
	odom_ptr->pose.pose.orientation.x = 0;
    */
    boost::shared_ptr<nav_msgs::Odometry> odom_ptr(new nav_msgs::Odometry);
	constexpr double deg2rad = 3.1415926 /  180.0;
	double yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2 + heading_compensation_degree * deg2rad );
    controlLog.yaw = yaw;
	*odom_ptr = amathutils::getOdometryFromPosAndYaw(msg->location.pose.pose.position,yaw);	
	input_data_.current_odometry_ptr = odom_ptr;
	current_pose_ = *odom_ptr;
    lon_input.location_data = *msg;	
}


bool Controller::isTimeOut()
{
  const auto now = ros::Time::now();
  if ((now - lateral_output_->control_cmd.header.stamp).toSec() > timeout_thr_sec_) {
    //RCLCPP_ERROR_THROTTLE(
    //  get_logger(), *get_clock(), 5000 /*ms*/,
     // "Lateral control command too old, control_cmd will not be published.");
    return true;
  }

  return false;
}

void Controller::lonControl()
{
	/**************longititude control**************/
	 // const auto start_time = ros::Time::now().toSec();
	if (longitudinal_controller_->SetLocalData(lon_input)) 
	{
		/*纵向控制输出变量*/
		driver_msgs::GearCmd gear_cmd;
		driver_msgs::DriveCmd lon_drive_cmd;
		driver_msgs::ParkingBrakeCmd parking_brake_cmd;
		if (longitudinal_controller_->Process(gear_cmd, lon_drive_cmd,parking_brake_cmd)) 
		{
			mHeader.stamp = ros::Time::now();
			mHeader.seq++;
			lon_drive_cmd.header = mHeader;
			gear_cmd.header = mHeader;
			// 发布话题
			gearcmd_pub_.publish(gear_cmd);
			loncmd_pub_.publish(lon_drive_cmd);
			parking_brake_pub_.publish(parking_brake_cmd);

	        controlLog.throttle = lon_drive_cmd.throttle_pedal;
	        controlLog.brake_pedal = lon_drive_cmd.brake_pedal;
	        controlLog.deired_acc = lon_drive_cmd.acc_target;
		}
		else 
		{
		    ROS_WARN_THROTTLE(2.0,
				"longitudinal control is skipped since process failed.");
		}
    } 
	else 
	{
		    ROS_WARN_THROTTLE(2.0,
				"longitudinal control is skipped since input data is not ready.");
	}
      controlLog.current_v  = input_data_.vel * 3.6;
      controlLog.deired_v  = longitudinal_controller_->GetDesiredVelocity() * 3.6;
	  controlLog.v_error  = controlLog.deired_v - controlLog.current_v;

}

void Controller::publishOpenSpaceHoldCommand()
{
	driver_msgs::DriveCmd hold_cmd;
	mHeader.stamp = ros::Time::now();
	mHeader.seq++;
	hold_cmd.header = mHeader;
	hold_cmd.velocity_target = 0.0;
	hold_cmd.throttle_pedal = 0.0;
	hold_cmd.brake_pedal = std::max(0.0, std::min(100.0,
		open_space_hold_brake_pedal));
	// 车辆仍在运动时，减速度方向必须始终与当前速度相反；停稳后通过
	// brake_pedal 维持行车制动，允许控制器继续完成原地前轮准备。
	if (std::fabs(input_data_.vel) > open_space_stop_speed_tolerance) {
		hold_cmd.acc_target = -std::copysign(
			std::max(0.0, open_space_hold_deceleration), input_data_.vel);
	} else {
		hold_cmd.acc_target = 0.0;
	}
	loncmd_pub_.publish(hold_cmd);
	ROS_DEBUG_THROTTLE(0.5,
		"[open_space_control] hold brake: speed=%.3f mps, acc=%.3f mps2, brake=%.1f%%",
		input_data_.vel, hold_cmd.acc_target, hold_cmd.brake_pedal);
}

void Controller::updateOpenSpaceSegmentEndHold(double remaining_distance)
{
	if (open_space_execution_state_ != OpenSpaceExecutionState::EXECUTING) {
		open_space_segment_end_candidate_start_ = ros::Time(0);
		return;
	}

	const bool is_end_hold_candidate =
		remaining_distance <= std::max(0.0,
			open_space_segment_end_remaining_distance) &&
		!sim_trajectory.points.empty() &&
		std::fabs(sim_trajectory.points.back().v) <= 1.0e-3 &&
		std::fabs(input_data_.vel) <= open_space_stop_speed_tolerance;
	if (!is_end_hold_candidate) {
		open_space_segment_end_candidate_start_ = ros::Time(0);
		return;
	}

	const ros::Time now = ros::Time::now();
	if (open_space_segment_end_candidate_start_.isZero()) {
		open_space_segment_end_candidate_start_ = now;
		return;
	}
	// 使用严格大于，确保默认配置下实际连续停稳时间超过 1 秒。
	if ((now - open_space_segment_end_candidate_start_).toSec() <=
		std::max(0.0, open_space_segment_end_stable_duration)) {
		return;
	}

	planning_msgs::OpenSpaceExecutionStatus status;
	status.header.stamp = now;
	status.state = planning_msgs::OpenSpaceExecutionStatus::SEGMENT_END_HOLD;
	status.remaining_distance = remaining_distance;
	status.actual_speed = input_data_.vel;
	setOpenSpaceExecutionState(OpenSpaceExecutionState::SEGMENT_END_HOLD);
	open_space_execution_status_pub_.publish(status);
	ROS_DEBUG_STREAM("[open_space_control] segment end hold: remaining="
		<< remaining_distance << " m, speed=" << input_data_.vel << " mps");
}


void Controller::latControl()
{
	if (open_space_execution_mode &&
		open_space_execution_state_ != OpenSpaceExecutionState::EXECUTING) {
		if (open_space_execution_state_ == OpenSpaceExecutionState::HOLD_STOP) {
			if (std::fabs(input_data_.vel) > open_space_stop_speed_tolerance) {
				ROS_DEBUG_THROTTLE(0.5,
					"[open_space_control] wait stop before steering preparation: speed=%.3f mps",
					input_data_.vel);
				return;
			}
			setOpenSpaceExecutionState(OpenSpaceExecutionState::PREPARE_STEERING);
		}

		if (open_space_execution_state_ != OpenSpaceExecutionState::PREPARE_STEERING ||
			!has_pending_open_space_trajectory_) {
			return;
		}

		const double steering_limit = std::fabs(
			vehcileInfo->max_steer_angle_rad /
			vehcileInfo->w2s_primary_coeff);
		const double target_tire_angle = std::max(-steering_limit,
			std::min(open_space_steering_prepare_target_, steering_limit));
		const double actual_tire_angle = input_data_.current_steering_ptr
			? input_data_.current_steering_ptr->steering_tire_angle : 0.0;
		const bool steering_ready = input_data_.current_steering_ptr != nullptr &&
			std::fabs(actual_tire_angle - target_tire_angle) <=
				open_space_steering_prepare_tolerance;
		if (steering_ready) {
			++open_space_steering_ready_cycles_;
		} else {
			open_space_steering_ready_cycles_ = 0;
		}
		const bool ready = open_space_steering_ready_cycles_ >=
			std::max(1, open_space_steering_prepare_stable_cycles);

		driver_msgs::SteeringWheelCmd latCmd;
		constexpr double rad2deg = 180.0 / 3.1415926;
		latCmd.steering_wheel_angle = target_tire_angle * rad2deg *
			vehcileInfo->w2s_primary_coeff + steer_compensation;
		latCmd.steering_wheel_angle_speed = vehcileInfo->steer_rate_lim_dps;
		latcmd_pub_.publish(latCmd);

		std_msgs::Float64 desired_tire_angle;
		desired_tire_angle.data = target_tire_angle;
		open_space_mpc_desired_tire_angle_pub_.publish(desired_tire_angle);
		std_msgs::Float64 actual_tire_angle_msg;
		actual_tire_angle_msg.data = actual_tire_angle;
		open_space_mpc_actual_tire_angle_pub_.publish(actual_tire_angle_msg);
		ROS_DEBUG_THROTTLE(0.5,
			"[open_space_control] steering preparation: target=%.3f rad, actual=%.3f rad, "
			"stable=%d/%d, ready=%s",
			target_tire_angle, actual_tire_angle,
			open_space_steering_ready_cycles_,
			std::max(1, open_space_steering_prepare_stable_cycles),
			ready ? "true" : "false");
		if (ready) {
			const double steering_error = actual_tire_angle - target_tire_angle;
			activateTrajectory(pending_open_space_trajectory_);
			// 新段准备完成后，以当前实际前轮角初始化 MPC 的上一控制量、延迟缓冲
			// 和转角滤波状态，避免首周期重新输出上一段末端转角。
			const auto mpc_controller = std::dynamic_pointer_cast<
				trajectory_follower::MpcLateralController>(lateral_controller_);
			if (mpc_controller && input_data_.current_steering_ptr) {
				mpc_controller->resetForOpenSpaceTrajectory(
					*input_data_.current_steering_ptr);
			}
			has_pending_open_space_trajectory_ = false;
			setOpenSpaceExecutionState(OpenSpaceExecutionState::EXECUTING);
			ROS_INFO_STREAM("[open_space_control] activate segment after steering preparation: target="
				<< target_tire_angle << " rad, actual=" << actual_tire_angle
				<< " rad, error=" << steering_error << " rad");
		}
		return;
	}

    lateral_controller_->setInputData(input_data_);  // trajectory, odometry, steering
    const auto lat_out = lateral_controller_->run();
    lateral_output_ = lat_out ? lat_out : lateral_output_;  // use previous value if none.

	if (!lateral_output_) {
  	   return;
    }
	controlLog.lat_error = lateral_output_->lat_error;
	controlLog.nearestPose = lateral_output_->nearestPose;
	controlLog.pose = lateral_output_->pose;
	controlLog.yaw_error = lateral_output_->yaw_error;
	
	driver_msgs::SteeringWheelCmd latCmd;
	constexpr double rad2deg = 180.0 / 3.1415926;
 	latCmd.steering_wheel_angle = lateral_output_->control_cmd.steering_tire_angle * rad2deg * vehcileInfo->w2s_primary_coeff+steer_compensation;
	latCmd.steering_wheel_angle_speed = vehcileInfo->steer_rate_lim_dps;
	latcmd_pub_.publish(latCmd);
	
	std_msgs::Float32 lat_error;
	lat_error.data = lateral_output_->lat_error;
	latcontrol_debug.publish(lat_error);

	if (open_space_execution_mode) {
		std_msgs::Float64 yaw_error;
		yaw_error.data = lateral_output_->yaw_error;
		open_space_mpc_yaw_error_pub_.publish(yaw_error);

		std_msgs::Float64 desired_tire_angle;
		desired_tire_angle.data = lateral_output_->control_cmd.steering_tire_angle;
		open_space_mpc_desired_tire_angle_pub_.publish(desired_tire_angle);

		std_msgs::Float64 actual_tire_angle;
		actual_tire_angle.data = input_data_.current_steering_ptr
			? input_data_.current_steering_ptr->steering_tire_angle : 0.0;
		open_space_mpc_actual_tire_angle_pub_.publish(actual_tire_angle);

		std_msgs::Float64 signed_speed;
		signed_speed.data = input_data_.vel;
		open_space_mpc_signed_speed_pub_.publish(signed_speed);

		geometry_msgs::PoseStamped reference_pose;
		reference_pose.header.stamp = ros::Time::now();
		reference_pose.header.frame_id = "map";
		reference_pose.pose = lateral_output_->nearestPose;
		open_space_mpc_reference_pose_pub_.publish(reference_pose);
	}

}


void Controller::diplayDesireVelocity()
{
	visualization_msgs::MarkerArray markerArray;
    double desire = 0;

	desire = longitudinal_controller_->GetDesiredVelocity();
	std::ostringstream oss;
	oss << std::setprecision(4) << desire * 3.6;
	std::string vel_str = oss.str() + "km/h-desired";
	
	geometry_msgs::Point strLocalPos;
	strLocalPos.x = (vehcileInfo->vehicle_length_m-vehcileInfo->rear_overhang_m)*0.5+2.0;strLocalPos.y = -vehcileInfo->vehicle_width_m * 0.5-3.0 ;strLocalPos.z= 0;
	double yaw = amathutils::getPoseYawAngle(current_pose_.pose.pose);
	geometry_msgs::Point globalPoint = amathutils::localToGlobal(current_pose_.pose.pose.position,yaw,strLocalPos);
	DisplayConfig config;
	config.id = 4;
	config.r  = 1.0;
	config.g  = 1.0;
	config.scale_z = 1.5;
	config.position = globalPoint;
	markerArray.markers.push_back(DisPlay::stringMarker(vel_str,config));
	
    pub_diplay.publish(markerArray);
}


void Controller::sendHeart(unsigned char flag)
{
    static int heart_count = 0;

	heart_count++;
	
    static unsigned char  send_count = 0;
	if (0 == heart_count%50 )
	{   
	    heart_count = 0;
	    heartbeat_msgs::Heartbeat beat;
		beat.flag = flag;
		beat.heart_beat = send_count;
		pub_heart.publish(beat);
	    send_count++;
	}
}

size_t Controller::QueryLowerBoundPoint(const double relative_time,
                                                   planning_msgs::TrajectoryPointArray &trajectory,const double epsilon) const {
	auto &points  = trajectory.points;
	if (relative_time >= points.back().relative_time ){
	    return points.size() - 1;
	}
	auto func = [&epsilon](const planning_msgs::TrajectoryPoint& tp,
	                     const double relative_time) {
	    return tp.relative_time + epsilon < relative_time;
	};
	
	auto it_lower = std::lower_bound(points.begin(), points.end(), relative_time, func);
	return std::distance(points.begin(), it_lower);
}


void Controller::computeLonSim(driver_msgs::DriveCmd &lonCmd)
{  
    if (!sim_trajectory.points.size())
		return ;
	if (open_space_execution_mode) {
		computeOpenSpaceLonSim(lonCmd);
		return;
	}
    auto time_now = ros::Time::now().toSec();
    const double veh_rel_time =
      time_now - sim_trajectory.header.stamp.toSec();
	
    auto time_match_index = QueryLowerBoundPoint(veh_rel_time,sim_trajectory);
    int matchIndex =  std::min(time_match_index+2, sim_trajectory.points.size()-1);
	lonCmd.velocity_target = sim_trajectory.points[matchIndex].v;
	lonCmd.acc_target = sim_trajectory.points[matchIndex].a;
	if ( matchIndex == sim_trajectory.points.size()-1){
		// sim_trajectory 的最后一点是 onTrajectory() 人工追加的几何延长点，
		// 倒数第二点才是 planner 下发的真实末点。若真实末点已经要求停车，
		// 必须保持零速，不能再被下面“尚未到达虚拟末点则以 1m/s 续走”的旧逻辑覆盖。
		// 否则冲突停车轨迹一旦走完时间视野，车辆会重新起步，下一帧又被迫急停。
		const bool planner_requests_stop =
			sim_trajectory.points.size() >= 2 &&
			std::fabs(sim_trajectory.points[sim_trajectory.points.size()-2].v) <= 1.0e-3;
		if (planner_requests_stop)
		{
			lonCmd.velocity_target = 0.0;
			lonCmd.acc_target = 0.0;
			return;
		}

        int closestPoint = amathutils::closestPoint(sim_trajectory.points,current_pose_.pose.pose.position);
		if (closestPoint != (sim_trajectory.points.size()-1))
			lonCmd.velocity_target = sim_trajectory.is_forward_shift?1.0:-1.0;
		else
			lonCmd.velocity_target = 0.0;
    }
}

void Controller::run()
{
    if (!open_space_execution_mode && inputTrajectoryType != trajectoryType)
		return;
    sendHeart(2);
	const bool open_space_hold = open_space_execution_mode &&
		open_space_execution_state_ != OpenSpaceExecutionState::EXECUTING;
	if (open_space_hold) {
		// 不再沿用前一段轨迹的纵向控制输出；停车与前轮准备均由泊车执行器接管。
		publishOpenSpaceHoldCommand();
	} else if (open_lon_controller){
		lonControl();
		diplayDesireVelocity();
	}
	
	if (open_lat_controller)
		latControl();
    
	double move_distance = amathutils::distance2D(controlLog.lastLogPose.position,controlLog.pose.position);
	if (driving_mode && move_distance >= 0.2 ){
		if (controlLog.driving_s  < 1e-3)
			move_distance = 0.2;
		time_t now = time(0);
	    tm *now_time = localtime(&now);
	    std::string strTime = std::to_string(now_time->tm_year + 1900) +std::string("-")+ std::to_string(now_time->tm_mon + 1) +std::string("-")+ std::to_string(now_time->tm_mday)+std::string("-")
			                  + std::to_string(now_time->tm_hour)+std::string("-")+ std::to_string(now_time->tm_min);
		controlLog.driving_s +=  move_distance;
		double desired_yaw = amathutils::getPoseYawAngle(controlLog.nearestPose);
	    logfile <<strTime<<"  "<<controlLog.driving_s<<"    "<<controlLog.pose.position.x<<"    "<<controlLog.pose.position.y<<"    "
	    <<controlLog.yaw << "    "<<controlLog.nearestPose.position.x<<"    "
	    <<controlLog.nearestPose.position.y<<"    "<<desired_yaw <<"    "
        <<controlLog.current_v <<"    "<<controlLog.deired_v<<"    "<<controlLog.v_error<<"    "
        <<controlLog.lat_error<<"    "<<controlLog.yaw_error<<"    "<<platoonLog.desiredDistance<<"    "
        <<platoonLog.realDistance<<"    "<<platoonLog.errorDistance<<"    "<<controlLog.throttle<<"    "<<controlLog.brake_pedal<<"    "<<controlLog.deired_acc<<std::endl;
		controlLog.lastLogPose = controlLog.pose;
	}
	
    if (0 == driving_mode)
	{
	     controlLog.lastLogPose.position.x = 0.0;
	     controlLog.lastLogPose.position.y = 0.0;
		 controlLog.driving_s = 0.0;
	}
	
	if (open_simulate && !open_space_hold){
		driver_msgs::DriveCmd lonCmd;
		computeLonSim(lonCmd);
		loncmd_pub_.publish(lonCmd);
	}
}

void Controller::computeOpenSpaceLonSim(driver_msgs::DriveCmd &lonCmd)
{
	if (open_space_execution_state_ != OpenSpaceExecutionState::EXECUTING) {
		lonCmd.velocity_target = 0.0;
		lonCmd.acc_target = 0.0;
		return;
	}
	if (sim_trajectory.points.size() < 2) {
		lonCmd.velocity_target = 0.0;
		lonCmd.acc_target = 0.0;
		return;
	}
	const size_t last_index = sim_trajectory.points.size() - 1;
	open_space_progress_index_ = std::min(open_space_progress_index_, last_index);
	if (open_space_progress_index_ >= last_index) {
		lonCmd.velocity_target = 0.0;
		lonCmd.acc_target = 0.0;
		updateOpenSpaceSegmentEndHold(0.0);
		return;
	}

	const auto &vehicle_position = current_pose_.pose.pose.position;
	const size_t search_begin = open_space_progress_index_ > 2
		? open_space_progress_index_ - 2 : 0;
	size_t projection_segment_index = search_begin;
	double projection_ratio = 0.0;
	double projection_distance = std::numeric_limits<double>::infinity();
	for (size_t i = search_begin; i < last_index; ++i) {
		const auto &start = sim_trajectory.points[i];
		const auto &end = sim_trajectory.points[i + 1];
		const double segment_x = end.x - start.x;
		const double segment_y = end.y - start.y;
		const double segment_length_squared = segment_x * segment_x + segment_y * segment_y;
		if (segment_length_squared <= 1.0e-9) {
			continue;
		}
		const double ratio = std::max(0.0, std::min(1.0,
			((vehicle_position.x - start.x) * segment_x +
			 (vehicle_position.y - start.y) * segment_y) / segment_length_squared));
		const double projection_x = start.x + ratio * segment_x;
		const double projection_y = start.y + ratio * segment_y;
		const double distance = std::hypot(vehicle_position.x - projection_x,
		                                  vehicle_position.y - projection_y);
		if (distance < projection_distance) {
			projection_distance = distance;
			projection_segment_index = i;
			projection_ratio = ratio;
		}
	}
	// 已提交段内的空间进度单调递增，避免在换挡点附近因定位噪声选到车后的轨迹点。
	if (projection_segment_index >= open_space_progress_index_) {
		open_space_progress_index_ = projection_segment_index;
	} else {
		projection_segment_index = open_space_progress_index_;
		projection_ratio = 0.0;
	}

	const auto segment_length = [&](const size_t index) {
		return amathutils::distance2D(sim_trajectory.points[index],
		                              sim_trajectory.points[index + 1]);
	};
	double remaining_distance =
		(1.0 - projection_ratio) * segment_length(projection_segment_index);
	for (size_t i = projection_segment_index + 1; i < last_index; ++i) {
		remaining_distance += segment_length(i);
	}

	// 从连续投影点沿轨迹前进指定的物理预瞄距离。
	size_t target_index = projection_segment_index + 1;
	double lookahead_distance = (1.0 - projection_ratio) * segment_length(projection_segment_index);
	while (target_index < last_index && lookahead_distance < open_space_lookahead_distance) {
		lookahead_distance += segment_length(target_index);
		++target_index;
	}
	const bool segment_requests_stop =
		std::fabs(sim_trajectory.points[last_index].v) <= 1.0e-3;

	// 预瞄点到达末端零速点时不能立即停车。保留预瞄范围内最后一个非零速度，
	// 实际减速时机由下方基于剩余距离的制动约束决定。
	size_t speed_reference_index = target_index;
	if (segment_requests_stop) {
		while (speed_reference_index > open_space_progress_index_ &&
			std::fabs(sim_trajectory.points[speed_reference_index].v) <= 1.0e-3) {
			--speed_reference_index;
		}
	}
	double target_velocity = sim_trajectory.points[speed_reference_index].v;
	if (!sim_trajectory.is_forward_shift && target_velocity > 0.0) {
		target_velocity = -target_velocity;
	}
	if (segment_requests_stop) {
		const double braking_limit = std::sqrt(std::max(
			0.0, 2.0 * std::max(open_space_max_deceleration, 1.0e-3) * remaining_distance));
		target_velocity = std::copysign(
			std::min(std::fabs(target_velocity), braking_limit), target_velocity);
		if (remaining_distance <= 0.05) {
			target_velocity = 0.0;
		}
	}

	const double current_velocity = input_data_.vel;
	double acceleration_command = open_space_speed_kp * (target_velocity - current_velocity);
	const bool is_braking =
		(std::fabs(target_velocity) < std::fabs(current_velocity)) ||
		(current_velocity * target_velocity < 0.0);
	const double acceleration_limit = std::max(
		1.0e-3, is_braking ? open_space_max_deceleration : open_space_max_acceleration);
	acceleration_command = std::max(-acceleration_limit,
		std::min(acceleration_command, acceleration_limit));
	if (segment_requests_stop && std::fabs(current_velocity) > 1.0e-3) {
		const double safe_stop_speed = std::sqrt(std::max(
			0.0, 2.0 * std::max(open_space_max_deceleration, 1.0e-3) * remaining_distance));
		if (std::fabs(current_velocity) > safe_stop_speed) {
			// 单独使用比例环在低速时减速度衰减过快，可能越过泊车换挡点。
			// 此处强制满足物理可停车速度包络。
			acceleration_command = -std::copysign(open_space_max_deceleration,
				current_velocity);
		}
	}

	lonCmd.velocity_target = target_velocity;
	lonCmd.acc_target = acceleration_command;
	// 判断的是轨迹末点是否要求停车，而不是当前周期的动态减速速度。
	// 因此 0.30 m 宽容范围只用于识别“已停住但未到严苛换挡点”，
	// 不会改变上方纵向控制器原有的停车位置。
	updateOpenSpaceSegmentEndHold(remaining_distance);
}



}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

using namespace autoware::motion::control::trajectory_follower_nodes;


int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_follower_nodes");
  //ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Error);
  ros::NodeHandle nh;
  Controller  controller(nh);
  
  ros::Rate rate(50.0);
  while (ros::ok()) {
	controller.run();
	ros::spinOnce();
	rate.sleep();
  }
  return 0;
}
