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

	using std::placeholders::_1;
	private_nh.param<double>("steer_compensation_degree",  steer_compensation_degree, 0.0);

	private_nh.param<bool>("open_lon_controller",  open_lon_controller, true);
	private_nh.param<bool>("open_lat_controller",  open_lat_controller, true);
	private_nh.param<bool>("open_simulate",  open_simulate, false);
	private_nh.param<double>("simulate_velocity",  simulate_velocity, 3.0);
	private_nh.param<double>("heading_compensation_degree",  heading_compensation_degree, 0.0);
	

	private_nh.param<bool>("lat_use_current_velocity_only",  lat_use_current_velocity_only,false);
	private_nh.param<bool>("enable_control_log",  enable_log,true);
	
	vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();
	vehcileInfo->loadVehicleingParam(private_nh);

	lon_input.motion_start_cmd.motion_start = 2; // invalid
	const double ctrl_period = 0.03;
	timeout_thr_sec_ = 0.5;//declare_parameter<double>("timeout_thr_sec", 0.5);

	lateral_controller_ = std::make_shared<trajectory_follower::MpcLateralController>(private_nh);

	std::string vehicle_platform_file;
	private_nh.param<std::string>("vehicle_platform_file", vehicle_platform_file, "vehicle_platform.yaml");
    ::common::getPlatformParam(vehicle_platform_file,platform_param);


	std::string param_node_dir  = ros::package::getPath("launch_node");
	std::string lon_config_yaml_file = param_node_dir + std::string("/param/control/")+platform_param.vehicle_type+std::string("/longitudinal_controller_defaults.param.yaml");

	longitudinal_controller_ =
	  std::make_shared<car::control::LonController>(lon_config_yaml_file,platform_param.vehicle_type);

	std::cout <<"-------------------------"<<std::endl;
	sub_ref_path_ = nh_.subscribe("trajectory",1, &Controller::onTrajectory, this);

	latcmd_pub_  = nh_.advertise<driver_msgs::SteeringWheelCmd>("auto_chassis_steeringwheel_cmd", 1);

	loncmd_pub_  = nh_.advertise<driver_msgs::DriveCmd>("auto_chassis_drive_cmd", 10);

	latcontrol_debug = nh_.advertise<std_msgs::Float32>("lat_error", 10);	  

	sub_pose_ = nh_.subscribe("odomData", 1, &Controller::callbackPose, this);

	chassis_sub_ = nh_.subscribe("chassis", 1, &Controller::chassisCallback, this);
    
	sub_platoon_log = nh_.subscribe("platoon_log", 1, &Controller::platoonLogCallback, this);

	
	motion_start_sub_ = nh_.subscribe("chassis_motion_start_cmd", 1, &Controller::motionStartCallback, this);
	steer_compensation_sub = nh_.subscribe("/steering_offset_estimator", 1, &Controller::steerCompensationCallback, this);

	gearcmd_pub_ = nh_.advertise<driver_msgs::GearCmd>("auto_chassis_gear_cmd", 1);

	parking_brake_pub_ = nh_.advertise<driver_msgs::ParkingBrakeCmd>("auto_chassis_parking_brake_cmd", 1);

	//timer = nh_.createTimer(ros::Duration(ctrl_period),&Controller::callbackTimerControl,this);
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
		     std::cout <<"----------just build the log file for now,add file header"<<std::endl;
			 logfile <<"时间         行驶距离(m)    当前位置(x)    当前位置(y)    当前朝向(rad)  期望位置(x)    期望位置(y)     期望朝向(rad)  当前速度(km/h) 期望速度(km/h) 速度误差(km/h)  横向误差(m)  航向误差(rad)   期望间距(m)    实际间距(m)    间距误差(m)    油门(%)    刹车(%)    期望加速度"<<std::endl;
		}
		else
			std::cout <<"---------------logfile: "<<logFile<<" has been exist"<<std::endl;
		
	}
     driving_mode = 0;
 }



void Controller::onTrajectory(const planning_msgs::TrajectoryPointArray::Ptr msg)
{
    inputTrajectoryType = msg->type;
	planning_msgs::TrajectoryPointArray lat_trajectory = *msg;
    sim_trajectory = *msg;
	planning_msgs::TrajectoryPoint lastPoint = sim_trajectory.points[sim_trajectory.points.size()-1];
	
	lastPoint.x += std::cos (lastPoint.theta)*0.5;
	lastPoint.y += std::sin (lastPoint.theta)*0.5;
	sim_trajectory.points.push_back(lastPoint);
	sim_trajectory.points[sim_trajectory.points.size()-1].relative_time = 100.0;
	//if (!lat_trajectory.is_forward_shift)
      //  std::reverse(lat_trajectory.points.begin(),lat_trajectory.points.end());  

    boost::shared_ptr<autoware_msgs::TrajectoryPointArray> trajectoryPtr(new autoware_msgs::TrajectoryPointArray);
    for (const auto &p:lat_trajectory.points){
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
	
	lon_input.trajectory_data = *msg;

	current_waypoints_ = *trajectoryPtr;
    input_data_.current_trajectory_ptr = trajectoryPtr;
	is_forward_shift = lat_trajectory.is_forward_shift;
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


void Controller::steerCompensationCallback(const std_msgs::Float32::ConstPtr &msg)

{
      std::cout <<"get steer pensation call back "<<std::endl;
      steer_compensation = msg->data;
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
		    ROS_WARN("longitudinal control is skipped since process failed.");
		}
    } 
	else 
	{
		    ROS_WARN("longitudinal control is skipped since input data is not ready.");
	}
	  const auto end_time = ros::Time::now().toSec();

      controlLog.current_v  = input_data_.vel * 3.6;
      controlLog.deired_v  = longitudinal_controller_->GetDesiredVelocity() * 3.6;
	  controlLog.v_error  = controlLog.deired_v - controlLog.current_v;

}


void Controller::latControl()
{

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


void Controller::callbackTimerControl(const ros::TimerEvent &event)
{
    if (open_lon_controller){
		lonControl();
	    diplayDesireVelocity();
    }
	if (open_lat_controller)
		latControl();
	if (open_simulate){
		driver_msgs::DriveCmd lonCmd;
		double v = simulate_velocity;
		lonCmd.velocity_target = is_forward_shift ? v : -v;
		loncmd_pub_.publish(lonCmd);
	}
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
    auto time_now = ros::Time::now().toSec();
    const double veh_rel_time =
      time_now - sim_trajectory.header.stamp.toSec();
	
    auto time_match_index = QueryLowerBoundPoint(veh_rel_time,sim_trajectory);
    int matchIndex =  std::min(time_match_index+2, sim_trajectory.points.size()-1);
	lonCmd.velocity_target = sim_trajectory.points[matchIndex].v;
	lonCmd.acc_target = sim_trajectory.points[matchIndex].a;
	if ( matchIndex == sim_trajectory.points.size()-1){
        int closestPoint = amathutils::closestPoint(sim_trajectory.points,current_pose_.pose.pose.position);
		if (closestPoint != (sim_trajectory.points.size()-1))
			lonCmd.velocity_target = sim_trajectory.is_forward_shift?1.0:-1.0;
		else
			lonCmd.velocity_target = 0.0;
    }
}

void Controller::run()
{
    if (inputTrajectoryType != trajectoryType)
		return;
	
    sendHeart(2);
	if (open_lon_controller){
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
	
	if (open_simulate){

		driver_msgs::DriveCmd lonCmd;
	    /*
		private_nh.getParam("/trajectory_follower/simulate_velocity", simulate_velocity);
		double v = simulate_velocity;
		lonCmd.velocity_target = is_forward_shift ? v : -v;
		*/
		computeLonSim(lonCmd);
		loncmd_pub_.publish(lonCmd);
	}
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
  ROS_INFO("trajectory_follower_nodes The end of node.");
  return 0;
}

