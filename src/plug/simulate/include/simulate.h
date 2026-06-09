#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <fstream>

#include <sstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#include "planning_msgs/TrajectoryPointArray.h"
#include "ins_msgs/Ins.h"
#include "localization_msgs/Localization.h"
#include "amathutils_lib/amathutils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include <ros/package.h>

#include "driver_msgs/DriveCmd.h"
#include "driver_msgs/ChassisReport.h"
#include "driver_msgs/SteeringWheelCmd.h"
#include <chrono>
#include "sensor_driver_msgs/GpswithHeading.h"
#include "utm/UTM.h"
#include "ray_msgs/Report.h"

#include "platoon_common/platoon_common.h"

#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"
#include "driver_msgs/MotionStartCmd.h"
#include "common/common.h"
#include "route_msgs/MultiPoint.h"
#include "route_msgs/InitPoint.h"

struct SimulateParam
{
	bool is_forward;
	double v;
	bool reverse_path;
	bool open_simulate_platoon;
};

struct TrajectoryPointData
{
	double x;
	double y;
	double z;
	double v;
	double theta;
	double relative_time;
};

class Simulate
{
public:
	explicit Simulate(ros::NodeHandle &nh);

	virtual ~Simulate() {}
	ros::Subscriber loncmd_sub_;
	ros::Subscriber latcmd_sub_;
	ros::Subscriber clickPoint_sub_;
	ros::Subscriber initialPose_sub_;
	ros::Subscriber goal_sub_;

	ros::Publisher bit_report_pub_;
	ros::Publisher gpsPub_;
	ros::Publisher pub_chassis_;
	ros::Publisher pub_odometry_;
	ros::Publisher pub_steering_;
	ros::Publisher pub_velcoty_;

	ros::Subscriber platoonMember_sub_;
	ros::Subscriber platoonMission_sub_;
	ros::Subscriber platoonConfig_sub_;

	ros::Subscriber platoonMember_self_sub_;
	ros::Subscriber platoonMission_self_sub_;
	ros::Subscriber motion_start_sub_;
	ros::Subscriber platoonConfig_self_sub_;
	ros::Subscriber init_point_sub_;
	ros::Subscriber trajectory_sub_;
	ros::Subscriber multi_point_sub_;

	void publishInitPose();
	bool readGpsTxts(std::string &fileName, std::vector<geometry_msgs::Point> &points);
	bool readCordTxts(std::string &fileName, std::vector<geometry_msgs::Point> &points);

	void onLonControlCommand(const driver_msgs::DriveCmd::ConstPtr msg);
	void onLatControlCommand(const driver_msgs::SteeringWheelCmd::ConstPtr msg);
	void publishChassis();
	void publishGpsdata();
	double normalizeRadian(const double _angle);
	void callbackTimer(const ros::TimerEvent &event);
	void callbackTimer2(const ros::TimerEvent &event);

	void initPoseCallBack(
		const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);

	void trajectoryCallBack(const planning_msgs::TrajectoryPointArray::ConstPtr &msg);
	double interpolateTheta(double theta1, double theta2, double alpha);
	void publishLocalizationMsg(double x, double y, double z, double theta);

	void callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg);
	void callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg);
	void callBackinitialPose(const geometry_msgs::PoseStamped::ConstPtr msg);
	void callBackgoal(const geometry_msgs::PointStamped::ConstPtr msg);
	void motionStartCallback(const driver_msgs::MotionStartCmd::ConstPtr &msg);
	void callBackMultiPointPlanning(const route_msgs::MultiPoint::ConstPtr msg);
	void callBackInitPoint(const route_msgs::InitPoint::ConstPtr msg);
	void callbackCloudmap(const std_msgs::UInt8::ConstPtr &msg);


	common::PlatformParam platformParam;

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	bool lon_timer_inited;
	bool lat_timer_inited;

	ros::Timer timer;
	ros::Timer timer2;
	projection::UtmProjector projector_;

private:
	geometry_msgs::Point pos;
	double theta;
	double v;
	double acc;
	vehicle_info_util::VehicleInfoUtil *vehcileInfo;
	bool inited;
	int steering;
	int plan_point_count = 0;

	std::chrono::system_clock::time_point lon_last_time;
	std::chrono::system_clock::time_point lat_last_time;
	SimulateParam param;
	std::vector<unsigned char> vehicle_num_list;
	bool built = false;
	unsigned char motion_start = 0;
	std::map<double, TrajectoryPointData> trajectory_map;
	ros::Time header_time;
	bool last_is_forward_shift;

	ros::Subscriber cloud_map_sub_;
	geometry_msgs::Point map_origin_;
};
