#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <deque>
#include <vector>
#include <thread>
#include "yaml-cpp/yaml.h"
#include "localization_msgs/Localization.h"
#include "driver_msgs/ChassisReport.h"
#include <algorithm>
#include <iostream>
#include <numeric>
#include "vehicle_info_util/vehicle_info_util.hpp"
#include <amathutils_lib/amathutils.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>

typedef double  float64;
typedef int 	int32;

struct EstimatorInfo
{
    float64	wheelbase;
	int32	average_num;
	float64 vel_thres;
	float64	steer_thres;
	float64	offset_limit;
};

struct TwistInfo
{
	float64 linear_x;
	float64 linear_y;
	float64 linear_z;
	float64 angular_x;
	float64 angular_y;
	float64 angular_z;
	float64 front_wheel_angle;
};

class SteeringOffsetEstimator
{
public:
 	SteeringOffsetEstimator(ros::NodeHandle &nh);
	~SteeringOffsetEstimator() = default;

private:
	ros::NodeHandle nh_;	
    ros::NodeHandle private_nh_;

	void callbackLocalization(const localization_msgs::Localization::ConstPtr &msg);
    void callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg);
	void callbackTimerLocalizationFeedback(const ros::TimerEvent &event);
	
	ros::Subscriber localization_sub_;
    ros::Subscriber chassis_sub_;
	ros::Publisher steering_offset_estimator_pub_;

	ros::Timer timer_localization_;
	
	void udpThreadFunc();
	void updateOffset();
	float64 getOffset() const;

	EstimatorInfo 	estimatorInfo;
	TwistInfo		twistInfo;
	
	std::deque<double> front_wheel_angle_deviation_storage;
	
	float64 front_wheel_angle_deviation;
	float64 limited_front_wheel_angle_deviation;
	float64 heading;
	float64 heading_;
	float64 steering_offset_estimator;

	bool new_localization_flag;
	bool new_chassis_flag;
	bool first_flag;

	std::thread udp_thread_;
	
	vehicle_info_util::VehicleInfoUtil  *vehcileInfo;
};


