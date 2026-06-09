#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include  "localization_msgs/Localization.h"
#include <sstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

#include <iostream>
#include "amathutils_lib/amathutils.hpp"
#include "amathutils_lib/geometry.hpp"
#include "display/display.h"

#include <ros/package.h>

#include <chrono>
#include <fstream>
#include <driver_msgs/ChassisReport.h>
#include <driver_msgs/ModeCmd.h>


#include "platoon_common/platoon_common.h"
#include <tf/transform_broadcaster.h>

#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"
#include <platoon_msgs/PlatoonOpt.h>

#include <std_msgs/Int32.h>
#include "amathutils_lib/geometry.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "yaml-cpp/yaml.h"
#include <thread>
#include "driver_msgs/MotionStartCmd.h"
#include "common/common.h"

typedef unsigned char uint8;

struct CurrentPose
{
    double x;
	double y;
	double heading;
};
struct CooperationParam
{
	int policy;
	double time;
	double  distance;
	double  stop_distance;
	double  lateral_offset;
	bool use_path_planning;
	double deceleration;
};


class  Cooperation 
{
public:
	typedef void (Cooperation::*optFuncSelf)(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	typedef void (Cooperation::*optFunc)(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	explicit Cooperation(ros::NodeHandle &nh);	

    void callbackCurrentPose(const localization_msgs::Localization::ConstPtr &msg);
    void callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg);
    void callbackOpt(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg) ;
	void callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg) ;
	void callBackGoal(const geometry_msgs::PoseStamped::ConstPtr msg);
	void callbackTimer(const ros::TimerEvent &event);
	bool followerReachedMassPoint(int index);
	bool fronterReachedMassPoint(int index);

private:
	ros::NodeHandle nh_;	
	ros::NodeHandle private_nh_;	
    ros::Subscriber platoon_opt_sub_; //结构界面的命令。
    ros::Subscriber current_pose_sub_;
    ros::Subscriber chassis_sub_;
	
    ros::Subscriber platoonMember_sub_;
    ros::Subscriber platoonMission_sub_;
    ros::Subscriber platoonConfig_sub_;
    ros::Subscriber reference_line_sub_;

	ros::Subscriber goal_sub_;

	ros::Publisher  pub_diplay_;
	ros::Publisher	pub_mission_;
	ros::Publisher	pub_config_;
	ros::Publisher	pub_member_;
	ros::Publisher	pub_start_;
	ros::Publisher	mode_cmd_pub_;
	ros::Timer timer;

	void  none(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  build(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  join(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  leave(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  disolve(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  column(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  diamond(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  triangle(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  cancel_reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  mass(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void distribute(const platoon_msgs::PlatoonMission::ConstPtr &msg);

	void  running(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  noneSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  buildSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  joinSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  leaveSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  disolveSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  columnSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  diamondSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  triangleSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  reverseSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  massSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  distributeSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  cancel_reverseSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void  runningSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg);
	void displayLoop();
	
	bool toStartMove();
	bool toStopMove();
	bool toManual();
	bool toAuto();
	
	std::vector<geometry_msgs::Pose>  distributeGoals;
	vehicle_info_util::VehicleInfoUtil *vehcileInfo;
	CooperationParam param;
	common::PlatformParam platformParam;
	std::vector<optFuncSelf> optsFuncSelf;
	std::vector<optFunc> optsFunc;
	PlatoonType platoonType;
	bool  platoonBuild = false;
	std::map<int,platoon_msgs::PlatoonMember> platoonMembers;
	std::vector<uint8> vehicle_num_list;
    std::thread display_thread_;
	CurrentPose current_pose;
	nav_msgs::Odometry odom;
	bool massFlag;
	bool distributeFlag;
	platoon_msgs::PlatoonConfig platoon_config;
	
};


  
