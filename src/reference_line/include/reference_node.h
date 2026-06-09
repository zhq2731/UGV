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
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "planning_msgs/TrajectoryPoint.h"
#include "planning_msgs/TrajectoryPointArray.h"
#include "ins_msgs/Ins.h"
#include "localization_msgs/Localization.h"
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
#include "display/display.h"

#include <ros/package.h>

#include <chrono>
#include <fstream>
#include <driver_msgs/ChassisReport.h>
#include <driver_msgs/DriveCmd.h>
#include <heartbeat_msgs/Heartbeat.h>

#include "smoother/fem_pos_deviation_smoother.h"
#include <tf/transform_broadcaster.h>
#include  "utm/UTM.h"
#include "platoon_common/platoon_common.h"

#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"

//beili
#include <lanelet_map_msgs/Way.h>
#include "common/common.h"
#include "common/error_code.h"

using float64_t = double;

typedef unsigned char uint8;

struct Point2D {
  Point2D() : x(0), y(0) {}

  Point2D(double x, double y) : x(x), y(y) {}

  // Point2D(const Point3D& point):x(point.x),y(point.y){}

  Point2D &operator+=(const Point2D &rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }

  Point2D &operator-=(const Point2D &rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }

  friend Point2D operator+(Point2D lhs, const Point2D &rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Point2D operator-(Point2D lhs, const Point2D &rhs) {
    lhs -= rhs;
    return lhs;
  }

  friend Point2D operator*(Point2D lhs, const int scale) {
    lhs.x *= scale;
    lhs.y *= scale;
    return lhs;
  }

  friend Point2D operator*(Point2D lhs, const double scale) {
    lhs.x *= scale;
    lhs.y *= scale;
    return lhs;
  }

  friend Point2D operator/(Point2D lhs, const double scale) {
    lhs.x /= scale;
    lhs.y /= scale;
    return lhs;
  }

  double x, y;
};


struct CurrentState {
  Point2D position;
  double yaw;
  double vel;

  CurrentState() = default;

  CurrentState(double x, double y) {
    this->position = Point2D(x, y);
    this->yaw = 0;
    this->vel = 0;
  }

  CurrentState(double x, double y, double yaw) {
    this->position = Point2D(x, y);
    this->yaw = yaw;
    this->vel = 0;
  }

  CurrentState(double x, double y, double yaw, double vel) {
    this->position = Point2D(x, y);
    this->yaw = yaw;
    this->vel = vel;
  }
};


struct ReferenceParam
{
	std::string pathFile;
	double forward_s;
	double backward_s;
	bool reverse_path;
	bool direct_to_control;
	bool is_forward_shift;
	double normal_area_forward_s;
	double others_area_forward_s;
	std::string task_area;
};


class  ReferenceNode 
{
public:
	
	typedef void (ReferenceNode::*optFunc)(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	explicit ReferenceNode(ros::NodeHandle &nh);
	void draw_route( std::vector<geometry_msgs::Point>   &globalRoute);
	
	void sendHeart(unsigned char flag);
	ros::NodeHandle nh_;	
	ros::NodeHandle private_nh_;	
    ros::Subscriber sub_current_pose;
    ros::Subscriber clickPoint_sub_;
    ros::Subscriber chassisSub;
    ros::Subscriber pathSub;
	ros::Subscriber route_sub;
    ros::Subscriber platoonMember_sub_ ;
    ros::Subscriber platoonMission_sub_;
    ros::Subscriber platoonConfig_sub_ 	;
    ros::Subscriber platoonMember_self_sub_ ;
    ros::Subscriber platoonConfig_self_sub_ ;
    ros::Subscriber platoonMission_self_sub_;
    ros::Subscriber sub_shift_forward;
	
    ros::Publisher  pub_reference;
    ros::Publisher  pub_diplay;
    ros::Publisher	pub_heart;

private:
	projection::UtmProjector projector;
	bool pose_inited = false;
	bool new_global_reference;
	CurrentState state;
	nav_msgs::Odometry odom;
	double zero_x_	;
	double zero_y_ ;
	planning_msgs::TrajectoryPointArray global_reference_line_utm;
	planning_msgs::TrajectoryPointArray lastRefLine;
	planning_msgs::TrajectoryPointArray lastSmoothedLine;
	
	planning_msgs::TrajectoryPointArray lastRefLineRaw;
	
	vehicle_info_util::VehicleInfoUtil *vehcileInfo;

	std::vector<geometry_msgs::Point> global_reference_line_wgs84;
	std::vector<geometry_msgs::Point> global_route;
	std::vector<geometry_msgs::Point> real_route;
	std::vector<geometry_msgs::Point> leader_route;
	double leader_route_length = 0.0;
	std::map<int,platoon_msgs::PlatoonMember> platoonMembers;
	
	std::vector<optFunc> optsFunc;
	ros::Timer timer;
	int lastClosestIndex ;
	ReferenceParam param;
	uint8  platoonType;
	uint8  shape;
	bool   platoonBuild = false;
	platoon_msgs::PlatoonConfig platoon_config;
	common::PlatformParam platformParam;
	std::string vehicle_id;
	std::vector<unsigned char> vehicle_num_list;
	uint8 trajectoryType = PlatoonType::NONE;
    FemPosDeviationSmootherConfig smoother_config;
	common::ErrorCode readGpsTxts(std::string &fileName, std::vector<geometry_msgs::Point> &trajectoryTotal);

	void set_vel( planning_msgs::TrajectoryPointArray &trajectory);

	template <typename T,typename V>    // 成员函数模板
	void forward_traj(int closestIndex ,const std::vector<T> &in,std::vector<V> &out,double forward_s,double backward_s);
	template <typename T,typename V>    // 成员函数模板
	void backward_traj(int closestIndex ,const  std::vector<T> &in,std::vector<V> &out,double forward_s,double backward_s);

	template <typename T> 
	bool deleteExceptionPoints(std::vector<T> &in);
	bool generateReferenceLine(const std::vector<geometry_msgs::Point> &route,planning_msgs::TrajectoryPointArray &referenceLine);
	
	void setTrajectory(std::vector<geometry_msgs::Point> &trajectory_);
	void smoother_init();
	int getClosestPointThrottleBrake();
	bool smoothFem(planning_msgs::TrajectoryPointArray &refTrajectory,int lIndex,int rIndex);
	std::pair<size_t, size_t> findNearestIndexPair(
	  const std::vector<float64_t> & accumulated_lengths, const float64_t target_length);
	void DeNormalizePoints(
		std::vector<std::pair<double, double>>* xy_points) ;
	void NormalizePoints(
		std::vector<std::pair<double, double>>* xy_points) ;
	bool tooBigCurlture(double x,double y,planning_msgs::TrajectoryPointArray &refTrajectory);
	void callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg) ;
	void callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg) ;
	void callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg);
	void callbackRemotePath(const planning_msgs::TrajectoryPointArray::ConstPtr &msg);
	void callbackReverse(const std_msgs::Int32::ConstPtr &msg);
	void currentPoseCallback(const localization_msgs::Localization::ConstPtr &msg);
    void clickPointCallBack(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg);
    void callbackTimerReference(const ros::TimerEvent &event);
	void addExtraPath( std::vector<geometry_msgs::Point> &inPath);
	void  trajMsg2GeometryMsg  (const planning_msgs::TrajectoryPointArray &outTrajectory,std::vector<geometry_msgs::Point> &outPoints);

	int    forwardAppendPath(const planning_msgs::TrajectoryPointArray &preTraj,const planning_msgs::TrajectoryPointArray &curTraj,
planning_msgs::TrajectoryPointArray &addpendPath,double refAppendLen);
	
	
	double backwardAppendPath(const planning_msgs::TrajectoryPointArray &preTraj,const planning_msgs::TrajectoryPointArray &curTraj,
planning_msgs::TrajectoryPointArray &addpendPath,double refAppendLen);
	void  updatePathToLeader();

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
	void  distribute(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  running(const platoon_msgs::PlatoonMission::ConstPtr &msg);
    
};


template <typename T,typename V>	// 成员函数模板
void ReferenceNode::forward_traj(int closestIndex ,const std::vector<T> &in,std::vector<V> &out,double forward_s,double backward_s)
{
    assert(in.size() > 0);
	
	int lowerIndex = 0,upIndex = 0;
	float backS = 0,forwardS = 0;
	for (int i = closestIndex;i >=1; i--){
	   backS += amathutils::distance2D(in[i],in[i-1]);
	   lowerIndex = i-1;
	   if (backS > backward_s)
		   break;
	}
	for (int i = closestIndex; i < in.size()-1; i++){
		 forwardS += amathutils::distance2D(in[i],in[i+1]); 
	     upIndex = i + 1;
		 if (forwardS > forward_s)
			 break;
	}
	for (int i = lowerIndex; i <= upIndex; i++){
	    V v_point;
		v_point.x = in[i].x;
		v_point.y = in[i].y;
	    out.push_back(v_point);
	}
	return ;
}



template <typename T,typename V>	// 成员函数模板
void ReferenceNode::backward_traj(int closestIndex ,const  std::vector<T> &in,std::vector<V> &out,double forward_s,double backward_s)
{
    assert(in.size() > 0);
	int lowerIndex = closestIndex,upIndex = closestIndex;
	float backS = 0,forwardS = 0;
	for (int i = closestIndex;i >=1; i--){
	   backS += amathutils::distance2D(in[i],in[i-1]);
	   lowerIndex = i - 1;
	   if (backS > forward_s)
		   break;
	}
	
	for (int i = closestIndex; i < in.size()-1; i++){
		 forwardS += amathutils::distance2D(in[i],in[i+1]);	 
	     upIndex = i + 1;
		 if (forwardS > backward_s)
			 break;
	}
	for (int i = lowerIndex; i <= upIndex; i++){
	    V v_point;
		v_point.x = in[i].x;
		v_point.y = in[i].y;
	    out.push_back(v_point);
	}
}


template <typename T>
bool ReferenceNode::deleteExceptionPoints(std::vector<T> &in)
{
    if (in.size() < 2)
		return true;
    std::vector<T> out;
	out.push_back(in[0]);
	out.push_back(in[1]);
	
	T p0 = amathutils::minus_2d(in[1],in[0]);
	T p1 ;
	size_t preIndex = 1;
	for (size_t i = 2;i < in.size();i++){
	    T p1 = amathutils::minus_2d(in[i],in[preIndex]);
		if (amathutils::dot(p0,p1) <= 0)
			continue;
		
		preIndex++;
		out.push_back(in[i]);
		p0 = p1;
	}
	in = out;
	return true;
}






