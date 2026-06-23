#pragma once
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include <ros/package.h>
#include <amathutils_lib/amathutils.hpp>
#include <map>
#include <string>
#include <vector>
#include <pugixml.hpp>
#include <thread>
#include <amathutils_lib/geometry.hpp>
#include <float.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "driver_msgs/ChassisReport.h"
#include "localization_msgs/Localization.h"
#include "math.h"
#include "float.h"
#include <std_msgs/UInt8.h>
#include <limits>
#include <geometry_msgs/Quaternion.h>



#include "route/common.h"
#include "route/mapLoader.h"
#include "route/osmFileParser.h"
#include "route/topoGraph.h"
#include "route_msgs/Replan.h"
#include "route_msgs/MultiPoint.h"
#include "route_msgs/InitPoint.h"


#include "display/display.h"

enum PlanChoice{
	MULTI_PLAN = 0,
	MULTI_REPLAN = 1,
};

struct MapBoundary {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
};

struct InitPoint {
    UtmPoint point;
    geometry_msgs::Quaternion orientation;
};

class Route {
 public:
	Route(ros::NodeHandle &nh);
	ErrCode FindPath(TopoGraph &topoGraph,std::vector<UtmPoint> &singleUtmResult);
	ros::Publisher  display_pub_;
	ros::Publisher  route_pub_;
	ros::Publisher  cloud_display_pub_;
	ros::Publisher  map_request_pub_;
	ros::Subscriber initialPose_sub_;
	ros::Subscriber goal_sub_;
	ros::Subscriber multi_point_sub_;
	ros::Subscriber current_pose_sub_;
	ros::Subscriber chassis_sub_;
	ros::Subscriber replan_sub_;
	ros::Subscriber cloud_map_sub_;
	ros::Subscriber init_point_sub_;
	void callBackinitialPose(const geometry_msgs::PoseStamped::ConstPtr msg);
	void callBackgoal(const geometry_msgs::PointStamped::ConstPtr msg);
	void callBackMultiPointPlanning(const route_msgs::MultiPoint::ConstPtr msg);
	void callBackInitPoint(const route_msgs::InitPoint::ConstPtr msg);
	void callBackCurrentPose(const localization_msgs::Localization::ConstPtr &msg);
	void callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg);
	void callbackReplan(const route_msgs::Replan::ConstPtr &msg);
	void callbackCloudmap(const std_msgs::UInt8::ConstPtr &msg);
    void displayLoop();
	void displayOsm();
	void calculateMapBoundary();
	bool checkMultiPointsOutOfBoundary();
	bool multiPointsPlan(std::vector<UtmPoint> multiPoints,std::vector<std::vector<UtmPoint>> &multiUtmResults,double &plannedLength,PlanChoice planChoice,int &turnCount);
	std::map<double,int> eCalculate(TopoGraph &tmpGraph,double ePointX,double ePointY,std::vector<UtmPoint> &SingleUtmResult);
	std::map<double,int> sCalculate(TopoGraph &tmpGraph,double sPointX,double sPointY);
	ErrCode positivePlan(std::vector<UtmPoint> multiPoints,std::vector<std::vector<UtmPoint>> &ptUtmResults,PlanChoice planChoice,int &turnCount);
	ErrCode reservePlan(std::vector<UtmPoint> multiPoints,std::vector<std::vector<UtmPoint>> &reUtmResults,PlanChoice planChoice,int &turnCount);
	void addObs(TopoGraph &tmpGraph);
	void sThetasRmv(std::map<double,int> &thetas,TopoGraph &tmpGraph,std::vector<std::shared_ptr<ANode>> &nodesRmv);
	void sThetasAdd(TopoGraph &tmpGraph,std::vector<std::shared_ptr<ANode>> &nodesRmv);
	void eThetasRmv(std::map<double,int> &thetas,TopoGraph &tmpGraph,std::vector<std::shared_ptr<ANode>> &nodesRmv);
	void eThetasAdd(TopoGraph &tmpGraph,std::vector<std::shared_ptr<ANode>> &nodesRmv);
	void printResultsNodeId();
	void publishRouteSegmentMarkers();
 private:
    ros::NodeHandle nh_;	
    ros::NodeHandle private_nh_;
	MapLoader mapLoader;
	MapInfo mapInfo;
	TopoGraph topoGraph_;
	TopoGraph replan_topoGraph_;
	std::vector<UtmPoint> multiPoints;
	std::vector<UtmPoint> lastMultiPoints;
	std::vector<UtmPoint> reMultiPoints;
	std::vector<std::vector<UtmPoint>> pubUtmResults;
	std::vector<std::vector<UtmPoint>> pubUtmResults_;
	int pubIndex = -1;
	std::vector<ResultPoint> reservePlannedResults;
	File *file;		
	std::thread display_thread_;
	VehiclePositionInfo vehicleInfo;
	std::vector<UtmPoint> obsInfoVec;
	bool s_flag = true;
	bool e_flag = true;
	int lastPointsNum = 0;
	int lastRoutesNum = 0;
	int lastRouteStatusIndex = -100;
	double threshold = 500.0;
	MapBoundary map_boundary;
	InitPoint init_point;
};
