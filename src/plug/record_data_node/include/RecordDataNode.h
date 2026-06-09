#ifndef RECORD_DATA_NODE_H_
#define RECORD_DATA_NODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include  "utm/UTM.h"
#include  "ins_msgs/Ins.h"
#include  "localization_msgs/Localization.h"
#include  "display/display.h"

#include <memory>
#include <vector>
#include <fstream> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include  "vehicle_info_util/vehicle_info_util.hpp"

#include <std_msgs/Int32.h>
#include "yaml-cpp/yaml.h"

class RecordDataNode
{
public:
  explicit RecordDataNode(ros::NodeHandle &nh);
  void stopRecord();
  ros::NodeHandle nh_;	
  ros::NodeHandle private_nh_;

private:
	
	vehicle_info_util::VehicleInfoUtil  *vehcileInfo;
	ros::Subscriber sub_gps_info_;
	ros::Subscriber chassis_sub_;
	ros::Subscriber record_sub_;
	void gpsInfoDataCallback(const localization_msgs::Localization::ConstPtr &msg);
	void recordFlagCallback(const std_msgs::Int32::ConstPtr &msg);

	double resolution;
	bool smooth_whole_line;
	int  record_flag = 0;
	bool origin_set;
	projection::UtmProjector projector;
	geometry_msgs::Point prePoint;
	std::vector<std::vector<double>> routePoints;
};


#endif  // RECORD_DATA_NODE_H_

