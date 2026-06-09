#pragma once
#include<dirent.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/ColorRGBA.h>
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
#include <tf/transform_broadcaster.h>
#include  "utm/UTM.h"


using float64_t = double;


class  DataCompare 
{
public:
	explicit DataCompare(ros::NodeHandle &nh);
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
    ros::Publisher  pub_diplay;
    ros::Timer timer;
    void setColor(std_msgs::ColorRGBA * cl, double r, double g, double b, double a);

private:
	projection::UtmProjector projector;
	std::vector<std::pair<std::string,planning_msgs::TrajectoryPointArray>> trajectorys;
	bool readGpsTxts(std::string &fileName, planning_msgs::TrajectoryPointArray &trajectoryTotal);
	void callbackTimerReference(const ros::TimerEvent &event);
	void readDataFileList(std::vector<std::string> &dataFileList);	

};



