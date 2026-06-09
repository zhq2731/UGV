#ifndef RECORD_DATA_NODE_H_
#define RECORD_DATA_NODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <sensor_msgs/NavSatFix.h>
#include  "ins_msgs/Ins.h"
#include  "localization_msgs/Localization.h"
#include <memory>
#include <vector>
#include <fstream> 

class TransformCoords
{
public:
  explicit TransformCoords(ros::NodeHandle &nh);
  void stopRecord();
  ros::NodeHandle nh_;	
  ros::NodeHandle private_nh_;
private:
	ros::Subscriber sub_gps_info_;
    ros::Publisher  pub_nav_;
	void gpsInfoDataCallback(const localization_msgs::Localization::ConstPtr &msg);
};


#endif  // RECORD_DATA_NODE_H_

