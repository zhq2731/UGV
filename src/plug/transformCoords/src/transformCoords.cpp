
#include "transformCoords.h"
#include <algorithm>
#include <memory>
#include <utility>
#include <fstream>
#include <ros/package.h>


TransformCoords::TransformCoords(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{
	sub_gps_info_ = nh_.subscribe("/odomData", 1, &TransformCoords::gpsInfoDataCallback, this);
	
	pub_nav_      = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
}



void TransformCoords::gpsInfoDataCallback(const localization_msgs::Localization::ConstPtr &msg)
{		
	sensor_msgs::NavSatFix navSatFix;
	navSatFix.latitude  = msg->original_ins.latitude;
	navSatFix.longitude = msg->original_ins.longitude;
	navSatFix.altitude  = msg->original_ins.altitude;
	navSatFix.position_covariance_type = 3;
	pub_nav_.publish(navSatFix);
	return;
}


int main(int argc ,char *argv[])
{        
	ros::init(argc, argv, "TransformCoords");
	ros::NodeHandle nh;
	ROS_INFO("TransformCoords start.");
	auto node_ptr = std::make_shared<TransformCoords>(nh);
	ros::spin();
	ROS_INFO(" TransformCoords  iteration end.");

	return 0;
}


