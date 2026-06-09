#include <ros/ros.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "driver_msgs/ControlCmd.h"


driver_msgs::ControlCmd cmd;

bool cmd_init = false;

void controlCmdFromCtrl(const driver_msgs::ControlCmd::ConstPtr msg)
{
	cmd = *msg;
	cmd_init = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sendControlCmd");
  ros::NodeHandle nh;
  ros::Publisher  pub_control_cmd_;
  
  ros::Subscriber sub_cmd_;
  pub_control_cmd_  = nh.advertise<driver_msgs::ControlCmd>("vehicle_cmd", 1);

  sub_cmd_ = nh.subscribe("vehicle_cmd_from_ctrl", 10, &controlCmdFromCtrl);

  ros::Rate rate(100);
  while (ros::ok()) {
  	if (cmd_init)
	    pub_control_cmd_.publish(cmd);
	ros::spinOnce();
	rate.sleep();
  }
  ROS_INFO(" sendControlCmd  iteration end.");
  return 0;
}





