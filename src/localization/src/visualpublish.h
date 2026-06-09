#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "localization_msgs/Localization.h"
#include <mutex>

class visualpublish {
private:
    ros::Publisher pub_pose_;

    bool use_ndt_mode_;

public:
    visualpublish(ros::NodeHandle &n);

    void publish(const localization_msgs::Localization& pose) {
        pub_pose_.publish(pose);
    }
};
