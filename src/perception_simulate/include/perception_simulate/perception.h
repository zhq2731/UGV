#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "perception_msgs/PredictionObstacles.h"
#include "perception_msgs/TrajectoryPoint.h"
#include "perception_msgs/PerceptionObstacle.h"
#include "localization_msgs/Localization.h"



class ObstacleSubscriber
{
public:
    ObstacleSubscriber(ros::NodeHandle& nh);

private:
    // 回调函数
    void obstacleCallback(const perception_msgs::PredictionObstacles::ConstPtr& msg);
    void localizationCallback(const localization_msgs::Localization::ConstPtr& msg);
    void timercallback(const ros::TimerEvent& event);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_obstacles_;
    ros::Subscriber sub_localization_;
    ros::Publisher pub_roi_obstacles_;
    ros::Timer timer_;
    Eigen::Matrix4d transform_matrix_;//转换位置
    perception_msgs::PredictionObstacles roi_obstacles_msg_;

};

#endif // PERCEPTION_H  