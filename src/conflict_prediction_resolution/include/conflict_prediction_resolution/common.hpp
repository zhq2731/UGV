#pragma once

#include <cmath>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/time.h>
#include <std_msgs/ColorRGBA.h>

namespace conflict_prediction_resolution
{

struct Pose2d
{
  // 平面位姿：x/y 为地图坐标，yaw 为绕 z 轴的航向角。
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

double clamp(double value, double lower, double upper);
double normalizeAngle(double angle);
double distance2d(double ax, double ay, double bx, double by);

// 以下工具函数负责在内部二维位姿和 ROS 消息类型之间转换。
geometry_msgs::Point makePoint(double x, double y, double z = 0.0);
geometry_msgs::Quaternion makeQuaternion(double yaw);
geometry_msgs::Pose makePose(const Pose2d& pose);
geometry_msgs::PoseStamped makePoseStamped(const std::string& frame_id,
                                           const Pose2d& pose,
                                           const ros::Time& stamp);
Pose2d pose2dFromMsg(const geometry_msgs::Pose& pose);
std_msgs::ColorRGBA makeColor(double r, double g, double b, double a);

}  // namespace conflict_prediction_resolution
