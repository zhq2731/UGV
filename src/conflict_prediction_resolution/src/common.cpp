#include "conflict_prediction_resolution/common.hpp"

#include <algorithm>

#include <ros/time.h>
#include <tf/transform_datatypes.h>

namespace conflict_prediction_resolution
{

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

double normalizeAngle(double angle)
{
  // 将角度归一化到 [-pi, pi]，便于后续做角度差和插值。
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double distance2d(const double ax, const double ay, const double bx, const double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::Point makePoint(const double x, const double y, const double z)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::Quaternion makeQuaternion(const double yaw)
{
  return tf::createQuaternionMsgFromYaw(yaw);
}

geometry_msgs::Pose makePose(const Pose2d& pose)
{
  geometry_msgs::Pose msg;
  msg.position = makePoint(pose.x, pose.y, 0.0);
  msg.orientation = makeQuaternion(pose.yaw);
  return msg;
}

geometry_msgs::PoseStamped makePoseStamped(const std::string& frame_id,
                                           const Pose2d& pose,
                                           const ros::Time& stamp)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose = makePose(pose);
  return msg;
}

Pose2d pose2dFromMsg(const geometry_msgs::Pose& pose)
{
  Pose2d result;
  result.x = pose.position.x;
  result.y = pose.position.y;
  // ROS 四元数中只取平面运动需要的 yaw。
  result.yaw = tf::getYaw(pose.orientation);
  return result;
}

std_msgs::ColorRGBA makeColor(const double r, const double g, const double b, const double a)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace conflict_prediction_resolution
