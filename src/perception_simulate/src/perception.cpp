#include "perception_simulate/perception.h"

ObstacleSubscriber::ObstacleSubscriber(ros::NodeHandle &nh)
    : nh_(nh)
{
    transform_matrix_ = Eigen::Matrix4d::Identity();

    sub_obstacles_ = nh_.subscribe(
        "/GlobalMultiObjectTracker", // topic 名
        10,                          // queue size
        &ObstacleSubscriber::obstacleCallback,
        this);

    sub_localization_ = nh_.subscribe(
        "/odomData", // topic 名
        10,          // queue size
        &ObstacleSubscriber::localizationCallback,
        this);

    pub_roi_obstacles_ = nh_.advertise<perception_msgs::PredictionObstacles>(
        "/MultiObjectTracker", 10);

    timer_ = nh_.createTimer(ros::Duration(0.1), &ObstacleSubscriber::timercallback, this);

    ROS_INFO("ObstacleSubscriber initialized.");
}

void ObstacleSubscriber::obstacleCallback(
    const perception_msgs::PredictionObstacles::ConstPtr &msg)
{

    roi_obstacles_msg_.timestamp = msg->timestamp;
    roi_obstacles_msg_.vehicle_position = msg->vehicle_position;
    roi_obstacles_msg_.vehicle_quaternion = msg->vehicle_quaternion;
    roi_obstacles_msg_.prediction_obstacles.clear();
    int i = 1;
    int obs_id = 0;
    std::cout << "obstacle size is:" << roi_obstacles_msg_.prediction_obstacles.size() << std::endl;
    for (auto obstacle : msg->prediction_obstacles)
    {
        std::cout << "num: " << i << std::endl;
        std::cout << "obs is static " << (int)obstacle.is_static << std::endl;
        obstacle.is_static = true;
        Eigen::Vector4d position(obstacle.perception_obstacle.position.x,
                                 obstacle.perception_obstacle.position.y,
                                 obstacle.perception_obstacle.position.z,
                                 1);
                                 
        Eigen::Vector4d transformed_position = transform_matrix_.inverse() * position; // utm转换到右前上
        std::cout << "x = " << transformed_position[0] << std::endl;
        std::cout << "y = " << transformed_position[1] << std::endl;

        double dx1 = obstacle.perception_obstacle.polygon.points.at(0).x - obstacle.perception_obstacle.polygon.points.at(1).x;
        double dy1 = obstacle.perception_obstacle.polygon.points.at(0).y - obstacle.perception_obstacle.polygon.points.at(1).y;
        double length = sqrt(dx1 * dx1 + dy1 * dy1);

        double dx2 = obstacle.perception_obstacle.polygon.points.at(0).x - obstacle.perception_obstacle.polygon.points.at(3).x;
        double dy2 = obstacle.perception_obstacle.polygon.points.at(0).y - obstacle.perception_obstacle.polygon.points.at(3).y;

        double width = sqrt(dx2 * dx2 + dy2 * dy2);
        obstacle.perception_obstacle.length = length;
        obstacle.perception_obstacle.width = width;
        std::cout << "length " << length << "  width " << width << std::endl;

        // 简单的ROI过滤条件：只保留距离小于50米的障碍物
        // if (std::fabs(transformed_position[0]) <= 50.0 && std::fabs(transformed_position[1]) <= 100.0)
        // {
        obstacle.perception_obstacle.id = obs_id;
        roi_obstacles_msg_.prediction_obstacles.push_back(obstacle);
        std::cout << "obs id " << obs_id << std::endl;
        obs_id++;
        // }
        i++;
        std::cout << "endddddddddddddddddddddd" << std::endl;
    }
}
void ObstacleSubscriber::localizationCallback(
    const localization_msgs::Localization::ConstPtr &msg)
{
    Eigen::Quaterniond q;
    q.w() = msg->location.pose.pose.orientation.w; // 右前上变换到UTM
    q.x() = msg->location.pose.pose.orientation.x;
    q.y() = msg->location.pose.pose.orientation.y;
    q.z() = msg->location.pose.pose.orientation.z;
    q.normalize();
    Eigen::Matrix3d rotation_matrix(q);

    transform_matrix_(0, 3) = msg->location.pose.pose.position.x; // 右前上变换到UTM
    transform_matrix_(1, 3) = msg->location.pose.pose.position.y;
    transform_matrix_(2, 3) = 0;
    transform_matrix_.block<3, 3>(0, 0) = rotation_matrix;
}

void ObstacleSubscriber::timercallback(const ros::TimerEvent &event)
{
    pub_roi_obstacles_.publish(roi_obstacles_msg_);
}