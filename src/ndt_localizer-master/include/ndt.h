#pragma once

#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <queue>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <map>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <fstream>

#include "earth.h"
#include "localization_msgs/Localization.h"
#include"multi_lidar_calibrator.h"

class NdtLocalizer{
public:

    NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~NdtLocalizer();

private:
    ros::NodeHandle nh_, private_nh_;

    ros::Subscriber initial_pose_sub_;
    ros::Subscriber map_points_sub_;
    ros::Subscriber sensor_points_sub_;
    ros::Subscriber result_path_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher sensor_aligned_pose_pub_;
    ros::Publisher ndt_pose_pub_;
    ros::Publisher exe_time_pub_;
    ros::Publisher transform_probability_pub_;
    ros::Publisher iteration_num_pub_;
    ros::Publisher diagnostics_pub_;
    ros::Publisher path_pub_;
    ros::Publisher ins_pub_;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    Eigen::Matrix4f base_to_sensor_matrix_;
    Eigen::Matrix4f pre_trans, delta_trans;
    bool init_pose = false;

    std::string base_frame_;
    std::string map_frame_;

    // init guess for ndt
    geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg_;

    std::mutex ndt_map_mtx_;

    double converged_param_transform_probability_;
    std::thread diagnostic_thread_;
    std::map<std::string, std::string> key_value_stdmap_;

    double init_latitude;
    double init_longitude;
    double init_altitude;
    Eigen::Vector3d origin;
    Eigen::Vector3d last_pos;
    std::string input_imu_topic;
    std::queue<sensor_msgs::Imu> imu_v;
    bool first_lidar = true;
    bool online_calib;
    bool extrinsics_enable = false;
    std::string map_grid_path;
    std::map<std::string, std::vector<double>> pcd_index;
    pcl::PointCloud<pcl::PointXYZ> map_cloud;

    // function
    void init_params();
    void timer_diagnostic();

    bool get_transform(const std::string & target_frame, const std::string & source_frame,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr,
                       const ros::Time & time_stamp);
    bool get_transform(const std::string & target_frame, 
                       const std::string & source_frame,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);
    void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                    const geometry_msgs::PoseStamped & pose_msg);

    void callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_conv_msg_ptr);
    void callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    void callback_pointpath(const geometry_msgs::PoseStamped pose);

    void callback_imu(const sensor_msgs::Imu::ConstPtr & imu_msg_ptr);
    bool auto_calib(pcl::PointCloud<pcl::PointXYZ>& local_lidar);

};// NdtLocalizer Core