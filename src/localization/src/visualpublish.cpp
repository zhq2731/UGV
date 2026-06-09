#include "visualpublish.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/impl/octree2buf_base.hpp>
#include <pcl/octree/octree_search.h>

using namespace ros;
using namespace Eigen;
std::mutex mtx_;
ros::Publisher pub_pose;


void odo_traj_Callback(const localization_msgs::Localization::ConstPtr &odo_msg)
{
    std::cout << "odo_traj_Callback" << std::endl;
    mtx_.lock();
    pub_pose.publish(*odo_msg);
    mtx_.unlock();
}

void odo_ndt_Callback(const localization_msgs::Localization::ConstPtr &odo_msg)
{
    std::cout << "odo_ndt_Callback" << std::endl;
    mtx_.lock();
    pub_pose.publish(*odo_msg);
    mtx_.unlock();
}


visualpublish::visualpublish(ros::NodeHandle &n)
{
    pub_pose = n.advertise<localization_msgs::Localization>("/odomData", 1000);
    ros::Subscriber sub_odo_traj = n.subscribe("/local_pose", 200, odo_traj_Callback);

//    all_octree->setInputCloud(all_cloud);
}

void publish_pos(const nav_msgs::Odometry pose)
{
    pub_pose.publish(pose);
}
