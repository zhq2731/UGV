#include "Localization.hpp"
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include "driver_msgs/ModeCmd.h"
// #include "localizer/TrajectoryPointArray.h"
// #include "localizer/TrajectoryPoint.h"
// #include "localizer/Replan.h"
#include "driver_msgs/MotionStartCmd.h"
#include "visualpublish.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdlib>
#include <thread>

using namespace std;
Localization localization;
double speed;
const double D2R = (M_PI / 180.0);
void publish_static_tf(
        const std::string & parent,
        const std::string & child,
        double tx, double ty, double tz,
        double roll, double pitch, double yaw)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = parent;
    ts.child_frame_id  = child;

    ts.transform.translation.x = tx;
    ts.transform.translation.y = ty;
    ts.transform.translation.z = tz;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    static_broadcaster.sendTransform(ts);
}



// void traj_Callback(const localizer::TrajectoryPointArray::ConstPtr &traj_msg)
// {
//     cout << "trajectory is here" << endl;

//     std::vector<Point> waypoints;
//     for(int i = 0; i < traj_msg->points.size(); i++)
//     {
//         Point point;
//         point.x = traj_msg->points[i].x;
//         point.y = traj_msg->points[i].y;
//         point.z = traj_msg->points[i].z;
//         waypoints.push_back(point);
//         cout << point.x << ", " << point.y << ", " << point.z << endl;
//     }
    
//     // std::thread localization_thread(localization.addload,waypoints, speed);
//     localization.addload(waypoints, speed);
// }


// void speed_Callback(const std_msgs::Float32::ConstPtr &speed_msg)
// {
//     cout << "configure is here" << endl;
//     cout << "speed = " << speed_msg->data << endl;
//     speed = speed_msg->data;
// }

// void replan_Callback(const localizer::Replan::ConstPtr &speed_msg)
// {
//     cout << "replan is here" << endl;

//     if(speed_msg->points.size() == 0)
//     {
//         localization.replan_mode = false;
        
//     }
//     else
//         localization.replan_mode = true;
//     return;

// }

void start_Callback(const driver_msgs::MotionStartCmd::ConstPtr &start_msg)
{
    cout << "start is here" << endl;
  
    if(start_msg->motion_start > 0)
    {
        cout <<"stop = " << false << endl;
        localization.stop = false;
        system("/home/nvidia/Desktop/zxn/src/rosbag.sh");

    }
    else
    {

        localization.stop = true;

    }
   

}

void mode_Callback(const std_msgs::UInt8::ConstPtr &mode_msg)
{
    cout << "mode is here" << endl;
    if(mode_msg->data == 1)
    {
        cout << "有卫星" << endl;
        localization.mode = true;
        return;
    }
    else
    {
        cout << "无卫星,启动基于地图定位" << endl;
        localization.mode = false;
        system("/home/nvidia/Desktop/zxn/src/ndt.sh");
    }
}

void odo_ndt_Callback(const localization_msgs::Localization::ConstPtr &odo_msg)
{
    std::cout << "odo_ndt_Callback" << std::endl;
    localization.pub_pose.publish(*odo_msg);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

 //   NdtLocalizer ndt_localizer(nh, private_nh);
//    ndt_localizer.localization_1.pub_pose = localization.pub_pose;

//    std::shared_ptr<visualpublish> prosviz;
//    prosviz.reset( new visualpublish(nh) );
    localization.registerPub(nh);

    nh.param<double>("latitude", localization.origin[0], 39.4723557);
    nh.param<double>("longitude", localization.origin[1], 115.6257681);
    nh.param<double>("altitude", localization.origin[2], 86.9899999999);
    localization.origin *= D2R;

    ros::Timer time1 = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent& event){
        localization.pubpose();
    });

    cout << "sub" << endl;
//    ros::Subscriber sub_traj = nh.subscribe("/cloud_route_display", 1000, traj_Callback);
//    ros::Subscriber sub_speed = nh.subscribe("/remote_chassis_speed_cmd", 1000, speed_Callback);
//    ros::Subscriber sub_replan = nh.subscribe("/replan", 1000, replan_Callback);
    ros::Subscriber sub_start = nh.subscribe("/chassis_motion_start_cmd", 1000, start_Callback);
    ros::Subscriber sub_mode = nh.subscribe("/mode", 1000, mode_Callback);
    // ros::Subscriber sub_odo_ndt = nh.subscribe("/ndt_ins", 200, odo_ndt_Callback);

    // publish_static_tf("base_link", "rslidar",  0,0,0,  0,0,0);  // 0 0 0 0 0 0
    // publish_static_tf("map",       "world",    0,0,0,  0,0,0);  // 0 0 0 0 0 0

    ros::spin();
    return 0;

}
