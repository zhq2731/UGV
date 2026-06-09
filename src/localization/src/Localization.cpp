#include "Localization.hpp"
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace std;

const double R2D = (180.0 / M_PI);
Localization::Localization()
{
    /*  配置车速与发布频率 */
    speed_mps   = 5.0;   // 5 m/s
    pub_hz      = 10.0; // 100 Hz
    dt          = 1.0 / pub_hz;

    stop = true;

    mode = true;
    
    replan_mode = false;

}

Localization::~Localization(){}



void Localization::registerPub(ros::NodeHandle  nh)
{

    pub_pose = nh.advertise<localization_msgs::Localization>("/odomData", 1000);
//    pub_pose = nh.advertise<localizer::Localization>("/local_pose", 1000);
}

// 计算两点距离
double Localization::distance(Point p1, Point p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 角度归一化到 [-pi, pi]
double Localization::normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void Localization::pubpose()
{

    if ( stop == false && mode == true  && !segments.empty() && replan_mode == false)
    {


        /* 4.1 计算本周期行驶距离 */
        double stepDist = speed_mps * dt;

        /* 4.2 更新到最新段 */
        while (stepDist > 0 && currSegIdx < segments.size())
        {

            Segment &seg = segments[currSegIdx];
            double remainInSeg = seg.length - segAccumDist;
            if (stepDist <= remainInSeg)
            {
                segAccumDist += stepDist;
                stepDist = 0;
            }
            else
            {
                stepDist      -= remainInSeg;
                segAccumDist   = 0;
                ++currSegIdx;
            }
        }

        /* 4.3 处理路径结束 */
        if (currSegIdx >= segments.size())
        {

               // std::cout << "Path finished. Exit.\n";
                return;
        }

        /* 4.4 计算当前位姿 */
        Segment &seg = segments[currSegIdx];
        double x = seg.start.x + segAccumDist * seg.dx;
        double y = seg.start.y + segAccumDist * seg.dy;
        double z = seg.start.z + segAccumDist * seg.dz;
        double yaw = std::atan2(seg.dy, seg.dx); // 车头朝向
        // std::cout << "算出的航向角： " << yaw << std::endl;

        /* 4.5 发布（这里直接打印，用户可换成 ROS/ZeroMQ/UDP） */
        // std::printf("t=%.3f  x=%.3f  y=%.3f  yaw=%.3f\n",
        //            simTime, x, y, yaw);


        localization_msgs::Localization posedata;
        posedata.location.pose.pose.position.x = x;
        posedata.location.pose.pose.position.y = y;
        posedata.location.pose.pose.position.z = z;



        //z and t modify
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        posedata.location.pose.pose.orientation = tf2::toMsg(q);


        // posedata.header.stamp = ros::Time::now();
        
  

        Eigen::Vector3d pos_local;
        pos_local(0) = y;
        pos_local(1) = x;
        pos_local(2) = -1.0 * z;

        Vector3d pos = Earth::local2global(origin, pos_local);
        pos(0) *= R2D;
        pos(1) *= R2D;
        // ins.latitude = -1.0 *  pose.pose.position.y;
        // ins.longitude = pose.pose.position.x;
        // ins.altitude = pose.pose.position.z;
        posedata.original_ins.latitude = pos(0);
        posedata.original_ins.longitude = pos(1);
        posedata.original_ins.altitude = pos(2);
        // cout << setprecision(15) << "latitude is " << pos(0) << endl;
        // cout << setprecision(15) <<"longitude is " << pos(1) << endl;
        // cout << setprecision(15) <<"altitude is " << pos(2) << endl;
        // cout << "publish pose " << endl;
        pub_pose.publish(posedata);

        simTime += dt;
    }
    else{
         /* 4. 主循环：按 100 Hz 发布定位 */
        currSegIdx   = 0;
        segAccumDist = 0.0;   // 在当前段已跑距离
        simTime      = 0.0;   // 累计仿真时间
        if(    replan_mode == true)
        {
            replan_mode == false;
        }
        segments.clear();

    }

    return;
}




void Localization::addload(const std::vector<Point> Load, const double speed)
{
   cout << "addload" << endl;
    /* 1. 配置路径点 —— 用户可任意修改 */
    std::vector<Point> waypoints = Load;
    speed_mps = speed;

    double totalLength = 0.0;

    for (size_t i = 0; i + 1 < waypoints.size(); ++i)
    {
        Segment seg;
        seg.start  = waypoints[i];
        seg.end    = waypoints[i + 1];
        seg.length = distance(seg.start, seg.end);
        seg.dx     = (seg.end.x - seg.start.x) / seg.length;
        seg.dy     = (seg.end.y - seg.start.y) / seg.length;
        seg.dz     = (seg.end.z - seg.start.z) / seg.length;
        segments.push_back(seg);
        totalLength += seg.length;
    }
    if (segments.empty())
    {
        std::cerr << "No valid path!\n";
        return ;
    }

   
    return;
}
