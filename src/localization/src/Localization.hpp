#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <float.h>
#include <ros/ros.h>
#include <mutex>
#include "localization_msgs/Localization.h"
#include "nav_msgs/Odometry.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include "visualpublish.h"
#include "earth.h"

using namespace std;

struct Point
{
    double x{0}, y{0}, z{0};
};

  /* 2. 预处理：计算每段总长度、单位方向向量 */
    struct Segment
    {
        Point  start, end;
        double length;
        double dx, dy, dz; // 单位向量
    };

class Localization
{
public:
    Localization();
    ~Localization();
    void registerPub(ros::NodeHandle nh);
    void addload(const std::vector<Point> Load, const double speed);
    void pubpose();
    std::mutex mtx;

private:
    double distance(Point p1, Point p2);
    double normalizeAngle(double a);
    
public:
    double  speed_mps;
    double  pub_hz ;
    double  dt;
    ros::Publisher pub_pose;
    bool stop;  //接受开始与停止信号；开始为0，停止为1；
    bool mode; //接受定位模式信号；速度定位为1，地图定位为0；
    bool replan_mode;
     /* 4. 主循环：按 100 Hz 发布定位 */
    size_t currSegIdx   = 0;
    double segAccumDist = 0.0;   // 在当前段已跑距离
    double simTime      = 0.0;   // 累计仿真时间
    std::vector<Segment> segments;

    Eigen::Vector3d origin;

};

