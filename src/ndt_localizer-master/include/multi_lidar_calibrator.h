#ifndef PROJECT_MULTI_LIDAR_CALIBRATOR_H
#define PROJECT_MULTI_LIDAR_CALIBRATOR_H

#include <string>
#include <vector>
#include <map>
//map 是一种关联容器， 提供一对一的关联， 关联的形式为： KEY----VALUE（键值对），关键字不能重复
#include <chrono>
//C++11中，<chrono>是标准模板库中与时间有关的头文件

//该消息表示带有时间标签和参考坐标的估计位姿

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#define __APP_NAME__ "multi_lidar_calibrator"

class ROSMultiLidarCalibratorApp

{
public:

	double voxel_size_;
	double ndt_epsilon_;
	double ndt_step_size_;
	double ndt_resolution_;

	double initial_x_;
	double initial_y_;
	double initial_z_;
	double initial_roll_;
	double initial_pitch_;
	double initial_yaw_;
    int max_score;

	int ndt_iterations_;
    Eigen::Matrix4f current_guess_;//4×4float矩阵


	typedef pcl::PointXYZ PointT;

public:
	ROSMultiLidarCalibratorApp();
    bool PerformNdtOptimize(const pcl::PointCloud<PointT> map, const pcl::PointCloud<PointT> source_point, const Eigen::Matrix4d pose);
    bool PerformNdtOptimize(const pcl::PointCloud<PointT> map, const pcl::PointCloud<PointT> source_point);
};

#endif //PROJECT_MULTI_LIDAR_CALIBRATOR_H
