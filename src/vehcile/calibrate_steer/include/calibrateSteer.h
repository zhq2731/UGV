#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <cmath>
#include <ros/package.h>
#include<dirent.h>
#include <fstream> 

class  CalibrateSteer
{
public:
	explicit CalibrateSteer(ros::NodeHandle &nh);
	virtual ~CalibrateSteer() {}
	Eigen::Vector3d computeCircle(std::vector<Eigen::Vector2d> &points);
	void calibrate();
    void readDataFileList(std::vector<std::string> &dataFileList);
    Eigen::Vector3d computeCubicPloy(std::vector<double> &x, std::vector<double> &y);
private:
	double wheel_base;
	double max_steer;
	std::vector<std::string> dataFileList;
	std::vector<std::vector<Eigen::Vector2d>> circlesData;
	std::vector<double> steerDegree;
	std::vector<double> steerRadian;
	ros::NodeHandle private_nh_;	
	ros::NodeHandle nh_;	

};



