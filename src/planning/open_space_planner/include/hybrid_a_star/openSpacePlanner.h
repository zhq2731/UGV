
#pragma once
#include "common/inputData.h"
#include "common/basePlanner.h"
#include <deque>
#include "cmath"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "hybrid_a_star/hybrid_a_star.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
//#include <glog/logging.h> 
#include <iostream>
#include <algorithm>
#include "yaml-cpp/yaml.h"

using float64_t = double;

class  OpenSpacePlanner : public BasePlanner
{
public:
	OpenSpacePlanner(displayCallback callBack_);
	void setInputData(InputData const & input_data) override;		
	bool  implement(double cur_time,const DiscretizedTrajectory prev_trajectory) override;
	
    void getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_) override;
private:
	geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
	// VehicleStatePtr current_vehicle_state_ptr_;
    geometry_msgs::PoseStampedPtr current_goal_pose_ptr_;
	// VehicleStatePtr goal_vehicle_state_ptr_;
    nav_msgs::OccupancyGridPtr current_costmap_ptr_;
	std::shared_ptr<VehicleState> current_vehicle_state_ptr_;
	std::shared_ptr<VehicleState> goal_vehicle_state_ptr_;

	OpenSpace_config openspace_config;
	bool prev_direction_;
	std::vector<Waypoint2D> prev_profile_;
	double map_resolution;
	planning_msgs::TrajectoryPointArray final_trajectory_;

	double car_vel;

	std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;

	planning_msgs::TrajectoryPointArray GetTraject(const HybridAStarType::VectorVec4d &path, HybridAStarType::Vec3d &start_state, HybridAStarType::Vec3d &start_state_map);
	planning_msgs::TrajectoryPointArray Velocity_Profile_output_os(OpenSpace_config &vel_config, double car_vel, planning_msgs::TrajectoryPointArray &traject_os);
};

