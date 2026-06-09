#pragma once
#include <iostream>
#include <vector>
#include <map>
#include  "pid/pid.h"




struct BoundryDeciderConfig
{
	bool is_lane_borrowing = false;
	bool is_pull_over= false;
	double pull_over_destination_to_adc_buffer=  25.0;
	double pull_over_destination_to_pathend_buffer=  10.0;
	double pull_over_road_edge_buffer=  0.15;
	double pull_over_approach_lon_distance_adjust_factor= 1.5;
	double adc_buffer_coeff= 1.0 ;
	bool is_extend_lane_bounds_to_include_adc= true ;

};

struct JerkPathOpimizerConfig
{
	double default_l_weight  = 1.0 ; //1.0
	double default_dl_weight = 20.0 ;//20.0
	double default_ddl_weight = 1000.0; //1000.0
	double default_dddl_weight = 50000.0; //50000.0
};


struct PathingConfig
{
	double kDefaultLaneWidth;
	std::vector<double> weight;
	std::vector<double> expansion;	
};

class PlanningConfig
{
public:
    PlanningConfig(const PlanningConfig&)=delete;
    PlanningConfig& operator=(const PlanningConfig&)=delete;
 
	static PlanningConfig* get_instance(){
		static PlanningConfig instance;
		return &instance;
	}
	std::map<std::string,PathingConfig>  pathingConfigs;
    BoundryDeciderConfig boundryConfig;
	JerkPathOpimizerConfig jerkPathOpimizerConfig;
    double kPathBoundsDeciderHorizon = 100.0;
    double kPathBoundsDeciderResolution = 0.5;
    double kDefaultLaneWidth = 5.0;
    double kDefaultRoadWidth = 20.0;
	double static_obstacle_nudge_l_buffer = 0.3;
	int kNumExtraTailBoundPoint = 20;
	int kNumExtraEndPoint = 10;
	int noplan_points_num = 8;
	double stopping_obs_distance = 10.0;
    double static_obstacle_speed_threshold = 0.5;
	double obstacle_lon_start_buffer= 3.0;
	double obstacle_lat_buffer= 0.4;
	double obstacle_lon_end_buffer= 2.0;

	double lateral_derivative_bound_default = 2.0;
	bool enable_osqp_debug = false;
	double trajectory_space_resolution = 1.0;
	double numerical_epsilon = 1e-6;

	double virtual_obs_width;
    double virtual_obs_length;
	bool   virtual_moveing_obs;

	int	mapParams_pointNum;
	double mapParams_resolution;
	int	mapParams_length;
	int mapParams_width;

	bool   open_velocity_planner;
	bool   open_path_planner;
	bool   stop_obs_strategy;
	
	bool   enable_trajectory_stitcher;
	double replan_lateral_distance_threshold;
    double replan_longitudinal_distance_threshold;
	int prepoint_num;
	double zudaunludaoche;
	bool open_perception_obs;
	
	PidConf pid_config_;
private:
    PlanningConfig(){    }
};


