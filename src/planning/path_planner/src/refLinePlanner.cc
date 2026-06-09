
#include "refLinePlanner.h"
using namespace ugv::planning;
using namespace ugv::common::math;



RefLinePlanner::RefLinePlanner(displayCallback callBack_):BasePlanner(callBack_)
{
	return ;
}

void RefLinePlanner::setInputData(InputData const & input_data){
 	obsList = input_data.obsList;
	refArray = input_data.refArray;
    vehicleState = input_data.vehicleState;
	trajMsg2RefPoints(refArray,referencePoints);
    updateParam(refArray.task_area);
	return ;
}


void  RefLinePlanner::getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_)
{
    trajectory_  = trajectory;
    return ;
}

bool  RefLinePlanner::implement(double cur_time,const DiscretizedTrajectory prev_trajectory) {
    
    PlanningConfig *planning_config = PlanningConfig::get_instance();
	TrajectoryPoint planning_start_point;
	ReferenceLine refenceLine(referencePoints);
	std::string replanReason;
	std::vector<TrajectoryPoint> traj_stitcher;
	traj_stitcher = TrajectoryStitcher::ComputeStitchingTrajectory(vehicleState, cur_time,0.1,0, true, &prev_trajectory,&replanReason);
    if (!replanReason.empty())
		std::cout <<replanReason<<std::endl;
	planning_start_point = traj_stitcher.back();
	
	Frame frame_;
	frame_.planning_start_point = planning_start_point;
    
	ReferenceLineInfo reference_line_info_(vehicleState,planning_start_point,refenceLine);
    
	reference_line_info_.Init(obsList);
	
	if ( refArray.task_area == std::string("normal_area") && reference_line_info_.total_obs_num())
	{
	     std::cout <<"change param to obstacle in normal_area "<<std::endl;
	     updateParam("road_obstacle_area"); 
	}
   
    PathBoundsDecider pathDecider;
	bool status_boundry = pathDecider.Process(&frame_,&reference_line_info_);

	if (!status_boundry){
		std::cout <<"PathBoundsDecider error "<<std::endl;
	    return false;
	}

	PiecewiseJerkPathOptimizer jerkPathOptimizer;
	bool status_optimizer = jerkPathOptimizer.Process(&reference_line_info_,planning_start_point,false);

	if (!status_optimizer){
		if (!prev_trajectory.empty()){
			return false;
		}
		
		else{
            trajectory = refArray;
			return true;
		}
	}
	
	blockedObs_ = reference_line_info_.GetBlockingObstacle();
    PathData pathData = reference_line_info_.GetCandidatePathData().back();
 
    DiscretizedPath trajectory_path;
    for (const auto& trajectory_point : traj_stitcher) {
      trajectory_path.push_back(trajectory_point.path_point());
    }
	
    const auto& best_ref_path = pathData.discretized_path();
    std::copy(best_ref_path.begin() + 1, best_ref_path.end(),
              std::back_inserter(trajectory_path));
    
	discretPath2TrajMsg(trajectory_path,trajectory);
	trajectory.header.stamp = ros::Time(cur_time);
	
    callBack(&trajectory,&planning_start_point,&reference_line_info_.GetDiplayPathBoundaries().back(),\
		&reference_line_info_.GetCandidatePathBoundaries().back(),&refenceLine);

	return true;
	
}



void RefLinePlanner::updateParam(std::string area)
{
    //std::cout <<"area changed to : "<<area <<std::endl;
    PlanningConfig *planning_config = PlanningConfig::get_instance();

	std::map<std::string,PathingConfig>::iterator iter = planning_config->pathingConfigs.find(area);
	
	if (iter == planning_config->pathingConfigs.end()) {
		std::cout <<"error ::: unknow area defined "<<std::endl;
		iter = planning_config->pathingConfigs.find("normal_area");
	}
	
	planning_config->obstacle_lon_start_buffer = iter->second.expansion[0];
	planning_config->obstacle_lon_end_buffer   = iter->second.expansion[1];
	planning_config->obstacle_lat_buffer       = iter->second.expansion[2];
	planning_config->kDefaultLaneWidth         = iter->second.kDefaultLaneWidth;

	planning_config->jerkPathOpimizerConfig.default_l_weight    = iter->second.weight[0];
	planning_config->jerkPathOpimizerConfig.default_dl_weight   = iter->second.weight[1];
	planning_config->jerkPathOpimizerConfig.default_ddl_weight  = iter->second.weight[2];
	planning_config->jerkPathOpimizerConfig.default_dddl_weight = iter->second.weight[3];

    /*
	std::cout <<" obstacle_lon_start_buffer: " <<planning_config->obstacle_lon_start_buffer<<std::endl;
	std::cout <<" obstacle_lon_end_buffer: "   <<planning_config->obstacle_lon_end_buffer  <<std::endl;
	std::cout <<" obstacle_lat_buffer:     "   <<planning_config->obstacle_lat_buffer      <<std::endl;
	std::cout <<" kDefaultLaneWidth:       "   <<planning_config->kDefaultLaneWidth        <<std::endl;

	std::cout <<" default_l_weight: "   <<planning_config->jerkPathOpimizerConfig.default_l_weight   <<std::endl;
	std::cout <<" default_dl_weight: "  <<planning_config->jerkPathOpimizerConfig.default_dl_weight  <<std::endl;
	std::cout <<" default_ddl_weight: " <<planning_config->jerkPathOpimizerConfig.default_ddl_weight <<std::endl;
	std::cout <<" default_dddl_weight: "<<planning_config->jerkPathOpimizerConfig.default_dddl_weight<<std::endl;
	*/
	
}


