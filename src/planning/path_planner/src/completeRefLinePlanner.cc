

#include "completeRefLinePlanner.h"
using namespace ugv::planning;


CompleteRefLinePlanner::CompleteRefLinePlanner(displayCallback callBack_):BasePlanner(callBack_)
{
	return ;
}

void CompleteRefLinePlanner::setInputData(InputData const & input_data) {
 	obsList = input_data.obsList;
	refArray = input_data.refArray;
    vehicleState = input_data.vehicleState;
	trajMsg2RefPoints(refArray,referencePoints);
	blockedObs_ = nullptr;
	return ;

}

void  CompleteRefLinePlanner::getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_)
{
    trajectory_  = trajectory;
    return ;
}


bool  CompleteRefLinePlanner::implement(double cur_time,const DiscretizedTrajectory prev_trajectory) {

	std::vector<planning_msgs::TrajectoryPoint>().swap(trajectory.points);
	
	planning_msgs::TrajectoryPointArray trajectoryTmp;
	int closestIndex = amathutils::closestPoint(refArray.points,vehicleState.x,vehicleState.y);
    trajectoryTmp.points.insert(trajectoryTmp.points.end(),refArray.points.begin()+ closestIndex,refArray.points.end());
		
	TrajectoryPoint planning_start_point;
	planning_start_point.path_point_.set_x (refArray.points[closestIndex].x);
	planning_start_point.path_point_.set_y (refArray.points[closestIndex].y);	
	PlanningConfig *planning_config = PlanningConfig::get_instance();
	auto iter = trajectoryTmp.points.end();
	
	if (planning_config->stop_obs_strategy){
		//停障代码
		ReferenceLine refenceLine(referencePoints);
		ReferenceLineInfo reference_line_info_(vehicleState,planning_start_point,refenceLine);
		reference_line_info_.Init(obsList);	
		vehicle_info_util::VehicleInfoUtil *vehicle_util = vehicle_info_util::VehicleInfoUtil::get_instance();
		CollisionChecker collisionCheck;
		double collision_s;
		
		if (collisionCheck.implement(&reference_line_info_,collision_s)){
			std::cout <<"----------------------implement collision-----------------------------"<<std::endl;
		    //double keep_s =  collision_s - (vehicle_util->wheel_base_m + vehicle_util->front_overhang_m) - planning_config->stopping_obs_distance ;
		    double keep_s =  collision_s - planning_config->stopping_obs_distance ;
			auto func = [](const planning_msgs::TrajectoryPoint &tp, const double path_s) {
	            return tp.s < path_s;};
	        iter = std::lower_bound(trajectoryTmp.points.begin(), trajectoryTmp.points.end(), keep_s, func);
			blockedObs_  = reference_line_info_.GetBlockingObstacle();
		}
	}
	
	trajectory.points.insert(trajectory.points.end(),trajectoryTmp.points.begin(),iter);
	trajectory.header.stamp = ros::Time(cur_time);
	
	callBack(&trajectory,&planning_start_point,nullptr,nullptr,nullptr);

	return true;
}

