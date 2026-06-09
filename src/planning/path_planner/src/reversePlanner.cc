
#include "reversePlanner.h"
using namespace ugv::planning;


ReversePlanner::ReversePlanner(displayCallback callBack_):BasePlanner(callBack_)
{
	return ;
}

void ReversePlanner::setInputData(InputData const & input_data) {
	refArray = input_data.refArray;
	vehicleState = input_data.vehicleState;
	return ;
}

void  ReversePlanner::getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_)
{
    trajectory_  = trajectory;
    return ;
}


bool  ReversePlanner::implement(double cur_time,const DiscretizedTrajectory prev_trajectory) {

    std::vector<planning_msgs::TrajectoryPoint>().swap(trajectory.points);
	int closestIndex = amathutils::closestPoint(refArray.points,vehicleState.x,vehicleState.y);
	trajectory.points.insert(trajectory.points.end(),refArray.points.begin(),refArray.points.begin()+ closestIndex);
    //trajectory = refArray;
	std::reverse(trajectory.points.begin(),trajectory.points.end());
	TrajectoryPoint planning_start_point;
	planning_start_point.path_point_.set_x (refArray.points[closestIndex].x);
	planning_start_point.path_point_.set_y (refArray.points[closestIndex].y);
	
	trajectory.header.stamp = ros::Time(cur_time);
	callBack(&trajectory,&planning_start_point,nullptr,nullptr,nullptr);

	
	return true;
}

