#pragma once

#include "common/inputData.h"
#include "common/path_boundary.h"
#include "trajectory/discretized_trajectory.h"


using namespace ugv::planning;

typedef void (*displayCallback)(const planning_msgs::TrajectoryPointArray *planned_trajectory,\
	const TrajectoryPoint *planning_start_point   ,const PathBoundary *lane_boundry ,const PathBoundary *planning_boundry  ,const ReferenceLine *reference_line );


class BasePlanner
{
public:
	BasePlanner(displayCallback callBack_){this->callBack = callBack_;}
    virtual bool implement(double cur_time,const DiscretizedTrajectory prev_trajectory) = 0;
    virtual void setInputData(InputData const & input_data) = 0;
    virtual void getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_) = 0;
    virtual Obstacle*  blockedObs() {return nullptr;}
    virtual ~BasePlanner() = default;
	displayCallback callBack;
};

