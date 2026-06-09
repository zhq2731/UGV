#pragma once
#include "common/inputData.h"
#include "common/basePlanner.h"
using namespace ugv::planning;

class  ReversePlanner : public BasePlanner
{
public:
	
	ReversePlanner(displayCallback callBack_);
	void setInputData(InputData const & input_data) override;
	bool  implement(double cur_time,const DiscretizedTrajectory prev_trajectory) override;
    void  getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_) override;
private:
	planning_msgs::TrajectoryPointArray refArray;
    VehicleState vehicleState;
	planning_msgs::TrajectoryPointArray trajectory;
};

