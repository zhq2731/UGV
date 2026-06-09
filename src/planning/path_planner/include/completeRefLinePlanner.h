#pragma once

#include "common/inputData.h"
#include "common/basePlanner.h"
#include "collisionChecker.h"
#include "common/points_convert.h"
#include "vehicle_info_util/vehicle_info_util.hpp"

using namespace ugv::planning;
class  CompleteRefLinePlanner : public BasePlanner
{
public:
	
	CompleteRefLinePlanner(displayCallback callBack_);
	void  setInputData(InputData const & input_data) override;
	bool  implement(double cur_time,const DiscretizedTrajectory prev_trajectory) override;
    void  getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_) override;
    Obstacle*  blockedObs()override {return blockedObs_;}
private:
	std::vector< const Obstacle*> obsList;
	planning_msgs::TrajectoryPointArray refArray;
	std::vector<ReferencePoint> referencePoints;
    VehicleState vehicleState;
	planning_msgs::TrajectoryPointArray trajectory;
	Obstacle*  blockedObs_;
};

