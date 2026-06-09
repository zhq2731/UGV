#pragma once
#include "common/inputData.h"
#include "common/basePlanner.h"
#include "math/vec2d.h"
#include "common/planning_config.h"
#include "common/frame.h"
#include "common/pnc_point.h"
#include "common/obstacle.h"
#include "trajectory/reference_point.h"
#include "trajectory/discretized_path.h"
#include "reference_line_info.h"
#include "common/planning_config.h"
#include "decider/path_bounds_decider.h"
#include "decider/piecewise_jerk_path_optimizer.h"
#include "trajectory/trajectory_stitcher.h"
#include "common/points_convert.h"

using namespace ugv::planning;


class  RefLinePlanner : public BasePlanner
{
public:
	RefLinePlanner(displayCallback callBack_);
	void  setInputData(InputData const & input_data) override;
	bool  implement(double cur_time,const DiscretizedTrajectory prev_trajectory) override;
    void  getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_) override;	
    virtual Obstacle*  blockedObs()override{return blockedObs_;}
	void  updateParam(std::string area);
private:
	std::vector< const Obstacle*> obsList;
	planning_msgs::TrajectoryPointArray refArray;
	std::vector<ReferencePoint> referencePoints;
    VehicleState vehicleState;
	planning_msgs::TrajectoryPointArray trajectory;
	Obstacle*  blockedObs_;
};

