
#pragma once

#include "planning_msgs/TrajectoryPointArray.h"
#include "common/obstacle.h"
#include "common/vehicle_state.h"

#include "nav_msgs/OccupancyGrid.h"

using namespace ugv::planning;
using namespace ugv::common::math;
using namespace ugv::common;

class InputData
{
public:
	std::vector< const Obstacle*> obsList;
	planning_msgs::TrajectoryPointArray refArray;
    VehicleState vehicleState;
    VehicleState goalState;
	nav_msgs::OccupancyGrid occGrid;
	
};

