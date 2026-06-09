#pragma once

#include "planning_msgs/TrajectoryPointArray.h"
#include "amathutils_lib/amathutils.hpp"
#include "amathutils_lib/geometry.hpp"

class LeavingVelocityPlanner
{
public:
	LeavingVelocityPlanner(){};
	bool   decelerateProfile( planning_msgs::TrajectoryPointArray &path,double startSpeed);
	double calcFinalSpeed(double v_i, double a, double d);
	double calcDistance(double v_i, double a, double d);
};

