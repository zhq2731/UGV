#include "trajectory/reference_point.h"
#include "planning_msgs/TrajectoryPointArray.h"
#include "trajectory/discretized_path.h"
#include "trajectory/discretized_trajectory.h"

#include "geometry_msgs/Point.h"


using namespace ugv::planning;

void  trajMsg2RefPoints(planning_msgs::TrajectoryPointArray &msgArray,std::vector<ReferencePoint> &refPoints) ;

void  trajMsg2DiscretTraj(planning_msgs::TrajectoryPointArray &trajectory,DiscretizedTrajectory  &discretTraj);

void  discretPath2TrajMsg(const DiscretizedPath &discretPath,planning_msgs::TrajectoryPointArray &outTrajectory);



