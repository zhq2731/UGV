#include "common/points_convert.h"

using namespace ugv::planning;

void  trajMsg2RefPoints(planning_msgs::TrajectoryPointArray &msgArray,std::vector<ReferencePoint> &refPoints) 
{
	std::vector< ReferencePoint >().swap(refPoints);
	for (auto &point :msgArray.points){
		ReferencePoint p;
		p.set_x (point.x);
		p.set_y (point.y);
		p.set_heading (point.theta);
		p.set_kappa (point.kappa);
		p.set_dkappa (point.dkappa);
		refPoints.push_back(p);
	}	 
	return ;
}


void trajMsg2DiscretTraj(planning_msgs::TrajectoryPointArray &trajectory,DiscretizedTrajectory  &discretTraj)
{
	std::vector<TrajectoryPoint>().swap(discretTraj);
	for (auto &p :trajectory.points)
	{
		TrajectoryPoint point;
		point.set_v(p.v);
		point.set_a(p.a);
		point.set_relative_time(p.relative_time);
		point.mutable_path_point()->set_x(p.x);
		point.mutable_path_point()->set_y(p.y);
		point.mutable_path_point()->set_s(p.s);
		point.mutable_path_point()->set_theta(p.theta);
		point.mutable_path_point()->set_kappa(p.kappa);
		point.mutable_path_point()->set_dkappa(p.dkappa);
		discretTraj.push_back(point);
	}
	discretTraj.header_time_ = trajectory.header.stamp.toSec();
	return ;
}


void discretPath2TrajMsg(const DiscretizedPath &discretPath,planning_msgs::TrajectoryPointArray &outTrajectory) 
{
	std::vector<planning_msgs::TrajectoryPoint>().swap(outTrajectory.points);
	for (const auto&p : discretPath){
	    planning_msgs::TrajectoryPoint point;
	    point.x = p.x();
		point.y = p.y();
		point.z = p.z();
		point.theta = p.theta();
		point.s = p.s();
		point.kappa = p.kappa();
		point.dkappa = p.dkappa();
		//point.v = current_velocity;
		outTrajectory.points.push_back(point);
	}
	return ;
}



