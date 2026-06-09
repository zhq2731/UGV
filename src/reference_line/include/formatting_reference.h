
#include <iostream>
#include <string>

#include "planning_msgs/TrajectoryPoint.h"
#include "planning_msgs/TrajectoryPointArray.h"

constexpr double kMathEpsilon = 1e-10;

class FormattingRefLine
{
public:
	FormattingRefLine(planning_msgs::TrajectoryPointArray &refLine);
	FormattingRefLine();
	bool setRefLine(planning_msgs::TrajectoryPointArray &refLine);
    bool implement(double latOffset,planning_msgs::TrajectoryPointArray &outRefLine);  
private:
	bool SLToXY(const double s,const double l,
							   double &x_,double &y_) const ;

    planning_msgs::TrajectoryPoint	MatchToPath(const planning_msgs::TrajectoryPointArray &reference_line,
									   const double s_) const;
	 
	planning_msgs::TrajectoryPoint InterpolateUsingLinearApproximation(const planning_msgs::TrajectoryPoint &p0,
												  const planning_msgs::TrajectoryPoint &p1,
												  const double s) const;
	
	double slerp(const double a0, const double t0, const double a1, const double t1,
				 const double t) const;

	double NormalizeAngle(const double angle) const;

	void CalculateCartesianPoint(const double rtheta,const double rx,const double ry,
															const double l,double &x_,double &y_) const;

	planning_msgs::TrajectoryPointArray refLine_;
	
};

