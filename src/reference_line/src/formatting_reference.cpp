

#include "formatting_reference.h"

FormattingRefLine::FormattingRefLine(planning_msgs::TrajectoryPointArray &refLine)
{
    refLine_ = refLine;
}
FormattingRefLine::FormattingRefLine()
{
}

bool FormattingRefLine::setRefLine(planning_msgs::TrajectoryPointArray &refLine)
{
    refLine_ = refLine;
    return true;
}

bool FormattingRefLine::implement(double latOffset,planning_msgs::TrajectoryPointArray &outRefLine)
{
    std::vector<planning_msgs::TrajectoryPoint>().swap(outRefLine.points);
	  for (std::size_t i = 0; i < refLine_.points.size(); ++i) {
	      double x ,y;
		    planning_msgs::TrajectoryPoint trajPoint;
		    SLToXY(refLine_.points[i].s,latOffset,trajPoint.x,trajPoint.y);
	      outRefLine.points.push_back(trajPoint);
	   }
    return true;
}


//need x y theta s 
bool FormattingRefLine::SLToXY(const double s,const double l,
                           double &x_,double &y_) const {

	planning_msgs::TrajectoryPoint matched_ref_point = MatchToPath(refLine_, s);
	const double rs = matched_ref_point.s;
	const double rx = matched_ref_point.x;
	const double ry = matched_ref_point.y;
	const double rtheta = matched_ref_point.theta;
	const double rkappa = matched_ref_point.kappa;
	const double rdkappa = matched_ref_point.dkappa;	
	FormattingRefLine::CalculateCartesianPoint(rtheta,rx,ry,l,x_,y_);
	return true;
}

						   
planning_msgs::TrajectoryPoint  FormattingRefLine::MatchToPath(const planning_msgs::TrajectoryPointArray &reference_line,
                                   const double s_) const{
  auto comp = [](const planning_msgs::TrajectoryPoint& point, const double s) {
    return point.s < s;
  };

  auto it_lower =
      std::lower_bound(reference_line.points.begin(), reference_line.points.end(), s_, comp);

  //lianbin  
  if (it_lower == reference_line.points.begin()) {
    return reference_line.points.front();
  } 

  if (it_lower == reference_line.points.end() || (it_lower+1) == reference_line.points.end()) 
      return reference_line.points.back();
  
  // interpolate between it_lower - 1 and it_lower
  // return interpolate(*(it_lower - 1), *it_lower, s);
  return InterpolateUsingLinearApproximation(*(it_lower), *(it_lower+1), s_);
}



planning_msgs::TrajectoryPoint FormattingRefLine::InterpolateUsingLinearApproximation(const planning_msgs::TrajectoryPoint &p0,
                                              const planning_msgs::TrajectoryPoint &p1,
                                              const double s) const  {
  double s0 = p0.s;
  double s1 = p1.s;

  planning_msgs::TrajectoryPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x + weight * p1.x;
  double y = (1 - weight) * p0.y + weight * p1.y;
  double theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
  //double ddkappa = (1 - weight) * p0.ddkappa + weight * p1.ddkappa;
  path_point.x = (x);
  path_point.y = (y);
  path_point.theta = (theta);
  path_point.kappa = (kappa);
  path_point.dkappa = (dkappa);
  path_point.s = (s);
  return path_point;
}

double FormattingRefLine::slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) const{
  if (std::abs(t1 - t0) <= kMathEpsilon) {
     std::cout  << "input time difference is too small";
     return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}


double FormattingRefLine::NormalizeAngle(const double angle)const {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

void FormattingRefLine::CalculateCartesianPoint(const double rtheta,
                                                        const double rx,const double ry,
                                                        const double l,double &x_,double &y_) const {
                                                     
  const double x = rx - l * std::sin(rtheta);
  const double y = ry + l * std::cos(rtheta);
  x_ = x;
  y_ = y;
}


