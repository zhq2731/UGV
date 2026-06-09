
#include <iostream>
#include "leavingVelocityPlanner.h"


//vt = v0 + at

bool LeavingVelocityPlanner::decelerateProfile(planning_msgs::TrajectoryPointArray &path,double startSpeed){

	double slow_speed_ = 0.5,acc_max_ = 0.5,acc_min_ = 0.5;

	double decel_distance = calcDistance(startSpeed, 0, -acc_max_);

	//double brake_distance = calcDistance(slow_speed_, 0, -acc_min_); 

	double vi = startSpeed;
	double vf;
	double acc = -acc_max_;
	double relative_time = 0;
	double sum_decel_distance = 0;
	int index  = -1;
	for (int i = 0; i < path.points.size(); i++) {
	  double dist = amathutils::distance2D(path.points[i], path.points[i + 1]);
	  sum_decel_distance += dist;
	  vf = calcFinalSpeed(vi, -acc_max_, dist);
      
	  if (vf < slow_speed_) {
	    vf = slow_speed_;
		acc = 0.0;
	  }
	  
      path.points[i].relative_time = relative_time;
	  path.points[i].v = vi;
      path.points[i].a = 0.0;
	  if (fabs(acc) > 1e-3)
	      relative_time += (vf - vi)/acc;
	  else
	      relative_time += dist / vf;

	  vi = vf;
	  
	  if (sum_decel_distance > decel_distance){
	  	  index = i;
	  	  break;
	  }
	}

    
    for (int i = index+1;i < path.points.size(); i++){
		path.points[i].relative_time = relative_time;
		path.points[i].v = 0.0;
		path.points[i].a = 0.0;
    }
	return true;
}


// Using d = (v_f^2 - v_i^2) / (2 * a), compute the distance
// required for a given acceleration/deceleration.
double LeavingVelocityPlanner::calcDistance(double v_i, double v_f, double a) {
  /**
  Computes the distance given an initial and final speed, with a constant
  acceleration.

  args:
      v_i: initial speed (m/s)
      v_f: final speed (m/s)
      a: acceleration (m/s^2)
  returns:
      d: the final distance (m)
  */
  return (v_f * v_f - v_i * v_i) / (2 * a);
}

// Using v_f = sqrt(v_i^2 + 2ad), compute the final speed for a given
// acceleration across a given distance, with initial speed v_i.
// Make sure to check the discriminant of the radical. If it is negative,
// return zero as the final speed.
double LeavingVelocityPlanner::calcFinalSpeed(double v_i, double a, double d) {
  /**
  Computes the final speed given an initial speed, distance travelled,
  and a constant acceleration.

  args:
      v_i: initial speed (m/s)
      a: acceleration (m/s^2)
      d: distance to be travelled (m)
  returns:
      v_f: the final speed (m/s)
  */
  double temp = v_i * v_i + 2 * d * a;
  if (temp < 0)
    return 0.0000001;
  else
    return sqrt(temp);
}

