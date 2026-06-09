#pragma once

#include "reference_line_info.h"

using namespace ugv::planning;

class CollisionChecker {
public:
	CollisionChecker() = default;
	template <typename T,typename V>	
	bool obsCollsionCheck(const T& ad_center,const double &yaw,double halfVehicleWidth,double halfVehicleLength, const std::vector<V> &ploygon);
	bool implement(ReferenceLineInfo* reference_line_info,double &collision_s);
};


template <typename T,typename V>	
bool CollisionChecker::obsCollsionCheck(const T& ad_center,const double &yaw,double halfVehicleWidth,double halfVehicleLength, const std::vector<V> &ploygon){

	T  longitudinal_dir,lateral_dir;
	longitudinal_dir.x = cos(yaw);longitudinal_dir.y = sin(yaw);
	lateral_dir.x = -sin(yaw);lateral_dir.y = cos(yaw);
	
	for (auto &point:ploygon)
	{
	     T obs_vec,obs_point;
		 obs_point.x = point.x();
		 obs_point.y = point.y();
		 obs_vec.x = obs_point.x - ad_center.x;
		 obs_vec.y = obs_point.y - ad_center.y;
		 
		 double disTolong = amathutils::abs_cross(obs_vec,longitudinal_dir);
	     double disTolat  = amathutils::abs_cross(obs_vec,lateral_dir);
		 if ((disTolong < halfVehicleWidth) && (disTolat < halfVehicleLength))
		    return true;
	} 
	return false;
	
}


