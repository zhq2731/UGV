
#include "collisionChecker.h"

using namespace ugv::planning;



bool CollisionChecker::implement(ReferenceLineInfo* reference_line_info,double &collision_s) {
      
	vehicle_info_util::VehicleInfoUtil *vehicle_util = vehicle_info_util::VehicleInfoUtil::get_instance();
	
    double  halfVehicleLength = (vehicle_util->vehicle_length_m + vehicle_util->long_expansion_distance_m*2.0)*0.5;
    double  halfVehicleWidth  = (vehicle_util->vehicle_width_m  + vehicle_util->lat_expansion_distance_m*2.0)*0.5;
    double  rearCenter_to_geometricCenter_dis = (halfVehicleLength - vehicle_util->long_expansion_distance_m - vehicle_util->rear_overhang_m);
    
    std::vector<PathPoint> path = reference_line_info->reference_line().getDiscretizedPathPoint();
	for (int j = 0; j < path.size(); j++) {

		geometry_msgs::Point center_point;//车辆的几何中心

		double yaw = path[j].theta();
		center_point.x =  path[j].x() + rearCenter_to_geometricCenter_dis* cos(yaw);
		center_point.y =  path[j].y() + rearCenter_to_geometricCenter_dis* sin(yaw);

		auto indexed_obstacles = reference_line_info->path_decision()->obstacles();
		
		//const auto obstacle_sl = obstacle->PerceptionSLBoundary();
		for (const Obstacle* obstacle : indexed_obstacles.Items()) {
			
		    if (obstacle->IsIgnore())
				continue;
			
			if (obsCollsionCheck(center_point,yaw,halfVehicleWidth,halfVehicleLength,obstacle->PerceptionPolygon().points())){
			    collision_s = path[j].s();
				reference_line_info->SetBlockingObstacle(obstacle->Id());
				return true;
			}
		}
		
   }
	
   return false;
}


