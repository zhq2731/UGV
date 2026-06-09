
#include "display.h"




visualization_msgs::Marker DisPlay::deleteMarker(const std::string &ns,int id_)
{
	visualization_msgs::Marker delete_marker;
	delete_marker.header.stamp = ros::Time::now();
	delete_marker.ns = ns;
	delete_marker.id = id_;
	delete_marker.action = visualization_msgs::Marker::DELETE;
	delete_marker.header.frame_id = "map";
	return delete_marker;
}


/*
Pivot point is around the tip of its tail.
Identity orientation points it along the +X axis. 
scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.
*/


visualization_msgs::Marker DisPlay::arrowMarkerMethod2(const DisplayConfig &cfg)

{
	visualization_msgs::Marker marker;
	marker.header.frame_id = cfg.frame_id;

	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation = cfg.quaternion;
	marker.pose.position = cfg.position;

	marker.ns = cfg.ns;
	marker.id = cfg.id;

	marker.scale.x = cfg.scale_x;
	marker.scale.y = cfg.scale_y;
	marker.scale.z = cfg.scale_z;

	marker.color.r = cfg.r;
	marker.color.g = cfg.g;
	marker.color.b = cfg.b;
	marker.color.a = cfg.a;
	return marker;
}	



visualization_msgs::MarkerArray DisPlay::vehicleMarkerArray(const nav_msgs::Odometry &odom,VehicleInfoUtil *vehcileInfo,DisplayConfig &cfg_)
{
	using namespace vehicle_info_util;
	DisplayConfig cfg = cfg_;
	//显示车
	double x_len = vehcileInfo->front_overhang_m+vehcileInfo->wheel_base_m;
	double y_len = vehcileInfo->vehicle_width_m;
    geometry_msgs::Point LocalBasePoint;
	LocalBasePoint.x = x_len*0.5;LocalBasePoint.y = 0.0;
	
	double yaw = amathutils::getPoseYawAngle(odom.pose.pose);
	geometry_msgs::Point GlobalBasePoint = amathutils::localToGlobal(odom.pose.pose.position,yaw,LocalBasePoint);

	cfg.g  =  1.0 ;
	cfg.scale_x = x_len;
	cfg.scale_y = y_len;
	cfg.scale_z = vehcileInfo->vehicle_height_m;	
	cfg.a  = 0.5;
	cfg.quaternion = odom.pose.pose.orientation;
	visualization_msgs::MarkerArray markerArray;
	markerArray.markers.push_back(DisPlay::cubeMarker(GlobalBasePoint,cfg));
   
	//显示车的方向
    cfg.position = GlobalBasePoint;
	cfg.quaternion = odom.pose.pose.orientation;
	cfg.id++;
	cfg.r = cfg.g =  1.0;
	cfg.b = 0.0;
	cfg.scale_x = x_len*0.5;
	cfg.scale_y = y_len*0.1;
	cfg.scale_z = y_len*0.1;
	markerArray.markers.push_back(DisPlay::arrowMarkerMethod2(cfg));

	//车的轮子
    geometry_msgs::Point LocalLeftFront,LocalLeftBack,LocalRightFront,LocalRightBack;
	LocalLeftFront.x = x_len*0.75;LocalLeftFront.y = y_len*0.5;
	LocalLeftBack.x = x_len*0.25;LocalLeftBack.y = y_len*0.5;

	LocalRightFront.x = x_len*0.75;LocalRightFront.y = -y_len*0.5;
	LocalRightBack.x = x_len*0.25;LocalRightBack.y = -y_len*0.5;
	//geometry_msgs::Point GlobalLeftFront = amathutils::localToGlobal(odom.pose.pose.position,yaw,LocalLeftFront);

	std::vector<geometry_msgs::Point> wheelsPoints;
	wheelsPoints.push_back(LocalLeftFront);
	wheelsPoints.push_back(LocalLeftBack);
	wheelsPoints.push_back(LocalRightFront);
	wheelsPoints.push_back(LocalRightBack);
	cfg.position = odom.pose.pose.position;
	cfg.quaternion = odom.pose.pose.orientation;
	cfg.id++;
	cfg.r = cfg.g =  1.0;
	cfg.b = 0.0;
	cfg.scale_x = 0.5;
	cfg.scale_y = 0.5;
	cfg.scale_z = 0.5;
	markerArray.markers.push_back(DisPlay::sphereListMarker(wheelsPoints,cfg));
	
	return markerArray;
}



visualization_msgs::MarkerArray DisPlay::plogonsMarkerArray(std::vector<geometry_msgs::Polygon> &polygons, DisplayConfig &cfg){

    visualization_msgs::MarkerArray  markerArray;

    for (const auto &polygon: polygons) {
	    markerArray.markers.push_back(DisPlay::plogonMarker(polygon,cfg));
		cfg.id++;
    }
	return markerArray;
}

/*
Line strips use the points member of the visualization_msgs/msg/Marker message. It will draw a line between every two consecutive points, so 0-1, 1-2, 2-3, 3-4, 4-5…

Line strips also have some special handling for scale: only scale.x is used and it controls the width of the line segments.

Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.

*/
visualization_msgs::Marker DisPlay::plogonMarker(const geometry_msgs::Polygon &polygon,const DisplayConfig &cfg)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = cfg.frame_id;
	marker.ns = cfg.ns;
	marker.id = cfg.id;
	
	marker.type   = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.orientation = cfg.quaternion;
	marker.pose.position = cfg.position;

	marker.scale.x = cfg.scale_x;
	marker.scale.y = 0.0;
	marker.scale.z = 0.0;
	
	marker.color.r = cfg.r;
	marker.color.g = cfg.g;
	marker.color.b = cfg.b;
	marker.color.a = cfg.a;

	for (const auto &p:polygon.points){
		geometry_msgs::Point point;
		point.x = p.x;point.y = p.y;point.z = p.z;
		marker.points.push_back(point);
	}

	return marker;
}

/*
This marker displays text in a 3D spot in the world.
The text always appears oriented correctly for the RViZ user to see the included text. Uses the text field in the marker.
Only scale.z is used. scale.z specifies the height of an uppercase “A”.
*/
visualization_msgs::Marker DisPlay::stringMarker(const std::string &str,const DisplayConfig &cfg)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = cfg.frame_id;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.ns = cfg.ns;
	marker.id = cfg.id;
	marker.pose.position = cfg.position;

	marker.scale.x = cfg.scale_x;
	marker.scale.y = cfg.scale_y;
	marker.scale.z = cfg.scale_z;
	marker.color.r = cfg.r;
	marker.color.g = cfg.g;
	marker.color.b = cfg.b;
	marker.color.a = cfg.a;
	marker.text = str;
	return marker;
}



