#pragma once

#include <ros/ros.h>
#include "planning_msgs/TrajectoryPoint.h"
#include "planning_msgs/TrajectoryPointArray.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include "localization_msgs/Localization.h"
#include "amathutils_lib/geometry.hpp"
#include "amathutils_lib/amathutils.hpp"
#include "geometry_msgs/Polygon.h"
#include "vehicle_info_util/vehicle_info_util.hpp"

/*
static visualization_msgs::MarkerArray generateVehicleMarkerArray(unsigned int id,nav_msgs::Odometry &odom,vehicle_info_util::VehicleInfoUtil *vehcileInfo,const std::string &ns)
{

	double x_len = vehcileInfo->front_overhang_m+vehcileInfo->wheel_base_m;
	double y_len = vehcileInfo->vehicle_width_m;
	
	std::vector<geometry_msgs::Point> velarray;
	geometry_msgs::Point p1,p2,p3,p4,p5;
	p1.x = 0;p1.y = -y_len * 0.5;p1.z= 0;
	p2.x = 0;p2.y = y_len * 0.5;p2.z= 0;
	p3.x = x_len;p3.y = y_len* 0.5;p3.z= 0;
	p4.x = x_len;p4.y = -y_len * 0.5;p4.z= 0;
	p5.x = 0;p5.y = -y_len * 0.5;p5.z= 0;
	velarray.push_back(p1);velarray.push_back(p2);velarray.push_back(p3);velarray.push_back(p4);velarray.push_back(p5);

	visualization_msgs::MarkerArray markerArray;
	visualization_msgs::Marker line_strip;
	line_strip.header.frame_id = "map";
	//line_strip.header.stamp = ros::Time::now();
	line_strip.ns = ns;
	line_strip.id = id++;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.scale.x = 0.1;
	line_strip.color.g = 1.0;
	line_strip.color.a = 1.0;

	double yaw = amathutils::getPoseYawAngle(odom.pose.pose);

	for (unsigned int j = 0; j < velarray.size(); j++) {
		
		geometry_msgs::Point globalPoint = amathutils::localToGlobal(odom.pose.pose.position,yaw,velarray[j]);
		geometry_msgs::Point p;
		p.x = globalPoint.x;
		p.y = globalPoint.y;
		line_strip.points.push_back(p);
	}
	markerArray.markers.push_back(line_strip);


	visualization_msgs::Marker sphere;
	sphere.header.frame_id = "map";
	sphere.ns = ns;
	sphere.id = id;
	sphere.type = visualization_msgs::Marker::SPHERE;
	sphere.action = visualization_msgs::Marker::ADD;
	sphere.pose.orientation.w = 1.0;
	sphere.scale.x = 1.0;
	sphere.scale.y =  1.0;
	sphere.scale.z =  0.001;

	sphere.color.g = 1.0;
	sphere.color.a = 1.0;

	geometry_msgs::Point spherePos;
	spherePos.x = vehcileInfo->front_overhang_m *0.5 + vehcileInfo->wheel_base_m;	
	spherePos.y = 0.0;
	geometry_msgs::Point globalSherePos = amathutils::localToGlobal(odom.pose.pose.position,yaw,spherePos);
	sphere.pose.position = globalSherePos;

	markerArray.markers.push_back(sphere);

	return markerArray;
}
*/

using namespace vehicle_info_util;

class DisplayConfig
{
public:
    DisplayConfig(){
		quaternion.x = 0;
		quaternion.y = 0;
		quaternion.z = 0;
		quaternion.w = 1;
		position.x = 0;
		position.y = 0;
		position.z = 0;
	}
	std::string frame_id{"map"};
    float r = 0.0; //0~1
	float g = 0.0; //0~1
	float b = 0.0; //0~1
	float a = 1.0; //0~1 越小越透明
	
	float scale_x; //长宽高
	float scale_y; 
	float scale_z;
	std::string ns;  //命名空间	  
	unsigned int id; //
	geometry_msgs::Quaternion  quaternion;  //位姿
	geometry_msgs::Point  position;
};



class  DisPlay
{
public:

	template <class T>
	static visualization_msgs::Marker arrowMarkerMethod1(const std::vector<T> &points,const DisplayConfig &cfg);

	static visualization_msgs::Marker arrowMarkerMethod2(const DisplayConfig &cfg);
	static visualization_msgs::MarkerArray vehicleMarkerArray(const nav_msgs::Odometry &odom,VehicleInfoUtil *vehcileInfo,DisplayConfig &cfg_);

	template <class T>
	static visualization_msgs::Marker pointsMarker(const std::vector<T> &points,const DisplayConfig &cfg);

	template <class T>
	static visualization_msgs::Marker lineListMarker(const std::vector<T> &points,const DisplayConfig &cfg) ;

	template <class T>
	static visualization_msgs::Marker lineMarker(const std::vector<T> &points,const DisplayConfig &cfg) ;

	template <class T>
	static visualization_msgs::Marker cubeMarker(const T &point,const DisplayConfig &cfg);

	template <class T>
	static visualization_msgs::Marker cubesListMarker(const std::vector<T> &points,const DisplayConfig &cfg);

	template <class T>
	static visualization_msgs::Marker sphereMarker(const T &point,const DisplayConfig &cfg);

	template <class T>
	static visualization_msgs::Marker sphereListMarker(const std::vector<T> &points,const DisplayConfig &cfg)	;
	
	template <class T>
	static visualization_msgs::Marker cylinderMarker(const T &point,const DisplayConfig &cfg);

	static visualization_msgs::MarkerArray plogonsMarkerArray(std::vector<geometry_msgs::Polygon> &polygons, DisplayConfig &cfg);

	static visualization_msgs::Marker plogonMarker(const geometry_msgs::Polygon &polygon,const DisplayConfig &cfg);

	static visualization_msgs::Marker stringMarker(const std::string &str,const DisplayConfig &cfg);		

	static visualization_msgs::Marker deleteMarker(const std::string &ns,int id_);

};

/*
The point at index 0 is assumed to be the start point, and the point at index 1 is assumed to be the end.

scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length.

*/


template <class T>
visualization_msgs::Marker DisPlay::arrowMarkerMethod1(const std::vector<T> &points,const DisplayConfig &cfg)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = cfg.frame_id;

	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.orientation.w = 1.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;

	marker.ns = cfg.ns;
	marker.id = cfg.id;

	marker.scale.x = cfg.scale_x;
	marker.scale.y = cfg.scale_y;
	marker.scale.z = cfg.scale_z;

	marker.color.r = cfg.r;
	marker.color.g = cfg.g;
	marker.color.b = cfg.b;
	marker.color.a = cfg.a;

	
	for (auto &point:points){
		geometry_msgs::Point p ;
		p.x = point.x;
		p.y = point.y;
		marker.points.push_back(p);
		if (2 == marker.points.size())
			break;
	}

	return marker;
}	


/*
Uses the points member of the visualization_msgs/msg/Marker message.

Points have some special handling for scale: scale.x is point width, scale.y is point height

Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.

*/

template <class T>
visualization_msgs::Marker	DisPlay::pointsMarker(const std::vector<T> &points,const DisplayConfig &cfg)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = cfg.frame_id;
	marker.ns = cfg.ns;
	marker.id = cfg.id;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation = cfg.quaternion;
	marker.pose.position = cfg.position;

	marker.scale.x = cfg.scale_x;
	marker.scale.y = cfg.scale_y;
	marker.scale.z = 0.0;
	marker.color.r = cfg.r;
	marker.color.g = cfg.g;
	marker.color.b = cfg.b;
	marker.color.a = cfg.a;

	for (auto &point:points){
		geometry_msgs::Point p ;
		p.x = point.x;
		p.y = point.y;
		marker.points.push_back(p);
	}
	return marker;
} 

/*
Line lists use the points member of the visualization_msgs/msg/Marker message. It will draw a line between each pair of points, so 0-1, 2-3, 4-5, …

Line lists also have some special handling for scale: only scale.x is used and it controls the width of the line segments.

Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.

*/

template <class T>
visualization_msgs::Marker DisPlay::lineListMarker(const std::vector<T> &points,const DisplayConfig &cfg) 
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = cfg.frame_id;
	marker.ns = cfg.ns;
	marker.id = cfg.id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
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

	for (auto &point:points){
		geometry_msgs::Point p ;
		p.x = point.x;
		p.y = point.y;
		marker.points.push_back(p);
	}
	return marker;
}


/*
Line strips use the points member of the visualization_msgs/msg/Marker message. It will draw a line between every two consecutive points, so 0-1, 1-2, 2-3, 3-4, 4-5…

Line strips also have some special handling for scale: only scale.x is used and it controls the width of the line segments.

Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.

*/


template <class T>
visualization_msgs::Marker DisPlay::lineMarker(const std::vector<T> &points,const DisplayConfig &cfg) 
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = cfg.frame_id;
	marker.ns = cfg.ns;
	marker.id = cfg.id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
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

	for (auto &point:points){
		geometry_msgs::Point p ;
		p.x = point.x;
		p.y = point.y;
		marker.points.push_back(p);
	}
	return marker;
}

    
/*
Pivot point is at the center of the cube.
*/
template <class T>
visualization_msgs::Marker DisPlay::cubeMarker(const T &point,const DisplayConfig &cfg)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = cfg.frame_id;

	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation = cfg.quaternion;

	marker.ns = cfg.ns;
	marker.id = cfg.id;

	marker.scale.x = cfg.scale_x;
	marker.scale.y = cfg.scale_y;
	marker.scale.z = cfg.scale_z;

	marker.color.r = cfg.r;
	marker.color.g = cfg.g;
	marker.color.b = cfg.b;
	marker.color.a = cfg.a;

	marker.pose.position.x = point.x;
	marker.pose.position.y = point.y;

	return marker;
}    
	 
/*
A cube list is a list of cubes with all the same properties except their positions.
Using this object type instead of a visualization_msgs/msg/MarkerArray allows RViz to batch-up rendering, 
which causes them to render much faster. The caveat is that they all must have the same scale.
The points member of the visualization_msgs/msg/Marker message is used for the position of each cube.

*/
	 
 template <class T>
visualization_msgs::Marker	DisPlay::cubesListMarker(const std::vector<T> &points,const DisplayConfig &cfg)
 {
	visualization_msgs::Marker marker;
	marker.header.frame_id = cfg.frame_id;
	marker.ns = cfg.ns;
	marker.id = cfg.id;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation = cfg.quaternion;
	marker.pose.position = cfg.position;
	marker.scale.x = cfg.scale_x;
	marker.scale.y = cfg.scale_y;
	marker.scale.z = cfg.scale_z;

	marker.color.r = cfg.r;
	marker.color.g = cfg.g;
	marker.color.b = cfg.b;
	marker.color.a = cfg.a;

	for (auto &point:points){
		geometry_msgs::Point p ;
		p.x = point.x;
		p.y = point.y;
		marker.points.push_back(p);
	}
	return marker;
 } 

/*
Pivot point is at the center of the sphere.

scale.x is diameter in x direction, scale.y in y direction, scale.z in z direction. 

By setting these to different values you get an ellipsoid instead of a sphere.

*/

 template <class T>
visualization_msgs::Marker	 DisPlay::sphereMarker(const T &point,const DisplayConfig &cfg)
 {
	 visualization_msgs::Marker marker;
	 marker.header.frame_id = cfg.frame_id;
 
	 marker.type = visualization_msgs::Marker::SPHERE;
	 marker.action = visualization_msgs::Marker::ADD;
	 marker.pose.orientation = cfg.quaternion;
 
	 marker.ns = cfg.ns;
	 marker.id = cfg.id;
 
	 marker.scale.x = cfg.scale_x;
	 marker.scale.y = cfg.scale_y;
	 marker.scale.z = cfg.scale_z;
 
	 marker.color.r = cfg.r;
	 marker.color.g = cfg.g;
	 marker.color.b = cfg.b;
	 marker.color.a = cfg.a;
 
	 marker.pose.position.x = point.x;
	 marker.pose.position.y = point.y;
 
	 return marker;
 }	  
	 
/*
A sphere list is a list of spheres with all the same properties except their positions. Using this object type instead of a visualization_msgs/msg/MarkerArray allows RViz to batch-up rendering, which causes them to render much faster. The caveat is that they all must have the same scale.

The points member of the visualization_msgs/msg/Marker message is used for the position of each sphere.

Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.

*/

template <class T>
visualization_msgs::Marker  DisPlay::sphereListMarker(const std::vector<T> &points,const DisplayConfig &cfg)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = cfg.frame_id;
	marker.ns = cfg.ns;
	marker.id = cfg.id;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation = cfg.quaternion;
	marker.pose.position = cfg.position;
	marker.scale.x = cfg.scale_x;
	marker.scale.y = cfg.scale_y;
	marker.scale.z = cfg.scale_z;

	marker.color.r = cfg.r;
	marker.color.g = cfg.g;
	marker.color.b = cfg.b;
	marker.color.a = cfg.a;

	for (auto &point:points){
		geometry_msgs::Point p ;
		p.x = point.x;
		p.y = point.y;
		marker.points.push_back(p);
	}
	return marker;
 } 


/*
Pivot point is at the center of the cylinder.

scale.x is diameter in x direction, scale.y in y direction, 
by setting these to different values you get an ellipse instead of a circle. Use scale.z to specify the height.

*/

template <class T>
visualization_msgs::Marker  DisPlay::cylinderMarker(const T &point,const DisplayConfig &cfg)
{
	 visualization_msgs::Marker marker;
	 marker.header.frame_id = cfg.frame_id;

	 marker.type = visualization_msgs::Marker::CYLINDER;
	 marker.action = visualization_msgs::Marker::ADD;
	 marker.pose.orientation = cfg.quaternion;

	 marker.ns = cfg.ns;
	 marker.id = cfg.id;

	 marker.scale.x = cfg.scale_x;
	 marker.scale.y = cfg.scale_y;
	 marker.scale.z = cfg.scale_z;

	 marker.color.r = cfg.r;
	 marker.color.g = cfg.g;
	 marker.color.b = cfg.b;
	 marker.color.a = cfg.a;

	 marker.pose.position.x = point.x;
	 marker.pose.position.y = point.y;

	 return marker;
} 


