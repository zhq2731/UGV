#pragma once

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct GPSPoint {
	double lat{0.}; 
	double lon{0.}; 
	double ele{0.};
};

struct UtmPoint{
	double x{0.};
	double y{0.};
	double z{0.};
};

 struct ResultPoint{
	 UtmPoint utmPoint;
	 bool isCrossing = false;
 };

 struct VehiclePositionInfo{
	double initial_orient_x;
	double initial_orient_y;
	double orient_x;
	double orient_y;
	UtmPoint utmPoint;
	double speed;
 };

 
/*
 template <class T>
 static visualization_msgs::Marker	generateCubesMarker(const unsigned int &id,const std::vector<T> &points,float r,float g,float b,double scale,const std::string &ns)
 {
	   visualization_msgs::Marker marker;
	   marker.header.frame_id = "map";
	   marker.ns = ns;
	   marker.id = id;
	   marker.type = visualization_msgs::Marker::CUBE_LIST;
	   marker.action = visualization_msgs::Marker::ADD;
	   marker.pose.orientation.w = 1.0;

	   marker.scale.x = scale;
	   marker.scale.y = scale;
	   marker.scale.z = scale;

	   marker.color.r = r;
	   marker.color.g = g;
	   marker.color.b = b;
	   marker.color.a = 1.0;
	   
	   for (auto &point:points){
		   geometry_msgs::Point p ;
		   p.x = point.x;
		   p.y = point.y;
		   marker.points.push_back(p);
	   }
	   return marker;
} 

  template <class T>
  static visualization_msgs::Marker  generateSpheresMarker(const unsigned int &id,const std::vector<T> &points,float r,float g,float b,double a,double scale,const std::string &ns)
  {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.ns = ns;
		marker.id = id;
		marker.type = visualization_msgs::Marker::SPHERE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = scale;
		marker.scale.y = scale;
		marker.scale.z = scale;
 
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = a;

		for (auto &point:points){
			geometry_msgs::Point p ;
			p.x = point.x;
			p.y = point.y;
			marker.points.push_back(p);
		}
		return marker;
 } 


   template <class T>
   static visualization_msgs::Marker  generateCYLINDERMarker(const unsigned int &id,const T &point,float r,float g,float b,double a,double scale,const std::string &ns)
   {
		 visualization_msgs::Marker marker;
		 marker.header.frame_id = "map";
		 marker.ns = ns;
		 marker.id = id;
		 marker.type = visualization_msgs::Marker::CYLINDER;
		 marker.action = visualization_msgs::Marker::ADD;
		 marker.pose.orientation.w = 1.0;
		 marker.scale.x = 4;
		 marker.scale.y = 4;
		 marker.scale.z = 5;
  
		 marker.color.r = r;
		 marker.color.g = g;
		 marker.color.b = b;
		 marker.color.a = a;
  
		 marker.pose.position.x = point.x;
		 marker.pose.position.y = point.y;

		 return marker;
  } 


  

template <class T>
static visualization_msgs::Marker generateLineMarker(unsigned id,std::vector<T> &trajectory,float r,float g,float b,float a,double scale,std::string ns,bool delete_) 
{
	visualization_msgs::Marker line_strip;
	line_strip.header.frame_id = "map";
	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = ns;
	line_strip.id = id;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;

	line_strip.scale.x = scale; 

	line_strip.color.r = r;
	line_strip.color.g = g;
	line_strip.color.b = b;
	line_strip.color.a = a;


	for (unsigned int j = 0; j < trajectory.size(); j++) {

		geometry_msgs::Point p;
		p.x = trajectory[j].x;
		p.y = trajectory[j].y;
		line_strip.points.push_back(p);
	}
	return line_strip;
}

*/
