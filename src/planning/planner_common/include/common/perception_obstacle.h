// syntax = "proto2";
#pragma once

#include <vector>

using namespace std;
namespace ugv{
namespace perception{

// import "modules/common/proto/error_code.proto";
// import "modules/common/proto/geometry.proto";
// import "modules/common/proto/header.proto";
// import "modules/map/proto/map_lane.proto";

struct BBox2D {
   double xmin = 1;  // in pixels.
   double ymin = 2;  // in pixels.
   double xmax = 3;  // in pixels.
   double ymax = 4;  // in pixels.
};
struct Point3D {
  double x = 1;// [default = nan];
  double y = 2;// [default = nan];
  double z = 3;// [default = nan];

  
};

// message LightStatus {
//    double brake_visible = 1;
//    double brake_switch_on = 2;
//    double left_turn_visible = 3;
//    double left_turn_switch_on = 4;
//    double right_turn_visible = 5;
//    double right_turn_switch_on = 6;
// }

// message SensorMeasurement {
//    string sensor_id = 1;
//    int32 id = 2;

//    common.Point3D position = 3;
//    double theta = 4;
//    double length = 5;
//    double width = 6;
//    double height = 7;

//    common.Point3D velocity = 8;

//    PerceptionObstacle.Type type = 9;
//    PerceptionObstacle.SubType sub_type = 10;
//    double timestamp = 11;
//    BBox2D box = 12;  // only for camera measurements
// }

struct PerceptionObstacle {
   int id = 1;  // obstacle ID.
  // obstacle position in the world coordinate system.
   Point3D position = {1,2,3};
   double theta = 3;  // heading in the world coordinate system.
   Point3D velocity = {0,0,0}; // obstacle velocity.
  // Size of obstacle bounding box.
   double length = 5;  // obstacle length.
   double width = 6;   // obstacle width.
   double height = 7;  // obstacle height.
   std::vector<Point3D> polygon_point;  // obstacle corner points.
  // duration of an obstacle since detection in s.
   double tracking_time = 9;

   double timestamp = 11;  // GPS time in seconds.
   vector <double> point_cloud;

   double confidence = 13;// [deprecated = true];
  vector<Point3D> drops;// = 15 [deprecated = true];
  Point3D acceleration;  // obstacle acceleration

   Point3D anchor_point;
   BBox2D bbox2d;
   double height_above_ground = 21;// [default = nan];
   unsigned char   type;

};

} //namespace perception
} //namespace ugv
