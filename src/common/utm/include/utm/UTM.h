#pragma once

#include "geometry_msgs/Point.h"

namespace projection {
class UtmProjector  {
 public:
  //origin:x = lat ,y = lon z = h
   UtmProjector(geometry_msgs::Point origin, const bool useOffset = true, const bool throwInPaddingArea=false);
   UtmProjector() = default;
  geometry_msgs::Point forward(const geometry_msgs::Point& gps) const ;

  geometry_msgs::Point reverse(const geometry_msgs::Point& utm) const ;

 private:
  int zone_{};
  bool isInNorthernHemisphere_{true}, useOffset_{}, throwInPaddingArea_{};
  double xOffset_{}, yOffset_{};
  geometry_msgs::Point origin_;
};
}  // namespace projection
