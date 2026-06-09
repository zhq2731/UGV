#include "UTM.h"

#include <GeographicLib/UTMUPS.hpp>
#include <iostream>
namespace projection {


UtmProjector::UtmProjector(geometry_msgs::Point origin, const bool useOffset, const bool throwInPaddingArea)
    : useOffset_{useOffset}, throwInPaddingArea_{throwInPaddingArea},origin_(origin) {
  double x = 0;
  double y = 0;
  GeographicLib::UTMUPS::Forward(origin_.x, origin_.y, zone_,
                                 isInNorthernHemisphere_, x, y);
  if (useOffset_) {
    xOffset_ = x;
    yOffset_ = y;
  }
}

geometry_msgs::Point UtmProjector::forward(const geometry_msgs::Point& gps) const {
  geometry_msgs::Point utm; 
  utm.x = 0.0;
  utm.y = 0.0;
  utm.z = gps.z;
  int zone{};
  bool northp{};
  try {
    GeographicLib::UTMUPS::Forward(gps.x, gps.y, zone, northp, utm.x, utm.y);
  } catch (GeographicLib::GeographicErr& e) {
     std::cout <<"forward error "<<std::endl;
  }

  if (zone != zone_ || northp != isInNorthernHemisphere_) {
    if (throwInPaddingArea_) {
        std::cout <<"throwInPaddingArea_ error "<<std::endl;
    }
    // try to transfer to the desired zone
    double xAfterTransfer = 0;
    double yAfterTransfer = 0;
    int zoneAfterTransfer = 0;
    try {
      GeographicLib::UTMUPS::Transfer(zone, northp, utm.x, utm.y, zone_, isInNorthernHemisphere_, xAfterTransfer,
                                      yAfterTransfer, zoneAfterTransfer);
    } catch (GeographicLib::GeographicErr& e) {
          std::cout <<"Transfer error "<<std::endl;
    }

    if (zoneAfterTransfer != zone_) {
        std::cout <<"zoneAfterTransfer error "<<std::endl;
    }
    utm.x = xAfterTransfer;
    utm.y = yAfterTransfer;
  }

  if (useOffset_) {
    utm.x -= xOffset_;
    utm.y -= yOffset_;
  }

  return utm;
}


geometry_msgs::Point UtmProjector::reverse(const geometry_msgs::Point& utm) const {
  geometry_msgs::Point gps;
  gps.x = 0.0;gps.y = 0.0;gps.z = utm.z;
  try {
    GeographicLib::UTMUPS::Reverse(zone_, isInNorthernHemisphere_, useOffset_ ? utm.x + xOffset_ : utm.x,
                                   useOffset_ ? utm.y + yOffset_ : utm.y, gps.x, gps.y);
  } catch (GeographicLib::GeographicErr& e) {
     std::cout <<"GeographicErr error "<<std::endl;
  }

  if (throwInPaddingArea_) {
    // for zone compliance testing:
      forward(gps);
  }
  return gps;
}

}  // namespace projection
