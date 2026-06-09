#ifndef STATE_H
#define STATE_H

//#include "amathutils_lib/geometry.hpp"
#include "amathutils_lib/geometry_point.hpp"


namespace planning {

struct State {
  geometry_point::Point2D position;
  double yaw;
  double vel;
  double acc;

  State() = default;

  State(double x, double y) {
    this->position = geometry_point::Point2D(x, y);
    this->yaw = 0;
    this->vel = 0;
    this->acc = 0;
  }

  State(double x, double y, double yaw) {
    this->position = geometry_point::Point2D(x, y);
    this->yaw = yaw;
    this->vel = 0;
    this->acc = 0;
  }

  State(double x, double y, double yaw, double vel) {
    this->position = geometry_point::Point2D(x, y);
    this->yaw = yaw;
    this->vel = vel;
    this->acc = 0;
  }

  State(double x, double y, double yaw, double vel, double acc) {
    this->position = geometry_point::Point2D(x, y);
    this->yaw = yaw;
    this->vel = vel;
    this->acc = acc;
  }
};

} // namespace planning

#endif // STATE_H
