#pragma once

#include <iostream>
#include <vector>
#include "common/obstacle.h"

using namespace std;

namespace ugv {
namespace planning {

class Frame {
 public:
	const TrajectoryPoint PlanningStartPoint() const{return planning_start_point;}
	TrajectoryPoint planning_start_point;
 private:
      std::vector< Obstacle*> obstacles_;
};

}
}
