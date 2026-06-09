#pragma once

namespace ugv {
namespace common {

struct VehicleState
{
    double x;
	double y;
	double heading;
	double kappa;
	double steering;//前轮v//2v3b1/9其c0rf 
	double v;
	double a;
	int gear;
};

}  // namespace common
}  // namespace ugv


