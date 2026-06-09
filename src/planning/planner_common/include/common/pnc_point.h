
#pragma once

#include <iostream>
#include <vector>
using namespace std;

namespace ugv {
namespace planning {

struct PathPoint {
  // coordinates
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;

  // direction on the x-y plane
    double theta_ = 0.0;
  // curvature on the x-y planning
    double kappa_ = 0.0;
  // accumulated distance from beginning of the path
    double s_ = 0.0;

  // derivative of kappa w.r.t s.
    double dkappa_ = 0.0;
  // derivative of derivative of kappa w.r.t s.
    double ddkappa_ = 0.0;
  // The lane ID where the path point is on
    string lane_id_ = "0";

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline5
    double x_derivative_ = 0.0;
    double y_derivative_ = 0.0;

    void set_x(double inp) {x_=inp;}
    void set_y(double inp) {y_=inp;}
    void set_z(double inp) {z_=inp;}
    void set_s(double inp) {s_=inp;}
    void set_theta(double inp) {theta_=inp;}
    void set_kappa(double inp) {kappa_=inp;}
    void set_dkappa(double inp) {dkappa_=inp;}
    void set_ddkappa(double inp) {ddkappa_=inp;}

	double x() const{return x_;}
    double y() const{return y_;}
    double z() const{return z_;}
    double s() const{return s_;}
    double theta() const{return theta_;}
    double kappa() const{return kappa_;}
    double dkappa() const{return dkappa_;}
    double ddkappa() const{return ddkappa_;}	
};


 struct TrajectoryPoint {
  // path point
    PathPoint path_point_;
  // linear velocity
    double v_ = 0.0;  // in [m/s]
  // linear acceleration
    double a_ = 0.0;
  // relative time from beginning of the trajectory
    double relative_time_ = 0.0;
  // longitudinal jerk
    double da_ = 0.0;
  // The angle between vehicle front wheel and vehicle longitudinal axis
    double steer_ =0.0;
    double probability_ = 0.0;
    double s_;
    void set_v(double inp) {v_=inp;}
    void set_a(double inp) {a_=inp;}
    void set_relative_time(double inp) {relative_time_ = inp;}
    void set_path_point(PathPoint &inp) {path_point_ = inp;}
    void set_steer(double inp) {steer_ = inp;}
    void set_s(double inp) {s_ = inp;}

	double v()const  {return v_;}
    double a()const  {return a_;}
    double relative_time()const  {return relative_time_;}
    double steer()const  {return steer_;}
	PathPoint path_point()const {return path_point_;}
	PathPoint* mutable_path_point() { return &path_point_;}
};


struct FrenetFramePoint {
	double s_ = 0.0;
	double l_ = 0.0;
	double dl_ = 0.0;
	double ddl_ = 0.0;
	
	void set_s(double inp) {s_=inp;}
	void set_l(double inp) {l_=inp;}
	void set_dl(double inp) {dl_ = inp;}
	void set_ddl(double inp) {ddl_ = inp;}

	double s() const{return s_;}
	double l() const{return l_;}
	double dl() const{return dl_;}
	double ddl() const{return ddl_;}
};


struct SLPoint {
  double s_ = 0.0;
  double l_ = 0.0;
  void set_s(double inp) {s_=inp;}
  void set_l(double inp) {l_=inp;}  
  double s() const{return s_;}
  double l() const{return l_;}

};

struct SLBoundary {
	double start_s_ = 0.0;
	double end_s_ = 0.0;
	double start_l_ = 0.0;
	double end_l_ = 0.0;
	std::vector<SLPoint> boundary_point;
	void set_start_s(double inp) {start_s_=inp;}
	void set_end_s(double inp) {end_s_=inp;}
	void set_start_l(double inp) {start_l_=inp;}
	void set_end_l(double inp) {end_l_=inp;}

	double start_s() const{return start_s_;}
	double end_s() const{return end_s_;}
	double start_l() const{return start_l_;}
	double end_l() const{return end_l_;}
	unsigned int boundary_point_size()const{return boundary_point.size();};
};

struct Trajectory {
  double probability = 1;
  std::vector<TrajectoryPoint> trajectory_point;
};

struct SpeedPoint {
	double s_ = 0.0;
	double t_ = 0.0;
	// speed (m/s)
	double v_ = 0.0;
	// acceleration (m/s^2)
	double a_ = 0.0;
	// jerk (m/s^3)
	double da_ = 0.0;
	
	void set_s(double inp) {s_=inp;}
	void set_t(double inp) {t_=inp;}
	void set_v(double inp) {v_ = inp;}
	void set_a(double inp) {a_ = inp;}
	void set_da(double inp) {da_ = inp;}

	double s() const{return s_;}
	double t() const{return t_;}
	double v() const{return v_;}
	double a() const{return a_;}
	double da() const{return da_;}

};
}
}

