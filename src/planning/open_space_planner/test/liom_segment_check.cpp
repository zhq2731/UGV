#include "hybrid_a_star/liom_optimizer.h"
#include "hybrid_a_star/openSpacePlanner.h"
#include "liom_test_config.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <iostream>

using planning_msgs::TrajectoryPointArray;

TrajectoryPointArray Seed(const OpenSpace_config &c,bool forward,double curvature,
                           double heading = 0.0) {
  TrajectoryPointArray path;
  path.header.frame_id = "map";
  path.header.seq = 23;
  path.task_area = "liom_stage2_check";
  path.is_forward_shift = forward;
  const double direction = forward ? 1.0 : -1.0;
  const int intervals = std::ceil(6.0/c.trajectory_resample_resolution);
  for (int i = 0; i <= intervals; ++i) {
    planning_msgs::TrajectoryPoint p;
    p.s = 6.0*i/intervals;
    const double yaw = direction*p.s*curvature;
    const double x = curvature == 0.0 ? direction*p.s : std::sin(yaw)/curvature;
    const double y = curvature == 0.0 ? 0.0 : (1.0-std::cos(yaw))/curvature;
    // 使用较大世界坐标并覆盖 ±π，验证平移与航向展开不会改变接口语义。
    p.x = 400000.0+x*std::cos(heading)-y*std::sin(heading);
    p.y = 3000000.0+x*std::sin(heading)+y*std::cos(heading);
    p.theta = std::atan2(std::sin(heading+yaw),std::cos(heading+yaw));
    p.kappa = direction*curvature;
    path.points.push_back(p);
  }
  // 调用实际 UGV 速度规划，不另写一套速度初值算法。
  return OpenSpacePlanner::GenerateVelocityProfile(c,path);
}

bool Check(const TrajectoryPointArray &seed,const TrajectoryPointArray &out,
           const OpenSpace_config &c) {
  if (out.points.size() != seed.points.size() || out.is_forward_shift != seed.is_forward_shift ||
      out.header.frame_id != seed.header.frame_id || out.task_area != seed.task_area ||
      out.header.seq != seed.header.seq) return false;
  const double sign = out.is_forward_shift ? 1.0 : -1.0;
  const double vmax = out.is_forward_shift ? c.forward_speed : c.reverse_speed;
  for (size_t i : {size_t(0),out.points.size()-1}) {
    const auto &a = seed.points[i], &b = out.points[i];
    if (std::hypot(a.x-b.x,a.y-b.y) > 1e-7 ||
        std::abs(std::remainder(a.theta-b.theta,2*M_PI)) > 1e-7 || std::abs(b.v) > 1e-7)
      return false;
  }
  for (size_t i = 0; i < out.points.size(); ++i) {
    const auto &p = out.points[i];
    const double phi = std::atan(c.wheel_base*sign*p.kappa);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta) ||
        !std::isfinite(p.v) || !std::isfinite(p.a) || !std::isfinite(p.kappa) ||
        !std::isfinite(p.dkappa) || !std::isfinite(p.s) || !std::isfinite(p.relative_time) ||
        sign*p.v < -1e-7 || sign*p.v > vmax+1e-7 ||
        sign*p.a < -c.max_deceleration-1e-7 || sign*p.a > c.max_acceleration+1e-7 ||
        std::abs(phi) > c.steering_angle*M_PI/180+1e-7 ||
        p.v*p.v*std::abs(p.kappa) > c.max_lateral_acceleration+1e-7) return false;
    if (i+1 == out.points.size()) continue;
    const auto &q = out.points[i+1];
    const double dt = q.relative_time-p.relative_time;
    const double next_phi = std::atan(c.wheel_base*sign*q.kappa);
    if (!(dt > 0) || !(q.s > p.s) ||
        std::abs(next_phi-phi) > dt*c.max_front_tire_steering_rate+1e-3) return false;
    // 独立用消息字段复算运动学，特别检查倒车的曲率/转角符号。
    const double ex = q.x-p.x-dt*(p.v*std::cos(p.theta)+q.v*std::cos(q.theta))/2;
    const double ey = q.y-p.y-dt*(p.v*std::sin(p.theta)+q.v*std::sin(q.theta))/2;
    const double etheta = std::remainder(q.theta-p.theta,2*M_PI)-
      dt*(p.v*std::tan(phi)+q.v*std::tan(next_phi))/(2*c.wheel_base);
    if (std::hypot(ex,ey) > 2e-3 || std::abs(etheta) > 1e-3 ||
        std::abs(q.v-p.v-dt*(p.a+q.a)/2) > 1e-3) return false;
  }
  return true;
}

int main(int argc,char **argv) {
  if (argc != 2) { std::cerr << "usage: liom_segment_check openspace.yaml\n"; return 2; }
  const auto vehicle = LoadVehicle(argv[1]);
  int failures = 0;
  for (bool forward : {true,false}) {
    for (double curvature : {0.0,0.08}) {
      const auto seed = Seed(vehicle,forward,curvature,curvature == 0 ? 0.0 : 3.0);
      TrajectoryPointArray output;
      open_space::LiomReport report;
      const bool solved = open_space::OptimizeLiomFreeSpaceSegment(seed,vehicle,output,report);
      const bool passed = solved && Check(seed,output,vehicle);
      std::cout << (forward ? "forward" : "reverse") << " curvature=" << curvature
        << " passed=" << passed << " status=" << report.solver_status
        << " outer=" << report.penalty_iterations << " residual=" << report.max_residual
        << " time_ms=" << report.elapsed_ms << " message=" << report.message << '\n';
      failures += !passed;
    }
  }
  const auto seed = Seed(vehicle,true,0.08);
  TrajectoryPointArray output;
  output.task_area = "unchanged_on_failure";
  open_space::LiomReport report;
  open_space::LiomOptions options;
  options.max_iterations = 1;
  const bool rejected = !open_space::OptimizeLiomFreeSpaceSegment(seed,vehicle,output,report,options)
    && output.task_area == "unchanged_on_failure" && output.points.empty();
  std::cout << "iteration_limit_rejected=" << rejected << " status=" << report.solver_status << '\n';
  failures += !rejected;

  options = open_space::LiomOptions{};
  options.max_runtime_ms = 5;
  const bool timed_out = !open_space::OptimizeLiomFreeSpaceSegment(seed,vehicle,output,report,options)
    && report.timed_out && output.task_area == "unchanged_on_failure" && output.points.empty();
  std::cout << "wall_budget_rejected=" << timed_out << " status=" << report.solver_status
            << " elapsed_ms=" << report.elapsed_ms << '\n';
  failures += !timed_out;

  options = open_space::LiomOptions{};
  options.initial_penalty = 100;
  options.max_penalty_iterations = 5;
  const bool recovered = open_space::OptimizeLiomFreeSpaceSegment(seed,vehicle,output,report,options)
    && report.penalty_iterations > 1 && Check(seed,output,vehicle);
  std::cout << "penalty_loop_passed=" << recovered << " outer=" << report.penalty_iterations
    << " residual=" << report.max_residual << " message=" << report.message << '\n';
  failures += !recovered;

  auto invalid = seed;
  invalid.points[1].relative_time = invalid.points[0].relative_time;
  output.task_area = "invalid_seed_unchanged";
  const bool invalid_rejected = !open_space::OptimizeLiomFreeSpaceSegment(invalid,vehicle,output,report)
    && output.task_area == "invalid_seed_unchanged";
  std::cout << "invalid_time_rejected=" << invalid_rejected << '\n';
  failures += !invalid_rejected;

  auto short_seed = Seed(vehicle,true,0.0);
  short_seed.points.resize(3);
  short_seed = OpenSpacePlanner::GenerateVelocityProfile(vehicle,short_seed);
  const bool short_passed = open_space::OptimizeLiomFreeSpaceSegment(short_seed,vehicle,output,report)
    && Check(short_seed,output,vehicle);
  std::cout << "short_segment_passed=" << short_passed << " message=" << report.message << '\n';
  failures += !short_passed;
  std::cout << "failures=" << failures << '\n';
  return failures == 0 ? 0 : 1;
}
