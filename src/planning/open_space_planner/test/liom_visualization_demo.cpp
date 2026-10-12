#include "hybrid_a_star/liom_optimizer.h"
#include "hybrid_a_star/openSpacePlanner.h"
#include "liom_test_config.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using planning_msgs::TrajectoryPointArray;

TrajectoryPointArray MakeDemo(const OpenSpace_config &c,bool reverse) {
  TrajectoryPointArray path;
  path.header.frame_id = "map";
  path.task_area = "liom_visualization_only";
  path.is_forward_shift = !reverse;
  const double sign = reverse ? -1.0 : 1.0;
  // 模拟搜索原语衔接造成的曲率变化；这不是实际 Hybrid A* 搜索结果。
  const double curvature[] = {0.0,0.12,-0.08,0.10,0.0};
  constexpr int count = 41;
  constexpr double ds = 0.2;
  planning_msgs::TrajectoryPoint p;
  for (int i = 0; i < count; ++i) {
    const double k = curvature[std::min(i/8,4)];
    p.s = i*ds;
    p.kappa = sign*k;
    path.points.push_back(p);
    // 常曲率区间按圆弧积分，theta 始终表示车身朝向而非倒车切线。
    const double next_theta = p.theta+sign*ds*k;
    if (std::abs(k) < 1e-9) {
      p.x += sign*ds*std::cos(p.theta);
      p.y += sign*ds*std::sin(p.theta);
    } else {
      p.x += (std::sin(next_theta)-std::sin(p.theta))/k;
      p.y += (std::cos(p.theta)-std::cos(next_theta))/k;
    }
    p.theta = next_theta;
  }
  return OpenSpacePlanner::GenerateVelocityProfile(c,path);
}

void WriteCsv(const std::string &file,const TrajectoryPointArray &path,double wheel_base) {
  std::ofstream csv(file);
  csv.exceptions(std::ios::badbit | std::ios::failbit);
  csv << "time_s,x_m,y_m,heading_rad,s_m,velocity_mps,acceleration_mps2,steer_deg\n";
  csv << std::setprecision(12);
  const double sign = path.is_forward_shift ? 1.0 : -1.0;
  for (const auto &p : path.points) {
    const double steer = std::atan(wheel_base*sign*p.kappa)*180.0/M_PI;
    csv << p.relative_time << ',' << p.x << ',' << p.y << ',' << p.theta << ','
        << p.s << ',' << p.v << ',' << p.a << ',' << steer << '\n';
  }
}

int main(int argc,char **argv) {
  if (argc != 4) {
    std::cerr << "usage: liom_visualization_demo vehicle.yaml output_directory forward|reverse\n";
    return 2;
  }
  try {
    const auto vehicle = LoadVehicle(argv[1]);
    const bool reverse = std::string(argv[3]) == "reverse";
    const auto before = MakeDemo(vehicle,reverse);
    TrajectoryPointArray after;
    open_space::LiomReport report;
    if (!open_space::OptimizeLiomFreeSpaceSegment(before,vehicle,after,report)) {
      std::cerr << "LIOM failed: " << report.message << ", status=" << report.solver_status << '\n';
      return 1;
    }
    const std::string directory = argv[2];
    WriteCsv(directory+"/before.csv",before,vehicle.wheel_base);
    WriteCsv(directory+"/after.csv",after,vehicle.wheel_base);
    // 绘图使用实际配置的物理上限，不在脚本里另写一套车辆参数。
    std::ofstream info(directory+"/summary.json");
    info.exceptions(std::ios::badbit | std::ios::failbit);
    info << std::setprecision(12) << "{\n"
      << "\"reverse\":" << (reverse ? "true" : "false") << ",\n"
      << "\"max_speed\":" << (reverse ? vehicle.reverse_speed : vehicle.forward_speed) << ",\n"
      << "\"a_min\":" << (reverse ? -vehicle.max_acceleration : -vehicle.max_deceleration) << ",\n"
      << "\"a_max\":" << (reverse ? vehicle.max_deceleration : vehicle.max_acceleration) << ",\n"
      << "\"steer_limit_deg\":" << vehicle.steering_angle << ",\n"
      << "\"rate_limit_deg_s\":" << vehicle.max_front_tire_steering_rate*180/M_PI << ",\n"
      << "\"solve_ms\":" << report.elapsed_ms << ",\n"
      << "\"max_residual\":" << report.max_residual << ",\n"
      << "\"penalty_iterations\":" << report.penalty_iterations << "\n}\n";
    std::cout << "LIOM demo: " << directory << ", solve_ms=" << report.elapsed_ms << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
