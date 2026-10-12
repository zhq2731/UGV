#include "hybrid_a_star/liom_grid.h"
#include "hybrid_a_star/liom_optimizer.h"
#include "hybrid_a_star/openSpacePlanner.h"
#include "liom_test_config.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using planning_msgs::TrajectoryPointArray;
using HybridAStarType::Vec3d;
using Clock = std::chrono::steady_clock;

nav_msgs::OccupancyGrid MakeMap(int scene) {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "parking_local";
  map.info.width = map.info.height = 400;
  map.info.resolution = 0.1;
  // 长距离场景前移同样大小的局部窗口，让默认 25 m 解析连接距离触发树展开。
  map.info.origin.position.x = scene == 5 ? -8 : -20;
  map.info.origin.position.y = -20;
  map.info.origin.orientation.w = 1;
  map.data.assign(400*400,25);
  for (unsigned y = 0; y < map.info.height; ++y) {
    for (unsigned x = 0; x < map.info.width; ++x) {
      const double px = map.info.origin.position.x+(x+0.5)*map.info.resolution;
      const double py = -20+(y+0.5)*map.info.resolution;
      // 绕障场景迫使搜索离开直线；窄通道保留约 4.3 m 净宽。
      const bool block = scene == 3 && px >= 5 && px < 7 && std::abs(py) < 2;
      const bool wall = scene == 4 && px >= -5 && px < 16 &&
        ((py >= 2.2 && py < 2.3) || (py >= -2.2 && py < -2.1));
      if (block || wall) map.data[y*map.info.width+x] = 100;
    }
  }
  return map;
}

// 独立从输出消息检查端点、档位、车辆限值及离散运动学，不仅相信求解状态。
bool CheckOutput(const std::vector<TrajectoryPointArray> &seed,
                 const std::vector<TrajectoryPointArray> &out,
                 const OpenSpace_config &c,const open_space::LiomGrid &grid) {
  if (out.size() != seed.size()) return false;
  for (size_t j = 0; j < out.size(); ++j) {
    const auto &path = out[j];
    if (path.points.size() != seed[j].points.size() ||
        path.is_forward_shift != seed[j].is_forward_shift ||
        path.header.frame_id != seed[j].header.frame_id ||
        !grid.TrajectoryFree(grid.ToGrid(path))) return false;
    for (size_t i : {size_t(0),path.points.size()-1}) {
      const auto &a = path.points[i], &b = seed[j].points[i];
      if (std::hypot(a.x-b.x,a.y-b.y) > 1e-7 ||
          std::abs(std::remainder(a.theta-b.theta,2*M_PI)) > 1e-7 || std::abs(a.v) > 1e-7) return false;
    }
    const double sign = path.is_forward_shift ? 1 : -1;
    const double vmax = path.is_forward_shift ? c.forward_speed : c.reverse_speed;
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto &p = path.points[i];
      const double phi = std::atan(sign*c.wheel_base*p.kappa);
      if (!std::isfinite(p.x+p.y+p.theta+p.v+p.a+p.kappa+p.dkappa+p.s+p.relative_time) ||
          sign*p.v < -1e-7 || sign*p.v > vmax+1e-7 ||
          sign*p.a < -c.max_deceleration-1e-7 || sign*p.a > c.max_acceleration+1e-7 ||
          std::abs(phi) > c.steering_angle*M_PI/180+1e-7 ||
          p.v*p.v*std::abs(p.kappa) > c.max_lateral_acceleration+1e-7) return false;
      if (i+1 == path.points.size()) continue;
      const auto &q = path.points[i+1];
      const double dt = q.relative_time-p.relative_time;
      const double next_phi = std::atan(sign*c.wheel_base*q.kappa);
      const double ex = q.x-p.x-dt*(p.v*std::cos(p.theta)+q.v*std::cos(q.theta))/2;
      const double ey = q.y-p.y-dt*(p.v*std::sin(p.theta)+q.v*std::sin(q.theta))/2;
      const double et = std::remainder(q.theta-p.theta,2*M_PI)-
        dt*(p.v*std::tan(phi)+q.v*std::tan(next_phi))/(2*c.wheel_base);
      if (!(dt > 0) || !(q.s > p.s) || std::hypot(ex,ey) > 2e-3 || std::abs(et) > 1e-3 ||
          std::abs(q.v-p.v-dt*(p.a+q.a)/2) > 1e-3 ||
          std::abs(next_phi-phi) > dt*c.max_front_tire_steering_rate+1e-3) return false;
    }
  }
  return true;
}

void WritePaths(std::ostream &csv,const char *name,const char *stage,
                const std::vector<TrajectoryPointArray> &paths) {
  for (size_t j = 0; j < paths.size(); ++j) {
    for (const auto &p : paths[j].points) {
      csv << name << ',' << stage << ',' << j << ',' << int(paths[j].is_forward_shift) << ','
          << p.x << ',' << p.y << ',' << p.theta << ',' << p.s << ',' << p.kappa << ','
          << p.v << ',' << p.a << ',' << p.relative_time << '\n';
    }
  }
}

int main(int argc,char **argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: liom_search_check openspace.yaml [results_directory]\n";
    return 2;
  }
  const auto vehicle = LoadVehicle(argv[1]);
  geometry_msgs::Pose2D pose;
  pose.x = 400000; pose.y = 3000000; pose.theta = 0.6;
  const Vec3d world_pose(pose.x,pose.y,pose.theta);
  const char *names[] = {"forward_straight","reverse_curve","parking_cusp","obstacle_detour","narrow_straight","search_tree_turn"};
  const Vec3d goals[] = {{8,0,0},{-6,-1,-0.3},{4,4,M_PI/2},{12,0,0},{8,0,0},{26,0,M_PI}};
  std::ofstream csv, raw_csv, obstacle_csv, metadata;
  if (argc == 3) {
    csv.open(std::string(argv[2])+"/search_paths.csv");
    if (!csv) return 2;
    csv << "scene,stage,segment,forward,x_m,y_m,heading_rad,s_m,kappa,velocity_mps,acceleration_mps2,time_s\n";
    csv << std::setprecision(12);
    raw_csv.open(std::string(argv[2])+"/search_raw.csv");
    obstacle_csv.open(std::string(argv[2])+"/search_obstacles.csv");
    metadata.open(std::string(argv[2])+"/search_metadata.json");
    if (!raw_csv || !obstacle_csv || !metadata) return 2;
    raw_csv << "scene,x_m,y_m,heading_rad\n" << std::setprecision(12);
    obstacle_csv << "scene,x_m,y_m,size_m\n" << std::setprecision(12);
    // 绘图直接使用本次地图和车辆参数，不在 Python 中另构造障碍或标定值。
    metadata << std::setprecision(12) << "{\"vehicle\":{\"length\":" << vehicle.car_length
      << ",\"width\":" << vehicle.car_width << ",\"rear_overhang\":" << vehicle.rear_overhang
      << ",\"wheel_base\":" << vehicle.wheel_base << ",\"margin\":" << vehicle.search_collision_margin
      << ",\"forward_speed\":" << vehicle.forward_speed << ",\"reverse_speed\":" << vehicle.reverse_speed
      << ",\"max_acceleration\":" << vehicle.max_acceleration << ",\"max_deceleration\":" << vehicle.max_deceleration
      << ",\"steering_angle_deg\":" << vehicle.steering_angle
      << ",\"steering_rate\":" << vehicle.max_front_tire_steering_rate
      << "},\"corridor_model\":\"oriented_body_rectangle\",\"map_pose\":[" << pose.x << ',' << pose.y << ',' << pose.theta << "],\"scenes\":[\n";
  }
  int failures = 0, accepted = 0, rejected = 0;
  for (int scene = 0; scene < 6; ++scene) {
    const auto map = MakeMap(scene);
    HybridAStar search(vehicle.steering_angle,vehicle.steering_angle_discrete_num,
      vehicle.segment_length,vehicle.segment_length_discrete_num,vehicle.wheel_base,
      vehicle.steering_penalty,vehicle.reversing_penalty,vehicle.steering_change_penalty,
      vehicle.gear_switch_penalty,vehicle.shot_distance);
    const double xmin = map.info.origin.position.x, ymin = map.info.origin.position.y;
    search.Init(xmin,xmin+40,ymin,ymin+40,1.0,map.info.resolution,vehicle.car_length,vehicle.car_width,
                vehicle.wheel_base,vehicle.rear_overhang);
    for (unsigned y = 0; y < map.info.height; ++y)
      for (unsigned x = 0; x < map.info.width; ++x)
        if (map.data[y*map.info.width+x] != 25) search.SetObstacle(x,y);
    const auto started = Clock::now();
    const bool searched = search.Search(Vec3d::Zero(),goals[scene],vehicle.search_collision_margin);
    const double search_ms = std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    const auto raw_path = search.GetPath();
    search.GetSearchedTree();
    // 与正常规划共用路径分段、世界坐标转换、重采样、曲率和速度初值。
    auto seeds = OpenSpacePlanner::GenerateSearchSegments(vehicle,raw_path,world_pose);
    for (auto &seed : seeds) seed.header.frame_id = "map";
    open_space::LiomGrid grid(map,pose,vehicle);
    const bool seed_free = !seeds.empty() && std::all_of(seeds.begin(),seeds.end(),
      [&](const TrajectoryPointArray &p) { return grid.TrajectoryFree(grid.ToGrid(p)); });
    std::vector<TrajectoryPointArray> output(1);
    output.front().task_area = "unchanged_on_failure";
    open_space::LiomReport report;
    const bool optimized = !seeds.empty() && open_space::OptimizeLiomParkingSegments(
      seeds,map,pose,vehicle,output,report);
    const bool valid = optimized && CheckOutput(seeds,output,vehicle,grid);
    const bool unchanged = !optimized && output.size() == 1 && output.front().task_area == "unchanged_on_failure";
    // 绕障初值的点间余量不足必须被修正，而不是删掉此场景或放松输出检查。
    const bool passed = searched && valid && report.corridor_builds == report.penalty_iterations &&
      (scene == 3 ? !seed_free && seeds.size() == 5 &&
      search.GetVisitedNodesNumber() > 1 : seed_free) &&
      (scene != 2 || seeds.size() > 1) && (scene != 5 || search.GetVisitedNodesNumber() > 1);
    std::cout << names[scene] << " searched=" << searched << " segments=" << seeds.size()
      << " visited=" << search.GetVisitedNodesNumber() << " seed_free=" << seed_free
      << " optimized=" << optimized << " output_valid=" << valid << " unchanged=" << unchanged
      << " search_ms=" << search_ms << " optimize_ms=" << report.elapsed_ms
      << " outer=" << report.penalty_iterations << " corridor_builds=" << report.corridor_builds
      << " corridor_ms=" << report.corridor_ms << " solver_ms=" << report.solver_ms
      << " collision_ms=" << report.collision_ms
      << " status=" << report.solver_status << " failed_segment=" << report.failed_segment
      << " reason=" << report.message << " repaired_seed=" << (optimized && !seed_free)
      << " check_passed=" << passed << std::endl;
    accepted += optimized;
    rejected += !optimized;
    if (!optimized) {
      auto nominal = vehicle;
      nominal.search_collision_margin = 0;
      open_space::LiomGrid nominal_grid(map,pose,nominal);
      for (size_t j = 0; j < seeds.size(); ++j) {
        const auto local = grid.ToGrid(seeds[j]);
        bool nodes_free = true;
        for (const auto &p : local.points) {
          TrajectoryPointArray single;
          single.points = {p};
          nodes_free = nodes_free && grid.TrajectoryFree(single);
        }
        std::vector<open_space::LiomCorridor> corridor;
        std::cout << "  segment=" << j << " length=" << seeds[j].points.back().s
          << " margin_nodes_free=" << nodes_free << " margin_sweep_free=" << grid.TrajectoryFree(local)
          << " nominal_sweep_free=" << nominal_grid.TrajectoryFree(local)
          << " seed_corridor_free=" << grid.BuildCorridors(local,corridor) << std::endl;
      }
    }
    failures += !passed;
    if (csv.is_open()) {
      WritePaths(csv,names[scene],"seed",seeds);
      if (optimized) WritePaths(csv,names[scene],"optimized",output);
      for (const auto &p : raw_path)
        raw_csv << names[scene] << ',' << p.x() << ',' << p.y() << ',' << p.z() << '\n';
      for (unsigned y = 0; y < map.info.height; ++y)
        for (unsigned x = 0; x < map.info.width; ++x)
          if (map.data[y*map.info.width+x] != 25)
            obstacle_csv << names[scene] << ',' << xmin+x*map.info.resolution << ','
              << ymin+y*map.info.resolution << ',' << map.info.resolution << '\n';
      if (scene) metadata << ",\n";
      metadata << "{\"name\":" << std::quoted(names[scene])
        << ",\"origin\":[" << xmin << ',' << ymin << "],\"resolution\":" << map.info.resolution
        << ",\"size\":[40,40],\"segments\":" << seeds.size()
        << ",\"searched\":" << searched << ",\"optimized\":" << optimized
        << ",\"output_valid\":" << valid << ",\"visited\":" << search.GetVisitedNodesNumber()
        << ",\"search_ms\":" << search_ms << ",\"optimize_ms\":" << report.elapsed_ms
        << ",\"penalty_iterations\":" << report.penalty_iterations
        << ",\"corridor_builds\":" << report.corridor_builds
        << ",\"corridor_ms\":" << report.corridor_ms << ",\"solver_ms\":" << report.solver_ms
        << ",\"collision_ms\":" << report.collision_ms
        << ",\"max_residual\":" << report.max_residual
        << ",\"failed_segment\":" << report.failed_segment
        << ",\"reason\":" << std::quoted(report.message) << '}';
    }
  }
  if (metadata.is_open()) metadata << "\n]}\n";
  std::cout << "accepted=" << accepted << " rejected=" << rejected << " check_failures=" << failures << '\n';
  return failures ? 1 : 0;
}
