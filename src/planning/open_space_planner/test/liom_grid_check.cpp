#include "hybrid_a_star/liom_grid.h"
#include "hybrid_a_star/liom_optimizer.h"
#include "hybrid_a_star/openSpacePlanner.h"
#include "liom_test_config.h"
#include <cmath>
#include <iostream>

using planning_msgs::TrajectoryPointArray;

nav_msgs::OccupancyGrid Map() {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "parking_local";
  map.info.width = map.info.height = 400;
  map.info.resolution = 0.1;
  map.info.origin.position.x = map.info.origin.position.y = -20;
  // 同时测试地图原点朝向与局部系世界朝向，不能把两者混为一谈。
  map.info.origin.orientation.z = std::sin(0.25/2);
  map.info.origin.orientation.w = std::cos(0.25/2);
  map.data.assign(400*400,25);
  return map;
}

void Cell(nav_msgs::OccupancyGrid &map,double x,double y,int value = 100) {
  const int gx = std::floor(x/map.info.resolution), gy = std::floor(y/map.info.resolution);
  map.data[gy*map.info.width+gx] = value;
}

TrajectoryPointArray Segment(const OpenSpace_config &vehicle,planning_msgs::TrajectoryPoint start,
                             bool forward,double curvature,double length) {
  TrajectoryPointArray seed;
  seed.header.frame_id = "map";
  seed.is_forward_shift = forward;
  const double sign = forward ? 1 : -1;
  const int intervals = std::ceil(length/0.2);
  for (int i = 0; i <= intervals; ++i) {
    const double distance = sign*length*i/intervals;
    const double yaw = curvature*distance;
    const double x = std::abs(curvature) < 1e-9 ? distance : std::sin(yaw)/curvature;
    const double y = std::abs(curvature) < 1e-9 ? 0 : (1-std::cos(yaw))/curvature;
    planning_msgs::TrajectoryPoint p;
    p.x = start.x+x*std::cos(start.theta)-y*std::sin(start.theta);
    p.y = start.y+x*std::sin(start.theta)+y*std::cos(start.theta);
    p.theta = start.theta+yaw;
    p.kappa = sign*curvature;
    p.s = length*i/intervals;
    seed.points.push_back(p);
  }
  return OpenSpacePlanner::GenerateVelocityProfile(vehicle,seed);
}

int main(int argc,char **argv) {
  if (argc != 2) return 2;
  auto vehicle = LoadVehicle(argv[1]);
  auto map = Map();
  geometry_msgs::Pose2D pose;
  pose.x = 400000; pose.y = 3000000; pose.theta = 0.6;
  int failures = 0;
  const auto expect = [&](bool passed,const char *name) {
    std::cout << name << "=" << passed << '\n';
    failures += !passed;
  };

  planning_msgs::TrajectoryPoint start;
  start.x = start.y = 20;
  auto first = Segment(vehicle,start,true,0.05,6);
  auto second = Segment(vehicle,first.points.back(),false,-0.08,4);
  // 在路径两侧放置栅格墙，地图不是全空；优化必须遵循局部安全空间。
  for (double x = 5; x < 36; x += 0.05) {
    Cell(map,x,25.5); Cell(map,x,14.5);
  }
  open_space::LiomGrid grid(map,pose,vehicle);
  std::vector<TrajectoryPointArray> seeds{grid.ToWorld(first),grid.ToWorld(second)}, output;
  open_space::LiomReport report;
  const bool solved = open_space::OptimizeLiomParkingSegments(seeds,map,pose,vehicle,output,report);
  expect(solved,"two_segments_solved");
  std::cout << "elapsed_ms=" << report.elapsed_ms << " residual=" << report.max_residual
            << " message=" << report.message << '\n';
  if (solved) {
    bool endpoints = output.size() == seeds.size();
    for (size_t i = 0; i < output.size(); ++i) {
      endpoints = endpoints && output[i].is_forward_shift == seeds[i].is_forward_shift &&
        output[i].header.frame_id == seeds[i].header.frame_id && grid.TrajectoryFree(grid.ToGrid(output[i]));
      for (size_t j : {size_t(0),output[i].points.size()-1}) {
        const auto &a = output[i].points[j], &b = seeds[i].points[j];
        endpoints = endpoints && std::hypot(a.x-b.x,a.y-b.y) < 1e-7 &&
          std::abs(std::remainder(a.theta-b.theta,2*M_PI)) < 1e-7 && std::abs(a.v) < 1e-7;
      }
    }
    expect(endpoints,"cusp_pose_zero_speed_and_sweep");
    const double previous = std::atan(vehicle.wheel_base*output[0].points.back().kappa);
    const double next = std::atan(-vehicle.wheel_base*output[1].points.front().kappa);
    expect(std::abs(next-previous) > 1e-3,"cusp_steering_discontinuity_allowed");
    std::cout << "cusp_steering_change_deg=" << (next-previous)*180/M_PI << '\n';
  }

  // 用较小初始惩罚触发多轮：确认每轮重建走廊，最终仍通过完整扫掠。
  open_space::LiomOptions options;
  options.initial_penalty = 100;
  options.max_penalty_iterations = 5;
  const bool rebuilt = open_space::OptimizeLiomParkingSegments(
    {seeds.front()},map,pose,vehicle,output,report,options);
  expect(rebuilt && report.penalty_iterations > 1 && report.corridor_builds == report.penalty_iterations &&
         grid.TrajectoryFree(grid.ToGrid(output.front())),"corridors_rebuilt_each_penalty_round");
  std::cout << "outer=" << report.penalty_iterations << " builds=" << report.corridor_builds
            << " corridor_ms=" << report.corridor_ms << " solver_ms=" << report.solver_ms
            << " collision_ms=" << report.collision_ms << " reason=" << report.message << '\n';
  options.max_penalty_iterations = 1;
  output.resize(1); output.front().task_area = "unchanged_on_exhaustion";
  const bool exhausted = open_space::OptimizeLiomParkingSegments(
    {seeds.front()},map,pose,vehicle,output,report,options);
  expect(!exhausted && output.size() == 1 && output.front().task_area == "unchanged_on_exhaustion" &&
         report.penalty_iterations == 1 && report.corridor_builds == 1,
         "outer_limit_keeps_output_unchanged");

  const auto reject = [&](const std::vector<TrajectoryPointArray> &input,const nav_msgs::OccupancyGrid &blocked,
                           const char *name,int failed_segment) {
    std::vector<TrajectoryPointArray> sentinel(1);
    sentinel[0].task_area = "unchanged";
    const bool ok = open_space::OptimizeLiomParkingSegments(input,blocked,pose,vehicle,sentinel,report);
    expect(!ok && sentinel.size() == 1 && sentinel[0].task_area == "unchanged" &&
           report.failed_segment == failed_segment,name);
    std::cout << "  reason=" << report.message << '\n';
  };
  options = open_space::LiomOptions{};
  options.max_runtime_ms = 0.001;
  output.resize(1); output.front().task_area = "unchanged_on_timeout";
  const bool budget_rejected = !open_space::OptimizeLiomParkingSegments(
    seeds,map,pose,vehicle,output,report,options) && report.timed_out && output.size() == 1 &&
    output.front().task_area == "unchanged_on_timeout";
  expect(budget_rejected,"multi_segment_budget_keeps_output_unchanged");
  auto blocked = map;
  Cell(blocked,21,20);
  reject(seeds,blocked,"body_internal_one_cell_rejected",0);
  Cell(blocked,21,20,-1);
  reject(seeds,blocked,"unknown_cell_rejected",0);
  Cell(blocked,21,20,0);
  reject(seeds,blocked,"non_25_cell_rejected",0);

  auto discontinuous = seeds;
  discontinuous[1].points.front().y += 0.01;
  reject(discontinuous,map,"broken_cusp_rejected",1);

  blocked = map;
  const auto &end = second.points.back();
  Cell(blocked,end.x-std::cos(end.theta)+1.4*std::sin(end.theta),
               end.y-std::sin(end.theta)-1.4*std::cos(end.theta));
  reject(seeds,blocked,"second_segment_failure_is_atomic",1);
  expect(report.corridor_builds >= 2 && report.penalty_iterations >= 1 && report.solver_ms > 0,
         "second_segment_failure_preserves_work_report");

  // 以下直接检查几何，不受求解器是否恰好绕开障碍的影响。
  TrajectoryPointArray single;
  single.points = {start};
  blocked = map;
  Cell(blocked,21,20+vehicle.car_width/2+0.2);
  auto zero_margin = vehicle;
  zero_margin.search_collision_margin = 0;
  expect(!open_space::LiomGrid(blocked,pose,vehicle).TrajectoryFree(single) &&
          open_space::LiomGrid(blocked,pose,zero_margin).TrajectoryFree(single),"configured_margin_applied");

  // 障碍距真实车侧约 0.3 m：优化结果的 0.2 m 复核通过，默认搜索余量仍拒绝。
  blocked = map;
  Cell(blocked,21,20+vehicle.car_width/2+0.35);
  auto short_path = single;
  short_path.points.push_back(start);
  short_path.points.back().x += 0.2;
  expect(open_space::LiomGrid(blocked,pose,vehicle).TrajectoryFree(short_path,vehicle.output_collision_margin) &&
         !open_space::LiomGrid(blocked,pose,vehicle).TrajectoryFree(short_path),
         "output_margin_override_preserves_search_margin");
  blocked = map;
  Cell(blocked,21,20+vehicle.car_width/2+0.15);
  expect(!open_space::LiomGrid(blocked,pose,vehicle).TrajectoryFree(short_path,vehicle.output_collision_margin),
         "output_margin_still_rejects_near_obstacle");

  // 只调大交付余量，不改变搜索和走廊；优化结果必须使用该参数，而非写死 0.2 m。
  auto strict_output = vehicle;
  strict_output.output_collision_margin = 4.0;
  std::vector<TrajectoryPointArray> rejected_output(1);
  rejected_output.front().task_area = "unchanged_on_failure";
  open_space::LiomReport strict_report;
  expect(!open_space::OptimizeLiomParkingSegments(seeds,map,pose,strict_output,
           rejected_output,strict_report) &&
         strict_report.message == "optimized footprint/sweep collides; output unchanged" &&
         rejected_output.size() == 1 && rejected_output.front().task_area == "unchanged_on_failure",
         "configured_output_margin_applied_to_optimized_result");

  single.points[0].y = 20.03;
  blocked = map;
  // 占据格中心在车外，但单元边缘与车身重叠，不能按栅格中心漏掉它。
  Cell(blocked,21,20.03+vehicle.car_width/2+0.01);
  expect(!open_space::LiomGrid(blocked,pose,zero_margin).TrajectoryFree(single),"partial_cell_overlap_rejected");

  blocked = map;
  Cell(blocked,20,20);
  TrajectoryPointArray sparse;
  sparse.points = {start,start};
  sparse.points.front().x = 10; sparse.points.back().x = 30;
  expect(!open_space::LiomGrid(blocked,pose,vehicle).TrajectoryFree(sparse),"between_points_obstacle_rejected");

  blocked = map;
  Cell(blocked,22.5,22.5);
  sparse.points = {start,start}; sparse.points.back().theta = M_PI/2;
  single.points = {sparse.points.front()};
  const bool rotation_start_free = open_space::LiomGrid(blocked,pose,vehicle).TrajectoryFree(single);
  single.points = {sparse.points.back()};
  expect(rotation_start_free && open_space::LiomGrid(blocked,pose,vehicle).TrajectoryFree(single) &&
         !open_space::LiomGrid(blocked,pose,vehicle).TrajectoryFree(sparse),"rotation_sweep_obstacle_rejected");
  single.points = {start}; single.points[0].x = 1;
  expect(!grid.TrajectoryFree(single),"map_boundary_rejected");

  auto invalid_map = map;
  invalid_map.data.pop_back();
  expect(!open_space::LiomGrid(invalid_map,pose,vehicle).valid(),"malformed_map_rejected");

  // 旧四圆末端伸到障碍中，但矩形车身及配置余量仍安全，应能构建新走廊。
  auto near_map = Map();
  Cell(near_map,25,20);
  open_space::LiomGrid near_grid(near_map,pose,vehicle);
  single.points = {start};
  std::vector<open_space::LiomCorridor> corridors;
  expect(near_grid.TrajectoryFree(single) && near_grid.BuildCorridors(single,corridors),
         "body_corridor_avoids_circle_overhang");
  Cell(near_map,21,20);
  expect(!open_space::LiomGrid(near_map,pose,vehicle).BuildCorridors(single,corridors),
         "corridor_internal_cell_rejected");

  // 直通道的净宽约 4.3 m：5x3 m 车身加 0.4 m 余量能通过。
  auto narrow_map = Map();
  for (double x = 5; x < 36; x += 0.05) {
    Cell(narrow_map,x,17.8); Cell(narrow_map,x,22.2);
  }
  open_space::LiomGrid narrow_grid(narrow_map,pose,vehicle);
  auto straight = Segment(vehicle,start,true,0,6);
  expect(narrow_grid.TrajectoryFree(straight),"narrow_seed_footprint_free");
  const bool narrow_solved = open_space::OptimizeLiomParkingSegments(
    {narrow_grid.ToWorld(straight)},narrow_map,pose,vehicle,output,report);
  expect(narrow_solved && narrow_grid.TrajectoryFree(narrow_grid.ToGrid(output.front())),
         "narrow_corridor_solved");
  std::cout << "corner_count=" << open_space::kLiomCorners << " narrow_ms=" << report.elapsed_ms
            << " reason=" << report.message << '\n';

  // 弓形绕行初值：无障碍优化会拉直穿过中间障碍，检验走廊实际参与优化。
  auto detour_map = Map();
  TrajectoryPointArray detour;
  detour.header.frame_id = "map"; detour.is_forward_shift = true;
  planning_msgs::TrajectoryPoint current;
  current.x = 8; current.y = 16;
  for (double curvature : {0.18,-0.18,-0.18,0.18}) {
    const auto piece = Segment(vehicle,current,true,curvature,4.5);
    detour.points.insert(detour.points.end(),piece.points.begin()+(detour.points.empty() ? 0 : 1),piece.points.end());
    current = piece.points.back();
  }
  detour.points.front().s = 0;
  for (size_t i = 1; i < detour.points.size(); ++i)
    detour.points[i].s = detour.points[i-1].s+std::hypot(detour.points[i].x-detour.points[i-1].x,
                                                       detour.points[i].y-detour.points[i-1].y);
  detour = OpenSpacePlanner::GenerateVelocityProfile(vehicle,detour);
  Cell(detour_map,(detour.points.front().x+detour.points.back().x)/2,16);
  open_space::LiomGrid detour_grid(detour_map,pose,vehicle);
  const auto world_detour = detour_grid.ToWorld(detour);
  TrajectoryPointArray unconstrained;
  const bool free_solved = open_space::OptimizeLiomFreeSpaceSegment(world_detour,vehicle,unconstrained,report);
  expect(free_solved && !detour_grid.TrajectoryFree(detour_grid.ToGrid(unconstrained)),
         "unconstrained_optimization_cuts_obstacle");
  const bool constrained = open_space::OptimizeLiomParkingSegments({world_detour},detour_map,pose,vehicle,output,report);
  expect(constrained && detour_grid.TrajectoryFree(detour_grid.ToGrid(output.front())),
         "corridor_prevents_corner_cutting");
  std::cout << "detour_ms=" << report.elapsed_ms << " reason=" << report.message << '\n';
  std::cout << "failures=" << failures << '\n';
  return failures ? 1 : 0;
}
