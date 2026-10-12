#include "hybrid_a_star/openSpacePlanner.h"
#include "hybrid_a_star/liom_grid.h"
#include "liom_test_config.h"
#include <ros/master.h>
#include <cmath>
#include <iostream>
#include <memory>

using planning_msgs::TrajectoryPointArray;
int published = 0;

// 只记录原规划器的显示回调，不向控制话题发布轨迹。
void Record(const TrajectoryPointArray *,const TrajectoryPoint *,const PathBoundary *,
            const PathBoundary *,const ReferenceLine *) { ++published; }

std::unique_ptr<OpenSpacePlanner> Planner(bool enabled,double budget) {
  ros::NodeHandle nh("~");
  nh.setParam("liom_enabled",enabled);
  nh.setParam("liom_time_budget_ms",budget);
  published = 0;
  return std::make_unique<OpenSpacePlanner>(&Record);
}

InputData Scenario(bool obstacle = false,bool cusp = false) {
  InputData input;
  input.vehicleState = VehicleState{};
  input.goalState = VehicleState{};
  input.vehicleState.x = 400000; input.vehicleState.y = 3000000; input.vehicleState.heading = 0.6;
  const double x = cusp ? 4 : obstacle ? 12 : 8, y = cusp ? 4 : 0;
  input.goalState.x = input.vehicleState.x+x*std::cos(0.6)-y*std::sin(0.6);
  input.goalState.y = input.vehicleState.y+x*std::sin(0.6)+y*std::cos(0.6);
  input.goalState.heading = 0.6+(cusp ? M_PI/2 : 0);
  auto &map = input.occGrid;
  map.header.frame_id = "parking_local";
  map.info.width = map.info.height = 400; map.info.resolution = 0.1;
  map.info.origin.position.x = map.info.origin.position.y = -20;
  map.info.origin.orientation.w = 1;
  map.data.assign(400*400,25);
  if (obstacle) {
    for (int gy = 180; gy < 220; ++gy)
      for (int gx = 250; gx < 270; ++gx) map.data[gy*400+gx] = 100;
  }
  return input;
}

bool Run(OpenSpacePlanner &planner,const InputData &input,TrajectoryPointArray &out) {
  planner.setInputData(input);
  const bool changed = planner.implement(ros::Time::now().toSec(),DiscretizedTrajectory{});
  planner.getTrajectory(out);
  return changed;
}

bool SamePath(const TrajectoryPointArray &a,const TrajectoryPointArray &b) {
  return a.is_forward_shift == b.is_forward_shift && a.points == b.points;
}

bool Stop(const TrajectoryPointArray &path) {
  if (path.points.size() != 3 || std::abs(path.points.back().v) > 1e-7) return false;
  for (const auto &p : path.points)
    if (std::hypot(p.x-path.points.front().x,p.y-path.points.front().y) > 1e-7) return false;
  return true;
}

int main(int argc,char **argv) {
  ros::init(argc,argv,"liom_planner_check");
  if ((argc != 2 && argc != 3) || !ros::master::check()) {
    std::cerr << "requires openspace.yaml [strict_output_config.yaml] and an isolated ROS master\n";
    return 2;
  }
  ros::NodeHandle nh("~");
  nh.setParam("open_space_config_file",std::string(argv[1]));
  const auto vehicle = LoadVehicle(argv[1]);
  const auto straight = Scenario();
  geometry_msgs::Pose2D pose;
  pose.x = straight.vehicleState.x; pose.y = straight.vehicleState.y; pose.theta = straight.vehicleState.heading;
  open_space::LiomGrid grid(straight.occGrid,pose,vehicle);
  int failures = 0;
  const auto expect = [&](bool ok,const char *name) {
    std::cout << name << '=' << ok << std::endl;
    failures += !ok;
  };

  auto disabled = Planner(false,1000);
  TrajectoryPointArray original,optimized;
  expect(Run(*disabled,straight,original) && published == 1 && !Stop(original),"disabled_keeps_search_path");
  auto enabled = Planner(true,1000);
  const bool adopted = Run(*enabled,straight,optimized);
  expect(adopted && published == 1 && !SamePath(original,optimized) && !Stop(optimized) &&
         grid.TrajectoryFree(grid.ToGrid(optimized)),"normal_planner_adopts_optimized_path");

  // 极小预算稳定触发超时，原轨迹全段安全时才允许原样回退。
  auto timed = Planner(true,0.001);
  TrajectoryPointArray fallback;
  expect(Run(*timed,straight,fallback) && published == 1 && SamePath(original,fallback),
         "timeout_uses_checked_search_fallback");
  // 该绕障搜索轨迹的 0.4 m 扫掠复核会失败，但 0.2 m 交付复核通过，允许原样回退。
  auto detour_search = Planner(false,1000);
  TrajectoryPointArray detour_original,detour_fallback,stopped;
  const bool detour_searched = Run(*detour_search,Scenario(true),detour_original);
  auto detour_timed = Planner(true,0.001);
  expect(detour_searched && Run(*detour_timed,Scenario(true),detour_fallback) && published == 1 &&
         !Stop(detour_fallback) && SamePath(detour_original,detour_fallback),
         "timeout_uses_output_margin_for_detour_fallback");
  if (argc == 3) {
    // 对照配置只把交付余量改回 0.4 m，确认参数真正生效且拒绝时仍发布停车轨迹。
    nh.setParam("open_space_config_file",std::string(argv[2]));
    auto blocked = Planner(true,0.001);
    expect(Run(*blocked,Scenario(true),stopped) && published == 1 && Stop(stopped),
           "strict_output_margin_rejects_detour_fallback");
    nh.setParam("open_space_config_file",std::string(argv[1]));
  }

  // 墙下缘位于 y=1.92 m：0.4 m 余量扫掠安全，但额外 0.025 m 建框缓冲不满足。
  auto near_wall = straight;
  near_wall.occGrid.info.origin.position.y = -20.08;
  for (int gx = 50; gx < 360; ++gx) near_wall.occGrid.data[220*400+gx] = 100;
  auto wall_baseline = Planner(false,1000);
  TrajectoryPointArray wall_seed,wall_fallback;
  const bool wall_searched = Run(*wall_baseline,near_wall,wall_seed);
  auto wall_planner = Planner(true,1000);
  expect(wall_searched && Run(*wall_planner,near_wall,wall_fallback) && published == 1 &&
         SamePath(wall_seed,wall_fallback),"corridor_failure_uses_checked_search_fallback");

  // 只注入到达换挡点的停车位姿，不模拟车辆跟踪；真实状态机应提交缓存的下一段。
  auto cusp_input = Scenario(false,true);
  auto cusp = Planner(true,1000);
  TrajectoryPointArray first,next;
  const bool first_ok = Run(*cusp,cusp_input,first);
  if (first_ok && !first.points.empty()) {
    const auto &end = first.points.back();
    cusp_input.vehicleState.x = end.x; cusp_input.vehicleState.y = end.y;
    cusp_input.vehicleState.heading = end.theta;
    const bool held = !Run(*cusp,cusp_input,next);
    const bool committed = Run(*cusp,cusp_input,next);
    expect(held && committed && published == 2 && next.is_forward_shift != first.is_forward_shift &&
           std::hypot(next.points.front().x-end.x,next.points.front().y-end.y) < 1e-7 &&
           std::abs(next.points.front().v) < 1e-7,"optimized_cusp_keeps_cached_segment_execution");
  } else expect(false,"optimized_cusp_keeps_cached_segment_execution");

  bool moving_brakes = true;
  for (double speed : {0.2,-0.2}) {
    auto moving_input = straight; moving_input.vehicleState.v = speed;
    auto moving = Planner(true,1000);
    moving_brakes = Run(*moving,moving_input,stopped) && published == 1 && Stop(stopped) &&
      stopped.is_forward_shift == (speed > 0) && moving_brakes;
  }
  expect(moving_brakes,"moving_vehicle_brakes_in_actual_direction");
  std::cout << "failures=" << failures << '\n';
  return failures ? 1 : 0;
}
