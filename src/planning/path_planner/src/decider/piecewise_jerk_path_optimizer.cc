/******************************************************************************
 * Copyright 2019 The ugv Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file piecewise_jerk_path_optimizer.cc
 **/

#include <memory>
#include <string>

#include "decider/piecewise_jerk_path_optimizer.h"
#include "common/planning_config.h"
#include "math/math_utils.h"
#include "optimizer/piecewise_jerk_trajectory1d.h"
#include "optimizer/piecewise_jerk_path_problem.h"
#include "vehicle_info_util/vehicle_info.hpp"

namespace ugv {
namespace planning {
using namespace ugv::common::math;

bool PiecewiseJerkPathOptimizer::Process( ReferenceLineInfo* reference_line_info_,
    const TrajectoryPoint& init_point, const bool path_reusable) {
  TrajectoryPoint planning_start_point = init_point;
  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  PlanningConfig  *config =  PlanningConfig::get_instance();
  const auto init_frenet_state =
      reference_line.ToFrenetFrame(planning_start_point);

  /*
  std::cout <<"init_frenet_state----"<<std::endl;
  std::cout <<init_frenet_state.first[0]<<std::endl;
  std::cout <<init_frenet_state.first[1]<<std::endl;
  std::cout <<init_frenet_state.first[2]<<std::endl;
  
  std::cout <<init_frenet_state.second[0]<<std::endl;
  std::cout <<init_frenet_state.second[1]<<std::endl;
  std::cout <<init_frenet_state.second[2]<<std::endl;
  */
  std::array<double, 5> w {
	      config->jerkPathOpimizerConfig.default_l_weight,
	      config->jerkPathOpimizerConfig.default_dl_weight *
	          std::fmax(init_frenet_state.first[1] * init_frenet_state.first[1],5.0),
	      config->jerkPathOpimizerConfig.default_ddl_weight, config->jerkPathOpimizerConfig.default_dddl_weight, 0.0};

  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
   //ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";
   // const auto& reference_path_data = reference_line_info_->path_data();

  std::vector<PathData> candidate_path_data;
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
    // if the path_boundary is normal, it is possible to have less than 2 points
    // skip path boundary of this kind
    if (path_boundary.label().find("regular") != std::string::npos &&
        path_boundary_size < 2) {
      continue;
    }

    int max_iter = 4000;
    // lower max_iter for regular/self/
    if (path_boundary.label().find("self") != std::string::npos) {
      max_iter = 4000;
    }
    assert(path_boundary_size > 1);

    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;

    std::array<double, 3> end_state = {0.0, 0.0, 0.0};

    // TODO(all): double-check this;
    // final_path_data might carry info from upper stream
    PathData path_data ;

    // updated cost function for path reference
    std::vector<double> path_reference_l(path_boundary_size, 0.0);
    bool is_valid_path_reference = false;
    //size_t path_reference_size = reference_path_data.path_reference().size();
    size_t path_reference_size = 0;
	/*
    if (path_boundary.label().find("regular") != std::string::npos ) {
     // ADEBUG << "path label is: " << path_boundary.label();
      // when path reference is ready
      for (size_t i = 0; i < path_reference_size; ++i) {
        SLPoint path_reference_sl;
        reference_line.XYToSL(
            common::util::PointFactory::ToPointENU(
                reference_path_data.path_reference().at(i).x(),
                reference_path_data.path_reference().at(i).y()),
            &path_reference_sl);
        path_reference_l[i] = path_reference_sl.l();
      }
      end_state[0] = path_reference_l.back();
      path_data.set_is_optimized_towards_trajectory_reference(true);
      is_valid_path_reference = true;
    }
    */

	vehicle_info_util::VehicleInfoUtil *veh_param = vehicle_info_util::VehicleInfoUtil::get_instance();

    const double lat_acc_bound = 2.0;
        //std::tan(veh_param->max_steer_angle_rad / veh_param->w2s_primary_coeff )/
        //veh_param->wheel_base_m;
    
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
	  
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }

    bool res_opt = OptimizePath(
        init_frenet_state, end_state, std::move(path_reference_l),
        path_reference_size, path_boundary.delta_s(), is_valid_path_reference,
        path_boundary.boundary(), ddl_bounds, w, max_iter, &opt_l, &opt_dl,
        &opt_ddl);
	

    if (res_opt) {
      for (size_t i = 0; i < path_boundary_size; i += 4) {
       // ADEBUG << "for s[" << static_cast<double>(i) * path_boundary.delta_s()
               //<< "], l = " << opt_l[i] << ", dl = " << opt_dl[i];
      }
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s());

      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data.push_back(std::move(path_data));
    }
  }
  if (candidate_path_data.empty()) {
  	  std::cout << "Path Optimizer failed to generate path"<<std::endl;
      return false;
  }
  
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return true;
}


bool PiecewiseJerkPathOptimizer::OptimizePath(
    const std::pair<std::array<double, 3>, std::array<double, 3>>& init_state,
    const std::array<double, 3>& end_state,
    std::vector<double> path_reference_l_ref, const size_t path_reference_size,
    const double delta_s, const bool is_valid_path_reference,
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::vector<std::pair<double, double>>& ddl_bounds,
    const std::array<double, 5>& w, const int max_iter, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx) {

	PlanningConfig *planning_config = PlanningConfig::get_instance();
  // num of knots
  const size_t kNumKnots = lat_boundaries.size();
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
                                                  init_state.second);


  // TODO(Hongyi): update end_state settings
  piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);

  /*
  // pull over scenarios
  // Because path reference might also make the end_state != 0
  // we have to exclude this condition here
  if (end_state[0] != 0 && !is_valid_path_reference) {
    std::vector<double> x_ref(kNumKnots, end_state[0]);
    const auto& pull_over_type = injector_->planning_context()
                                     ->planning_status()
                                     .pull_over()
                                     .pull_over_type();
    const double weight_x_ref =
        pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER ? 200.0 : 10.0;
    piecewise_jerk_problem.set_x_ref(weight_x_ref, std::move(x_ref));
  }
  // use path reference as a optimization cost function
  
  if (is_valid_path_reference) {
    // for non-path-reference part
    // weight_x_ref is set to default value, where
    // l weight = weight_x_ + weight_x_ref_ = (1.0 + 0.0)
    std::vector<double> weight_x_ref_vec(kNumKnots, 0.0);
    // increase l weight for path reference part only

    const double peak_value = config_.piecewise_jerk_path_optimizer_config()
                                  .path_reference_l_weight();
    const double peak_value_x =
        0.5 * static_cast<double>(path_reference_size) * delta_s;
    for (size_t i = 0; i < path_reference_size; ++i) {
      // Gaussian weighting
      const double x = static_cast<double>(i) * delta_s;
      weight_x_ref_vec.at(i) = GaussianWeighting(x, peak_value, peak_value_x);
      ADEBUG << "i: " << i << ", weight: " << weight_x_ref_vec.at(i);
    }
    piecewise_jerk_problem.set_x_ref(std::move(weight_x_ref_vec),
                                     path_reference_l_ref);
  }
  */
  // for debug:here should use std::move
  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);

  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  auto start_time = std::chrono::system_clock::now();

  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(-planning_config->lateral_derivative_bound_default,
                                       planning_config->lateral_derivative_bound_default);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

  // Estimate lat_acc and jerk boundary from vehicle_params
  vehicle_info_util::VehicleInfoUtil *veh_param = vehicle_info_util::VehicleInfoUtil::get_instance();


  const double axis_distance = veh_param->wheel_base_m;
  const double max_yaw_rate = veh_param->steer_rate_lim_dps/180.0 * 3.1415 / veh_param->w2s_primary_coeff;
      //veh_param->steer_rate_lim_dps/180.0 * 3.1415 / veh_param->w2s_primary_coeff / 2.0;

  const double jerk_bound = EstimateJerkBoundary(
                  std::fmax(init_state.first[1], 1.0),
                  axis_distance, max_yaw_rate);
  piecewise_jerk_problem.set_dddx_bound(jerk_bound);

  bool success = piecewise_jerk_problem.Optimize(max_iter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
 // ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
      std::cout  << " piecewise jerk path optimizer failed"<<std::endl;
      std::stringstream ssm;
     // std::cout << "dl bound  " << planning_config->lateral_derivative_bound_default
           //<< " jerk bound  " << jerk_bound<<std::endl;
    
	for (size_t i = 0; i < lat_boundaries.size(); i++) {
      ssm << lat_boundaries[i].first << " " << lat_boundaries[i].second << ","
          << ddl_bounds[i].first << " " << ddl_bounds[i].second << ","
          << path_reference_l_ref[i] << std::endl;
    }
    //std::cout << "lat boundary, ddl boundary , path reference"  << ssm.str()<< std::endl;
    return false;
  }

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  return true;
}

FrenetFramePath PiecewiseJerkPathOptimizer::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) const {
  assert(!x.empty());
  assert(!dx.empty());
  assert(!ddx.empty());
  PlanningConfig *planning_config = PlanningConfig::get_instance();
  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += planning_config->trajectory_space_resolution;
  }

  return FrenetFramePath(std::move(frenet_frame_path));
}

double PiecewiseJerkPathOptimizer::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) const {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

double PiecewiseJerkPathOptimizer::GaussianWeighting(
    const double x, const double peak_weighting,
    const double peak_weighting_x) const {
  double std = 1 / (std::sqrt(2 * M_PI) * peak_weighting);
  double u = peak_weighting_x * std;
  double x_updated = x * std;
  //ADEBUG << peak_weighting *
                //exp(-0.5 * (x - peak_weighting_x) * (x - peak_weighting_x));
  //ADEBUG << Gaussian(u, std, x_updated);
  return Gaussian(u, std, x_updated);
}


/*

TrajectoryPoint
PiecewiseJerkPathOptimizer::InferFrontAxeCenterFromRearAxeCenter(
	const TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
	  VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
	  traj_point.path_point().x() +
	  front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
	  traj_point.path_point().y() +
	  front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

std::vector<common::PathPoint>
PiecewiseJerkPathOptimizer::ConvertPathPointRefFromFrontAxeToRearAxe(
	const PathData& path_data) {
  std::vector<common::PathPoint> ret;
  double front_to_rear_axe_distance =
	  VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  for (auto path_point : path_data.discretized_path()) {
	common::PathPoint new_path_point = path_point;
	new_path_point.set_x(path_point.x() - front_to_rear_axe_distance *
											  std::cos(path_point.theta()));
	new_path_point.set_y(path_point.y() - front_to_rear_axe_distance *
											  std::sin(path_point.theta()));
	ret.push_back(new_path_point);
  }
  return ret;
}
*/
}  // namespace planning
}  // namespace ugv
