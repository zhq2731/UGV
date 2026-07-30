// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trajectory_follower/open_space_mpc_lateral_controller.hpp"

#include "trajectory_follower/common.hpp"
#include "trajectory_follower/qp_solver/qp_solver_osqp.hpp"
#include "trajectory_follower/qp_solver/qp_solver_unconstr_fast.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{

OpenSpaceMpcLateralController::OpenSpaceMpcLateralController(
  ros::NodeHandle & nh)
: node_(&nh),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil::get_instance())
{
  // 泊车控制器拥有自己的 MPC 对象和全部历史状态。当前控制周期与原控制器
  // 保持一致，先完成无行为变化的代码隔离，后续再单独标定泊车控制周期。
  mpc_.m_ctrl_period = 0.03;

  node_->param<bool>("enable_path_smoothing", enable_path_smoothing_, false);
  node_->param<int>(
    "path_filter_moving_ave_num", path_filter_moving_average_num_, 25);
  node_->param<int>(
    "curvature_smoothing_num_traj",
    trajectory_curvature_smoothing_num_, 15);
  node_->param<int>(
    "curvature_smoothing_num_ref_steer",
    reference_steer_curvature_smoothing_num_, 15);
  node_->param<double>(
    "traj_resample_dist", trajectory_resample_distance_, 0.1);

  node_->param<double>(
    "admissible_position_error", mpc_.m_admissible_position_error, 5.0);
  node_->param<double>(
    "admissible_yaw_error_rad", mpc_.m_admissible_yaw_error_rad, 1.57);
  node_->param<bool>(
    "use_steer_prediction", mpc_.m_use_steer_prediction, false);
  node_->param<double>(
    "vehicle_model_steer_tau", mpc_.m_param.steer_tau, 0.005);

  node_->param<double>(
    "stop_state_entry_ego_speed", stop_state_entry_ego_speed_, 0.001);
  node_->param<double>(
    "stop_state_entry_target_speed", stop_state_entry_target_speed_, 0.001);
  node_->param<double>("converged_steer_rad", converged_steer_rad_, 0.1);
  node_->param<bool>(
    "keep_steer_control_until_converged",
    keep_steer_control_until_converged_, true);
  node_->param<double>(
    "new_traj_end_dist", new_trajectory_end_distance_, 0.3);

  constexpr double kDegreesToRadians = M_PI / 180.0;
  mpc_.m_steer_lim =
    vehicle_info_->max_steer_angle_rad / vehicle_info_->w2s_primary_coeff;
  mpc_.m_steer_rate_lim =
    vehicle_info_->steer_rate_lim_dps * kDegreesToRadians /
    vehicle_info_->w2s_primary_coeff;

  const auto vehicle_model =
    std::make_shared<KinematicsBicycleModel>(
      vehicle_info_->wheel_base_m, mpc_.m_steer_lim, mpc_.m_param.steer_tau);

  std::string qp_solver_type;
  node_->param<std::string>("qp_solver_type", qp_solver_type, "osqp");
  std::shared_ptr<QPSolverInterface> qp_solver;
  if (qp_solver_type == "unconstraint_fast") {
    qp_solver = std::make_shared<QPSolverEigenLeastSquareLLT>();
  } else {
    // 与原参考线控制器保持相同的兼容行为：未知配置也回退到 OSQP，
    // 避免泊车控制器因拼写错误而留下空求解器。
    if (qp_solver_type != "osqp") {
      ROS_WARN_STREAM(
        "[open_space_mpc] unknown qp_solver_type '" << qp_solver_type
        << "', fallback to osqp");
    }
    qp_solver = std::make_shared<QPSolverOSQP>();
  }

  double input_delay = 0.0;
  node_->param<double>("input_delay", input_delay, 0.0);
  const double delay_steps = std::round(input_delay / mpc_.m_ctrl_period);
  mpc_.m_param.input_delay = delay_steps * mpc_.m_ctrl_period;
  mpc_.m_input_buffer =
    std::deque<double>(static_cast<size_t>(delay_steps), 0.0);

  double steering_lpf_cutoff_hz = 0.0;
  double error_derivative_lpf_cutoff_hz = 0.0;
  node_->param<double>(
    "steering_lpf_cutoff_hz", steering_lpf_cutoff_hz, 0.0);
  node_->param<double>(
    "error_deriv_lpf_cutoff_hz", error_derivative_lpf_cutoff_hz, 0.0);
  mpc_.initializeLowPassFilters(
    steering_lpf_cutoff_hz, error_derivative_lpf_cutoff_hz);

  node_->param<double>(
    "ego_nearest_dist_threshold", ego_nearest_distance_threshold_, 0.0);
  node_->param<double>(
    "ego_nearest_yaw_threshold", ego_nearest_yaw_threshold_, 0.0);
  mpc_.ego_nearest_dist_threshold = ego_nearest_distance_threshold_;
  mpc_.ego_nearest_yaw_threshold = ego_nearest_yaw_threshold_;

  loadMpcParameters();
  mpc_.setQPSolver(qp_solver);
  mpc_.setVehicleModel(vehicle_model, "kinematics");
}

boost::optional<LateralOutput> OpenSpaceMpcLateralController::run()
{
  if (!checkData()) {
    return boost::none;
  }

  autoware_msgs::AckermannLateralCommand command;
  autoware_msgs::TrajectoryPointArray predicted_trajectory;

  // 首次执行使用底盘实际转角建立控制历史，避免默认零转角造成首帧跳变。
  if (!previous_control_initialized_) {
    previous_control_command_ = getInitialControlCommand();
    previous_control_initialized_ = true;
  }

  MPCData mpc_data;
  const bool solved = mpc_.calculateMPC(
    *current_steering_, current_odometry_->twist.twist.linear.x,
    current_odometry_->pose.pose, command, predicted_trajectory, mpc_data);
  mpc_data.current_pose = current_odometry_->pose.pose;

  const auto make_output =
    [this, &mpc_data](const autoware_msgs::AckermannLateralCommand & cmd) {
      LateralOutput output;
      output.lat_error = mpc_data.lateral_err;
      output.yaw_error = mpc_data.yaw_err;
      output.pose = mpc_data.current_pose;
      output.nearestPose = mpc_data.nearest_pose;
      output.control_cmd = createControlCommand(cmd);
      output.sync_data.is_steer_converged = isSteerConverged(cmd);
      return boost::optional<LateralOutput>(output);
    };

  if (isStoppedState()) {
    // 车辆和参考点均为零速时保留末端转角，防止停车等待期间横向命令跳变。
    for (auto & buffered_steer : mpc_.m_input_buffer) {
      buffered_steer = previous_control_command_.steering_tire_angle;
    }
    mpc_.m_raw_steer_cmd_prev =
      previous_control_command_.steering_tire_angle;
    return make_output(previous_control_command_);
  }

  if (!solved) {
    command = getStopControlCommand();
  }
  previous_control_command_ = command;
  return make_output(command);
}

void OpenSpaceMpcLateralController::setInputData(
  InputData const & input_data)
{
  setTrajectory(input_data.current_trajectory_ptr);
  current_odometry_ = input_data.current_odometry_ptr;
  current_steering_ = input_data.current_steering_ptr;
  if (current_odometry_) {
    current_odometry_->twist.twist.linear.x = input_data.vel;
  }
}

void OpenSpaceMpcLateralController::resetForNewTrajectory(
  const autoware_msgs::SteeringReport & current_steer)
{
  // 新段开始时保留车辆真实前轮角，只清除上一段控制器内部“记忆”。
  previous_control_command_.steering_tire_angle =
    current_steer.steering_tire_angle;
  previous_control_command_.steering_tire_rotation_rate = 0.0;
  previous_steer_command_ = current_steer.steering_tire_angle;
  previous_control_initialized_ = true;
  trajectory_buffer_.clear();
  mpc_.resetForOpenSpaceTrajectory(current_steer);
}

void OpenSpaceMpcLateralController::setTrajectory(
  const autoware_msgs::TrajectoryPointArray::Ptr & trajectory)
{
  if (!trajectory) {
    return;
  }
  current_trajectory_ = trajectory;
  if (!current_odometry_ || trajectory->points.size() < 3 ||
    !isValidTrajectory(*trajectory))
  {
    return;
  }

  mpc_.setReferenceTrajectory(
    *trajectory, trajectory_resample_distance_, enable_path_smoothing_,
    path_filter_moving_average_num_, trajectory_curvature_smoothing_num_,
    reference_steer_curvature_smoothing_num_);

  trajectory_buffer_.push_back(*trajectory);
}

bool OpenSpaceMpcLateralController::checkData() const
{
  return mpc_.hasVehicleModel() && mpc_.hasQPSolver() && current_odometry_ &&
    current_steering_ && mpc_.m_ref_traj.size() > 3;
}

bool OpenSpaceMpcLateralController::isValidTrajectory(
  const autoware_msgs::TrajectoryPointArray & trajectory) const
{
  for (const auto & point : trajectory.points) {
    if (!std::isfinite(point.pose.position.x) ||
      !std::isfinite(point.pose.position.y) ||
      !std::isfinite(point.pose.orientation.w) ||
      !std::isfinite(point.pose.orientation.x) ||
      !std::isfinite(point.pose.orientation.y) ||
      !std::isfinite(point.pose.orientation.z) ||
      !std::isfinite(point.longitudinal_velocity_mps))
    {
      return false;
    }
  }
  return true;
}

bool OpenSpaceMpcLateralController::isStoppedState() const
{
  if (!current_trajectory_ || current_trajectory_->points.empty()) {
    return false;
  }

  const size_t nearest = findFirstNearestIndexWithSoftConstraints(
    current_trajectory_->points, current_odometry_->pose.pose,
    ego_nearest_distance_threshold_, ego_nearest_yaw_threshold_);
  const double current_velocity = current_odometry_->twist.twist.linear.x;
  const double target_velocity =
    current_trajectory_->points.at(nearest).longitudinal_velocity_mps;

  if (keep_steer_control_until_converged_ &&
    !isSteerConverged(previous_control_command_))
  {
    return false;
  }
  return std::fabs(current_velocity) < stop_state_entry_ego_speed_ &&
    std::fabs(target_velocity) < stop_state_entry_target_speed_;
}

bool OpenSpaceMpcLateralController::isSteerConverged(
  const autoware_msgs::AckermannLateralCommand & command) const
{
  if (!received_first_trajectory_ || isTrajectoryShapeChanged()) {
    return false;
  }
  return std::fabs(
    command.steering_tire_angle - current_steering_->steering_tire_angle) <
    converged_steer_rad_;
}

bool OpenSpaceMpcLateralController::isTrajectoryShapeChanged() const
{
  if (!current_trajectory_ || current_trajectory_->points.empty()) {
    return false;
  }
  for (const auto & trajectory : trajectory_buffer_) {
    if (!trajectory.points.empty() &&
      calcDistance2d(
        trajectory.points.back().pose,
        current_trajectory_->points.back().pose) >
      new_trajectory_end_distance_)
    {
      return true;
    }
  }
  return false;
}

autoware_msgs::AckermannLateralCommand
OpenSpaceMpcLateralController::createControlCommand(
  autoware_msgs::AckermannLateralCommand command)
{
  command.header.stamp = ros::Time::now();
  previous_steer_command_ = command.steering_tire_angle;
  return command;
}

autoware_msgs::AckermannLateralCommand
OpenSpaceMpcLateralController::getStopControlCommand() const
{
  autoware_msgs::AckermannLateralCommand command;
  command.steering_tire_angle = previous_steer_command_;
  command.steering_tire_rotation_rate = 0.0;
  return command;
}

autoware_msgs::AckermannLateralCommand
OpenSpaceMpcLateralController::getInitialControlCommand() const
{
  autoware_msgs::AckermannLateralCommand command;
  command.steering_tire_angle = current_steering_->steering_tire_angle;
  command.steering_tire_rotation_rate = 0.0;
  return command;
}

void OpenSpaceMpcLateralController::loadMpcParameters()
{
  node_->param<int>(
    "mpc_prediction_horizon", mpc_.m_param.prediction_horizon, 50);
  node_->param<double>(
    "mpc_prediction_dt", mpc_.m_param.prediction_dt, 0.1);
  node_->param<double>(
    "mpc_weight_lat_error", mpc_.m_param.weight_lat_error, 0.5);
  node_->param<double>(
    "mpc_weight_heading_error", mpc_.m_param.weight_heading_error, 0.0);
  node_->param<double>(
    "mpc_weight_heading_error_squared_vel",
    mpc_.m_param.weight_heading_error_squared_vel, 0.3);
  node_->param<double>(
    "mpc_weight_steering_input", mpc_.m_param.weight_steering_input, 1.0);
  node_->param<double>(
    "mpc_weight_steering_input_squared_vel",
    mpc_.m_param.weight_steering_input_squared_vel, 0.25);
  node_->param<double>(
    "mpc_weight_lat_jerk", mpc_.m_param.weight_lat_jerk, 0.0);
  node_->param<double>(
    "mpc_weight_steer_rate", mpc_.m_param.weight_steer_rate, 0.0);
  node_->param<double>(
    "mpc_weight_steer_acc", mpc_.m_param.weight_steer_acc, 0.000001);

  node_->param<double>(
    "mpc_low_curvature_weight_lat_error",
    mpc_.m_param.low_curvature_weight_lat_error, 0.5);
  node_->param<double>(
    "mpc_low_curvature_weight_heading_error",
    mpc_.m_param.low_curvature_weight_heading_error, 0.0);
  node_->param<double>(
    "mpc_low_curvature_weight_heading_error_squared_vel",
    mpc_.m_param.low_curvature_weight_heading_error_squared_vel, 0.3);
  node_->param<double>(
    "mpc_low_curvature_weight_steering_input",
    mpc_.m_param.low_curvature_weight_steering_input, 1.0);
  node_->param<double>(
    "mpc_low_curvature_weight_steering_input_squared_vel",
    mpc_.m_param.low_curvature_weight_steering_input_squared_vel, 0.25);
  node_->param<double>(
    "mpc_low_curvature_weight_lat_jerk",
    mpc_.m_param.low_curvature_weight_lat_jerk, 0.0);
  node_->param<double>(
    "mpc_low_curvature_weight_steer_rate",
    mpc_.m_param.low_curvature_weight_steer_rate, 0.0);
  node_->param<double>(
    "mpc_low_curvature_weight_steer_acc",
    mpc_.m_param.low_curvature_weight_steer_acc, 0.000001);
  node_->param<double>(
    "mpc_low_curvature_thresh_curvature",
    mpc_.m_param.low_curvature_thresh_curvature, 0.0);

  node_->param<double>(
    "mpc_weight_terminal_lat_error",
    mpc_.m_param.weight_terminal_lat_error, 1.0);
  node_->param<double>(
    "mpc_weight_terminal_heading_error",
    mpc_.m_param.weight_terminal_heading_error, 0.1);
  node_->param<double>(
    "mpc_zero_ff_steer_deg", mpc_.m_param.zero_ff_steer_deg, 0.5);
  node_->param<double>(
    "mpc_acceleration_limit", mpc_.m_param.acceleration_limit, 2.0);
  node_->param<double>(
    "mpc_velocity_time_constant",
    mpc_.m_param.velocity_time_constant, 0.3);
  node_->param<double>(
    "mpc_min_prediction_length",
    mpc_.m_param.min_prediction_length, 5.0);
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
