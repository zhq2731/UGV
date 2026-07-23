// Copyright 2021 The Autoware Foundation
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

#include "trajectory_follower/mpc_lateral_controller.hpp"


#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
namespace
{
using namespace std::literals::chrono_literals;


}  // namespace

MpcLateralController::MpcLateralController(ros::NodeHandle &nh) : node_{&nh}
{

  vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();

  using std::placeholders::_1;
  m_mpc.m_ctrl_period = 0.03;

  node_->param<bool>("enable_path_smoothing", m_enable_path_smoothing, "false");
  node_->param<int>("path_filter_moving_ave_num", m_path_filter_moving_ave_num, 25);
  node_->param<int>("curvature_smoothing_num_traj", m_curvature_smoothing_num_traj, 15);
  node_->param<int>("curvature_smoothing_num_ref_steer", m_curvature_smoothing_num_ref_steer, 15);
  node_->param<double>("traj_resample_dist", m_traj_resample_dist, 0.1);

  
  node_->param<double>("admissible_position_error", m_mpc.m_admissible_position_error, 5.0);
  node_->param<double>("admissible_yaw_error_rad",  m_mpc.m_admissible_yaw_error_rad, 1.57);

  
  node_->param<bool>("use_steer_prediction",  m_mpc.m_use_steer_prediction, false);
  node_->param<double>("vehicle_model_steer_tau",  m_mpc.m_param.steer_tau, 0.005);
  

  node_->param<double>("stop_state_entry_ego_speed", m_stop_state_entry_ego_speed, 0.001);
  node_->param<double>("stop_state_entry_target_speed", m_stop_state_entry_target_speed, 0.001);
  node_->param<double>("converged_steer_rad", m_converged_steer_rad, 0.1);
  node_->param<bool>("keep_steer_control_until_converged", m_keep_steer_control_until_converged, true);
  node_->param<double>("new_traj_duration_time", m_new_traj_duration_time, 1.0);
  node_->param<double>("new_traj_end_dist", m_new_traj_end_dist, 0.3);


  constexpr double deg2rad = static_cast<double>(M_PI) / 180.0;
  m_mpc.m_steer_lim = vehcileInfo->max_steer_angle_rad / vehcileInfo->w2s_primary_coeff;
  m_mpc.m_steer_rate_lim = vehcileInfo->steer_rate_lim_dps  * deg2rad/vehcileInfo->w2s_primary_coeff;
	
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr;
    vehicle_model_ptr = std::make_shared<trajectory_follower::KinematicsBicycleModel>(
      vehcileInfo->wheel_base_m, m_mpc.m_steer_lim, m_mpc.m_param.steer_tau);
  std::string qp_solver_type;
  node_->param<std::string>("qp_solver_type", qp_solver_type, "osqp");
  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr;
  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr = std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  } else if (qp_solver_type == "osqp") {
    qpsolver_ptr = std::make_shared<trajectory_follower::QPSolverOSQP>();
  } else {
    //RCLCPP_ERROR(node_->get_logger(), "qp_solver_type is undefined");
  }


  /* delay compensation */
  {
        double delay_tmp;
		node_->param<double>("input_delay", delay_tmp, 0.0);
        const double delay_step = std::round(delay_tmp / m_mpc.m_ctrl_period);
        m_mpc.m_param.input_delay = delay_step * m_mpc.m_ctrl_period;
        m_mpc.m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
  }

  
  /* initialize lowpass filter */
  {
    
     double steering_lpf_cutoff_hz ;
     double error_deriv_lpf_cutoff_hz 	;
	node_->param<double>("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz, 0.0);
	node_->param<double>("error_deriv_lpf_cutoff_hz", error_deriv_lpf_cutoff_hz, 0.0);
    m_mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);

  }


  node_->param<double>("ego_nearest_dist_threshold", m_ego_nearest_dist_threshold, 0.0);
  
  node_->param<double>("ego_nearest_yaw_threshold", m_ego_nearest_yaw_threshold, 0.0);


  m_mpc.ego_nearest_dist_threshold = m_ego_nearest_dist_threshold;
  m_mpc.ego_nearest_yaw_threshold = m_ego_nearest_yaw_threshold;

  // TODO(Frederik.Beaujean) ctor is too long, should factor out parameter declarations
  declareMPCparameters();

  m_mpc.setQPSolver(qpsolver_ptr);
  m_mpc.setVehicleModel(vehicle_model_ptr, "kinematics");

  //m_mpc.setLogger(node_->get_logger());
  //m_mpc.setClock(node_->get_clock());
}



MpcLateralController::~MpcLateralController() 
{
}


boost::optional<LateralOutput> MpcLateralController::run()
{
  if (!checkData()) {
      return boost::none;
  }

  autoware_msgs::AckermannLateralCommand ctrl_cmd;
  autoware_msgs::TrajectoryPointArray predicted_traj;
  //tier4_debug_msgs::msg::Float32MultiArrayStamped debug_values;

  //程序启动的时候，获取当前的方向盘转角，做为上一次的指令
  if (!m_is_ctrl_cmd_prev_initialized) {
    m_ctrl_cmd_prev = getInitialControlCommand();
    m_is_ctrl_cmd_prev_initialized = true;
  }
  MPCData  mpc_data;
  const bool is_mpc_solved = m_mpc.calculateMPC(
    *m_current_steering_ptr, m_current_kinematic_state_ptr->twist.twist.linear.x,
    m_current_kinematic_state_ptr->pose.pose, ctrl_cmd, predicted_traj,mpc_data);

  mpc_data.current_pose =  m_current_kinematic_state_ptr->pose.pose;
  
  const auto createLateralOutput = [this](const auto & cmd,const auto &mpc_data) {
    LateralOutput output;
	output.lat_error = mpc_data.lateral_err;
	output.yaw_error = mpc_data.yaw_err;
	output.pose = mpc_data.current_pose;
	output.nearestPose = mpc_data.nearest_pose;
	
    output.control_cmd = createCtrlCmdMsg(cmd);
    output.sync_data.is_steer_converged = isSteerConverged(cmd);
    return boost::optional<LateralOutput>(output);
  };

  if (isStoppedState()) {
    // Reset input buffer
    for (auto & value : m_mpc.m_input_buffer) {
      value = m_ctrl_cmd_prev.steering_tire_angle;
    }
    // Use previous command value as previous raw steer command
    m_mpc.m_raw_steer_cmd_prev = m_ctrl_cmd_prev.steering_tire_angle;
    return createLateralOutput(m_ctrl_cmd_prev,mpc_data);
  }
  
  if (!is_mpc_solved) {
    //RCLCPP_WARN_SKIPFIRST_THROTTLE(
      //node_->get_logger(), *node_->get_clock(), 5000 /*ms*/,
      //"MPC is not solved. publish 0 velocity.");
      ctrl_cmd = getStopControlCommand();
  }

  m_ctrl_cmd_prev = ctrl_cmd;
  return createLateralOutput(ctrl_cmd,mpc_data);
}

void MpcLateralController::setInputData(InputData const & input_data)
{
  setTrajectory(input_data.current_trajectory_ptr);
  m_current_kinematic_state_ptr = input_data.current_odometry_ptr;
  m_current_steering_ptr = input_data.current_steering_ptr;
  if (m_current_kinematic_state_ptr)
      m_current_kinematic_state_ptr->twist.twist.linear.x = input_data.vel;
}

void MpcLateralController::resetForOpenSpaceTrajectory(
  const autoware_msgs::SteeringReport & current_steer)
{
  m_ctrl_cmd_prev.steering_tire_angle = current_steer.steering_tire_angle;
  m_ctrl_cmd_prev.steering_tire_rotation_rate = 0.0;
  m_steer_cmd_prev = current_steer.steering_tire_angle;
  m_is_ctrl_cmd_prev_initialized = true;
  m_trajectory_buffer.clear();
  m_mpc.resetForOpenSpaceTrajectory(current_steer);
}

bool MpcLateralController::isSteerConverged(
  const autoware_msgs::AckermannLateralCommand & cmd) const
{
  // wait for a while to propagate the trajectory shape to the output command when the trajectory
  // shape is changed.
  if (!m_has_received_first_trajectory || isTrajectoryShapeChanged()) {
    return false;
  }

  const bool is_converged =
    std::abs(cmd.steering_tire_angle - m_current_steering_ptr->steering_tire_angle) <
    static_cast<float>(m_converged_steer_rad);
  return is_converged;
}

bool MpcLateralController::checkData() const
{
  if (!m_mpc.hasVehicleModel()) {
    //RCLCPP_DEBUG(node_->get_logger(), "MPC does not have a vehicle model");
    return false;
  }
  if (!m_mpc.hasQPSolver()) {
    //RCLCPP_DEBUG(node_->get_logger(), "MPC does not have a QP solver");
    return false;
  }

  if (!m_current_kinematic_state_ptr) {
    //RCLCPP_DEBUG(
      //node_->get_logger(), "waiting data. kinematic_state = %d",
      //m_current_kinematic_state_ptr != nullptr);
    return false;
  }

  if (!m_current_steering_ptr) {
    //RCLCPP_DEBUG(
      //node_->get_logger(), "waiting data. current_steering = %d",
      //m_current_steering_ptr != nullptr);
    return false;
  }

  if (m_mpc.m_ref_traj.size() <= 3) {
  	//std::cout <<"trajectory size is zero."<<std::endl;
    //std(node_->get_logger(), "trajectory size is zero.");
    return false;
  }

  return true;
}

void MpcLateralController::setTrajectory(
  const autoware_msgs::TrajectoryPointArray::Ptr msg)
{
  if (!msg) return;

  m_current_trajectory_ptr = msg;

  if (!m_current_kinematic_state_ptr) {
    //RCLCPP_DEBUG(node_->get_logger(), "Current kinematic state is not received yet.");
    return;
  }

  if (msg->points.size() < 3) {
    //RCLCPP_DEBUG(node_->get_logger(), "received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(*msg)) {
   // RCLCPP_ERROR(node_->get_logger(), "Trajectory is invalid!! stop computing.");
    return;
  }

  m_mpc.setReferenceTrajectory(
    *msg, m_traj_resample_dist, m_enable_path_smoothing, m_path_filter_moving_ave_num,
    m_curvature_smoothing_num_traj, m_curvature_smoothing_num_ref_steer);

  // update trajectory buffer to check the trajectory shape change.
  m_trajectory_buffer.push_back(*m_current_trajectory_ptr);
  //while (rclcpp::ok()) 
}

autoware_msgs::AckermannLateralCommand
MpcLateralController::getStopControlCommand() const
{
  autoware_msgs::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = static_cast<decltype(cmd.steering_tire_angle)>(m_steer_cmd_prev);
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

autoware_msgs::AckermannLateralCommand
MpcLateralController::getInitialControlCommand() const
{
  autoware_msgs::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = m_current_steering_ptr->steering_tire_angle;
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

bool MpcLateralController::isStoppedState() const
{
  // If the nearest index is not found, return false
  if (m_current_trajectory_ptr->points.empty()) {
    return false;
  }

  // Note: This function used to take into account the distance to the stop line
  // for the stop state judgement. However, it has been removed since the steering
  // control was turned off when approaching/exceeding the stop line on a curve or
  // emergency stop situation and it caused large tracking error.
  const size_t nearest = findFirstNearestIndexWithSoftConstraints(
    m_current_trajectory_ptr->points, m_current_kinematic_state_ptr->pose.pose,
    m_ego_nearest_dist_threshold, m_ego_nearest_yaw_threshold);

  const double current_vel = m_current_kinematic_state_ptr->twist.twist.linear.x;
  const double target_vel =
    m_current_trajectory_ptr->points.at(static_cast<size_t>(nearest)).longitudinal_velocity_mps;

  const auto latest_published_cmd = m_ctrl_cmd_prev;  // use prev_cmd as a latest published command
  if (m_keep_steer_control_until_converged && !isSteerConverged(latest_published_cmd)) {
    return false;  // not stopState: keep control
  }

  if (
    std::fabs(current_vel) < m_stop_state_entry_ego_speed &&
    std::fabs(target_vel) < m_stop_state_entry_target_speed) {
    return true;
  } else {
    return false;
  }
}

autoware_msgs::AckermannLateralCommand MpcLateralController::createCtrlCmdMsg(
  autoware_msgs::AckermannLateralCommand ctrl_cmd)
{
  ctrl_cmd.header.stamp = ros::Time::now();

  m_steer_cmd_prev = ctrl_cmd.steering_tire_angle;
  return ctrl_cmd;
}



void MpcLateralController::declareMPCparameters()
{

  node_->param<int>("mpc_prediction_horizon", m_mpc.m_param.prediction_horizon, 50);
  node_->param<double>("mpc_prediction_dt", m_mpc.m_param.prediction_dt, 0.1);
  node_->param<double>("mpc_weight_lat_error", m_mpc.m_param.weight_lat_error, 0.5);
  node_->param<double>("mpc_weight_heading_error", m_mpc.m_param.weight_heading_error, 0.0);

  node_->param<double>("mpc_weight_heading_error_squared_vel", m_mpc.m_param.weight_heading_error_squared_vel, 0.3);
  node_->param<double>("mpc_weight_steering_input", m_mpc.m_param.weight_steering_input, 1.0);
  node_->param<double>("mpc_weight_steering_input_squared_vel", m_mpc.m_param.weight_steering_input_squared_vel, 0.25);


  node_->param<double>("mpc_weight_lat_jerk", m_mpc.m_param.weight_lat_jerk, 0.0);
  node_->param<double>("mpc_weight_steer_rate", m_mpc.m_param.weight_steer_rate, 0.0);
  node_->param<double>("mpc_weight_steer_acc", m_mpc.m_param.weight_steer_acc, 0.000001);

  node_->param<double>("mpc_low_curvature_weight_lat_error", m_mpc.m_param.low_curvature_weight_lat_error, 0.5);
  node_->param<double>("mpc_low_curvature_weight_heading_error", m_mpc.m_param.low_curvature_weight_heading_error, 0.0);
  node_->param<double>("mpc_low_curvature_weight_heading_error_squared_vel", m_mpc.m_param.low_curvature_weight_heading_error_squared_vel, 0.3);

  node_->param<double>("mpc_low_curvature_weight_steering_input", m_mpc.m_param.low_curvature_weight_steering_input, 1.0);
  node_->param<double>("mpc_low_curvature_weight_steering_input_squared_vel", m_mpc.m_param.low_curvature_weight_steering_input_squared_vel, 0.25);
  node_->param<double>("mpc_low_curvature_weight_lat_jerk", m_mpc.m_param.low_curvature_weight_lat_jerk, 0.0);

  node_->param<double>("mpc_low_curvature_weight_steer_rate", m_mpc.m_param.low_curvature_weight_steer_rate, 0.0);
  node_->param<double>("mpc_low_curvature_weight_steer_acc", m_mpc.m_param.low_curvature_weight_steer_acc, 0.000001);
  node_->param<double>("mpc_low_curvature_thresh_curvature", m_mpc.m_param.low_curvature_thresh_curvature, 0.0);

  node_->param<double>("mpc_weight_terminal_lat_error", m_mpc.m_param.weight_terminal_lat_error, 1.0);
  node_->param<double>("mpc_weight_terminal_heading_error", m_mpc.m_param.weight_terminal_heading_error, 0.1);
  node_->param<double>("mpc_zero_ff_steer_deg", m_mpc.m_param.zero_ff_steer_deg, 0.5);

  node_->param<double>("mpc_acceleration_limit", m_mpc.m_param.acceleration_limit, 2.0);
  node_->param<double>("mpc_velocity_time_constant", m_mpc.m_param.velocity_time_constant, 0.3);
  node_->param<double>("mpc_min_prediction_length", m_mpc.m_param.min_prediction_length, 5.0);

}



bool MpcLateralController::isTrajectoryShapeChanged() const
{
  // TODO(Horibe): update implementation to check trajectory shape around ego vehicle.
  // Now temporally check the goal position.
  for (const auto & trajectory : m_trajectory_buffer) {
    if (
      calcDistance2d(
        trajectory.points.back().pose, m_current_trajectory_ptr->points.back().pose) >
      m_new_traj_end_dist) {
      return true;
    }
  }
  return false;
}

bool MpcLateralController::isValidTrajectory(
  const autoware_msgs::TrajectoryPointArray & traj) const
{
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.pose.position.x) || !isfinite(p.pose.position.y) ||
      !isfinite(p.pose.orientation.w) || !isfinite(p.pose.orientation.x) ||
      !isfinite(p.pose.orientation.y) || !isfinite(p.pose.orientation.z) ||
      !isfinite(p.longitudinal_velocity_mps)) {
      return false;
    }
  }
  return true;
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
