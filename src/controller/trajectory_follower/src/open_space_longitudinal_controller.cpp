#include "trajectory_follower/open_space_longitudinal_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

double clampValue(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double pointDistance(
  const planning_msgs::TrajectoryPoint &first,
  const planning_msgs::TrajectoryPoint &second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

double jerkLimitedStoppingDistance(
  double speed, double max_deceleration, double max_jerk)
{
  // 制动力需要从零按jerk逐步建立：先计算减速度爬升段，再计算恒减速度段。
  // 该距离用于决定何时开始制动，避免仍按“减速度瞬间达到上限”估算而越过终点。
  speed = std::fabs(speed);
  if (speed <= 1.0e-6) {
    return 0.0;
  }
  if (max_jerk <= 1.0e-6) {
    return speed * speed / (2.0 * max_deceleration);
  }

  const double jerk_ramp_time = max_deceleration / max_jerk;
  const double ramp_velocity_reduction =
    0.5 * max_jerk * jerk_ramp_time * jerk_ramp_time;
  if (speed <= ramp_velocity_reduction) {
    const double stop_time = std::sqrt(2.0 * speed / max_jerk);
    return speed * stop_time -
      max_jerk * stop_time * stop_time * stop_time / 6.0;
  }

  const double speed_after_ramp = speed - ramp_velocity_reduction;
  const double ramp_distance =
    speed * jerk_ramp_time -
    max_jerk * jerk_ramp_time * jerk_ramp_time * jerk_ramp_time / 6.0;
  return ramp_distance +
    speed_after_ramp * speed_after_ramp / (2.0 * max_deceleration);
}

}  // namespace

OpenSpaceLongitudinalController::OpenSpaceLongitudinalController(
  const ros::NodeHandle &node)
{
  // 仿真/实车输出方式由泊车纵控内部读取，控制节点只负责场景门控。
  std::string output_mode;
  node.param<std::string>(
    "open_space_longitudinal_control_mode", output_mode, "vehicle");
  output_mode_ = parseOutputMode(output_mode);

  node.param<double>(
    "open_space_lookahead_distance", lookahead_distance_, 0.5);
  node.param<double>(
    "open_space_forward_speed_kp", forward_speed_kp_, 1.5);
  node.param<double>(
    "open_space_forward_speed_ki", forward_speed_ki_, 0.0);
  node.param<double>(
    "open_space_reverse_speed_kp", reverse_speed_kp_, 1.5);
  node.param<double>(
    "open_space_reverse_speed_ki", reverse_speed_ki_, 0.0);
  node.param<double>(
    "open_space_speed_integral_limit", speed_integral_limit_, 0.5);
  node.param<double>(
    "open_space_max_acceleration", max_acceleration_, 0.5);
  node.param<double>(
    "open_space_max_deceleration", max_deceleration_, 1.0);
  node.param<double>(
    "open_space_max_jerk", max_jerk_, 2.0);
  node.param<double>(
    "open_space_longitudinal_control_period", nominal_control_period_, 0.02);
  // 先读取旧的共用参数作为兼容默认值，再允许车型配置按方向覆盖。
  double legacy_throttle_gain = 3.0;
  double legacy_brake_gain = 6.0;
  double legacy_rolling_resistance = 0.15;
  node.param<double>(
    "open_space_carla_throttle_acceleration_gain",
    legacy_throttle_gain, legacy_throttle_gain);
  node.param<double>(
    "open_space_carla_brake_deceleration_gain",
    legacy_brake_gain, legacy_brake_gain);
  node.param<double>(
    "open_space_carla_rolling_resistance",
    legacy_rolling_resistance, legacy_rolling_resistance);
  node.param<double>(
    "open_space_carla_forward_throttle_acceleration_gain",
    carla_forward_throttle_acceleration_gain_, legacy_throttle_gain);
  node.param<double>(
    "open_space_carla_reverse_throttle_acceleration_gain",
    carla_reverse_throttle_acceleration_gain_, legacy_throttle_gain);
  node.param<double>(
    "open_space_carla_forward_throttle_offset",
    carla_forward_throttle_offset_, legacy_rolling_resistance);
  node.param<double>(
    "open_space_carla_reverse_throttle_offset",
    carla_reverse_throttle_offset_, legacy_rolling_resistance);
  node.param<double>(
    "open_space_carla_forward_brake_deceleration_gain",
    carla_forward_brake_deceleration_gain_, legacy_brake_gain);
  node.param<double>(
    "open_space_carla_reverse_brake_deceleration_gain",
    carla_reverse_brake_deceleration_gain_, legacy_brake_gain);
  node.param<double>(
    "open_space_carla_forward_brake_offset",
    carla_forward_brake_offset_, legacy_rolling_resistance);
  node.param<double>(
    "open_space_carla_reverse_brake_offset",
    carla_reverse_brake_offset_, legacy_rolling_resistance);
  node.param<double>(
    "open_space_carla_acceleration_deadband",
    carla_acceleration_deadband_, 0.02);
  node.param<double>(
    "open_space_carla_stop_brake_pedal",
    carla_stop_brake_pedal_, 20.0);
  node.param<double>(
    "open_space_stop_speed_tolerance", stop_speed_tolerance_, 0.05);
  node.param<double>(
    "open_space_carla_reverse_coast_deceleration",
    reverse_coast_deceleration_, 1.8);

  lookahead_distance_ = std::max(0.0, lookahead_distance_);
  speed_integral_limit_ = std::max(0.0, speed_integral_limit_);
  max_acceleration_ = std::max(1.0e-3, max_acceleration_);
  max_deceleration_ = std::max(1.0e-3, max_deceleration_);
  max_jerk_ = std::max(0.0, max_jerk_);
  nominal_control_period_ = std::max(1.0e-3, nominal_control_period_);
  carla_forward_throttle_acceleration_gain_ =
    std::max(1.0e-3, carla_forward_throttle_acceleration_gain_);
  carla_reverse_throttle_acceleration_gain_ =
    std::max(1.0e-3, carla_reverse_throttle_acceleration_gain_);
  carla_forward_throttle_offset_ =
    std::max(0.0, carla_forward_throttle_offset_);
  carla_reverse_throttle_offset_ =
    std::max(0.0, carla_reverse_throttle_offset_);
  carla_forward_brake_deceleration_gain_ =
    std::max(1.0e-3, carla_forward_brake_deceleration_gain_);
  carla_reverse_brake_deceleration_gain_ =
    std::max(1.0e-3, carla_reverse_brake_deceleration_gain_);
  carla_acceleration_deadband_ = std::max(0.0, carla_acceleration_deadband_);
  carla_stop_brake_pedal_ =
    clampValue(carla_stop_brake_pedal_, 0.0, 100.0);
  stop_speed_tolerance_ = std::max(0.0, stop_speed_tolerance_);
  reverse_coast_deceleration_ = std::max(0.1, reverse_coast_deceleration_);

  ROS_INFO_STREAM("[open_space_lon] output mode: "
    << outputModeName(output_mode_));
}

bool OpenSpaceLongitudinalController::Compute(
  const car::control::InputData &input,
  driver_msgs::DriveCmd *command,
  double *remaining_distance)
{
  if (command == nullptr || remaining_distance == nullptr) {
    return false;
  }

  *command = driver_msgs::DriveCmd();
  *remaining_distance = 0.0;
  // 三种后端共享轨迹和车辆输入，未实现的后端返回false，由控制节点安全制动。
  switch (output_mode_) {
    case OutputMode::IDEAL_ACCELERATION:
      return computeIdealAccelerationCommand(
        input, command, remaining_distance);
    case OutputMode::CARLA_PEDAL:
      return computeCarlaPedalCommand(input, command, remaining_distance);
    case OutputMode::VEHICLE:
      return computeVehicleCommand(input, command, remaining_distance);
  }
  return false;
}

void OpenSpaceLongitudinalController::Reset()
{
  // 新轨迹不能继承上一段的匹配进度、积分量和加速度历史。
  progress_index_ = 0;
  speed_error_integral_ = 0.0;
  previous_acceleration_command_ = 0.0;
  previous_compute_time_ = ros::Time(0);
}

bool OpenSpaceLongitudinalController::computeIdealAccelerationCommand(
  const car::control::InputData &input,
  driver_msgs::DriveCmd *command,
  double *remaining_distance)
{
  MotionReference reference;
  if (!computeMotionReference(input, &reference)) {
    return false;
  }

  // 理想仿真直接使用有符号目标速度和加速度；踏板与扭矩保持为零。
  const double current_velocity = currentSignedVelocity(input);
  command->velocity_target = reference.target_velocity;
  command->acc_target = computeAccelerationCommand(
    reference.target_velocity, current_velocity,
    reference.remaining_distance, reference.requests_stop, controlPeriod());
  command->throttle_pedal = 0.0;
  command->brake_pedal = 0.0;
  command->engine_torque_target = 0.0;
  *remaining_distance = reference.remaining_distance;
  return true;
}

bool OpenSpaceLongitudinalController::computeCarlaPedalCommand(
  const car::control::InputData &input,
  driver_msgs::DriveCmd *command,
  double *remaining_distance)
{
  MotionReference reference;
  if (!computeMotionReference(input, &reference)) {
    return false;
  }

  const double current_velocity = currentSignedVelocity(input);
  const double acceleration_command = computeAccelerationCommand(
    reference.target_velocity, current_velocity,
    reference.remaining_distance, reference.requests_stop, controlPeriod());
  const double direction =
    input.trajectory_data.is_forward_shift ? 1.0 : -1.0;
  // 倒车加速时a_cmd为负，乘档位方向后仍得到正的沿行驶方向加速度；
  // 因此踏板选择不能简单使用a_cmd正负号。
  const double drive_direction_acceleration =
    direction * acceleration_command;
  const bool is_forward = input.trajectory_data.is_forward_shift;
  const double throttle_gain = is_forward ?
    carla_forward_throttle_acceleration_gain_ :
    carla_reverse_throttle_acceleration_gain_;
  const double throttle_offset = is_forward ?
    carla_forward_throttle_offset_ : carla_reverse_throttle_offset_;
  const double brake_gain = is_forward ?
    carla_forward_brake_deceleration_gain_ :
    carla_reverse_brake_deceleration_gain_;
  const double brake_offset = is_forward ?
    carla_forward_brake_offset_ : carla_reverse_brake_offset_;

  double throttle = 0.0;
  double brake = 0.0;
  const bool target_stopped =
    std::fabs(reference.target_velocity) <= 1.0e-3;
  const bool vehicle_stopped =
    std::fabs(current_velocity) <= stop_speed_tolerance_;
  if (target_stopped && vehicle_stopped) {
    // 末端停稳后维持行车制动，下一周期控制状态机会转入SEGMENT_END_HOLD。
    brake = carla_stop_brake_pedal_ / 100.0;
  } else if (drive_direction_acceleration <
    -carla_acceleration_deadband_)
  {
    // 需要沿行驶方向减速 → 使用制动分支（主要停车手段，不再依赖滑行）。
    // 制动模型给出请求减速度之上的附加制动力；offset 取较小值（弱滑行
    // 基线），使制动在弱滑行区域也能可靠起作用。
    brake = clampValue(
      (-drive_direction_acceleration - brake_offset) /
      brake_gain, 0.0, 1.0);
    // 停车阶段保证最小制动力，避免接近零速时滑行溜车。
    if (target_stopped) {
      brake = std::max(brake, 0.05);
    }
  } else if (!target_stopped) {
    // 加速或保持速度 → 油门。a_cmd 接近零但仍要求行驶时，
    // 用标定的零加速度油门（即 offset/gain）。
    throttle = clampValue(
      (drive_direction_acceleration + throttle_offset) /
      throttle_gain, 0.0, 1.0);
  } else {
    // 目标停车且车辆仍在缓慢移动、减速命令落在死区内时，
    // 直接施加最小制动力，确保停稳而不是滑行。
    brake = 0.05;
  }

  command->velocity_target = reference.target_velocity;
  command->acc_target = acceleration_command;
  command->throttle_pedal = 100.0 * throttle;
  command->brake_pedal = 100.0 * brake;
  command->engine_torque_target = 0.0;
  *remaining_distance = reference.remaining_distance;
  return true;
}

bool OpenSpaceLongitudinalController::computeVehicleCommand(
  const car::control::InputData &input,
  driver_msgs::DriveCmd *command,
  double *remaining_distance)
{
  // 预留实车执行器映射入口，后续可接现有车型的踏板或扭矩接口。
  (void)input;
  (void)command;
  (void)remaining_distance;
  ROS_WARN_THROTTLE(
    2.0, "[open_space_lon] vehicle mode is reserved but not implemented");
  return false;
}

bool OpenSpaceLongitudinalController::computeMotionReference(
  const car::control::InputData &input,
  MotionReference *reference)
{
  const auto &trajectory = input.trajectory_data;
  const auto &points = trajectory.points;
  if (reference == nullptr || points.size() < 2) {
    return false;
  }

  const std::size_t last_index = points.size() - 1;
  progress_index_ = std::min(progress_index_, last_index - 1);
  const auto &vehicle_position = input.location_data.location.pose.pose.position;
  // 允许向后检查少量线段以吸收定位噪声，但最终进度仍保持单调前进。
  const std::size_t search_begin =
    progress_index_ > 2 ? progress_index_ - 2 : 0;

  // 将车辆位置投影到离车最近的轨迹线段，得到连续的段索引和段内比例。
  std::size_t projection_segment = search_begin;
  double projection_ratio = 0.0;
  double projection_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = search_begin; index < last_index; ++index) {
    const auto &start = points[index];
    const auto &end = points[index + 1];
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double length_squared =
      segment_x * segment_x + segment_y * segment_y;
    if (length_squared <= 1.0e-9) {
      continue;
    }

    const double ratio = clampValue(
      ((vehicle_position.x - start.x) * segment_x +
      (vehicle_position.y - start.y) * segment_y) / length_squared,
      0.0, 1.0);
    const double projection_x = start.x + ratio * segment_x;
    const double projection_y = start.y + ratio * segment_y;
    const double distance = std::hypot(
      vehicle_position.x - projection_x,
      vehicle_position.y - projection_y);
    if (distance < projection_distance) {
      projection_distance = distance;
      projection_segment = index;
      projection_ratio = ratio;
    }
  }

  if (projection_segment >= progress_index_) {
    progress_index_ = projection_segment;
  } else {
    // 禁止匹配进度倒退，避免换挡点附近的定位抖动造成目标速度跳变。
    projection_segment = progress_index_;
    projection_ratio = 0.0;
  }

  // 剩余弧长同时供停车速度规划和控制节点的轨迹段结束判定使用。
  double remaining_distance =
    (1.0 - projection_ratio) *
    pointDistance(points[projection_segment], points[projection_segment + 1]);
  for (std::size_t index = projection_segment + 1;
    index < last_index; ++index)
  {
    remaining_distance += pointDistance(points[index], points[index + 1]);
  }

  // 按空间距离而不是轨迹点个数预瞄，避免轨迹采样疏密改变控制效果。
  std::size_t target_index = projection_segment + 1;
  double accumulated_distance =
    (1.0 - projection_ratio) *
    pointDistance(points[projection_segment], points[projection_segment + 1]);
  while (target_index < last_index &&
    accumulated_distance < lookahead_distance_)
  {
    accumulated_distance +=
      pointDistance(points[target_index], points[target_index + 1]);
    ++target_index;
  }

  const bool requests_stop = std::fabs(points.back().v) <= 1.0e-3;
  std::size_t speed_reference_index = target_index;
  if (requests_stop) {
    // 预瞄命中末端零速点时，回退到最后一个非零速度点；实际减速由停车包络决定。
    while (speed_reference_index > progress_index_ &&
      std::fabs(points[speed_reference_index].v) <= 1.0e-3)
    {
      --speed_reference_index;
    }
  }

  const double direction = trajectory.is_forward_shift ? 1.0 : -1.0;
  double target_velocity =
    direction * std::fabs(points[speed_reference_index].v);
  if (requests_stop) {
    // 随剩余距离收紧目标速度，并在末端小范围内明确下发零速。
    const double braking_limit = std::sqrt(
      std::max(0.0, 2.0 * max_deceleration_ * remaining_distance));
    target_velocity = std::copysign(
      std::min(std::fabs(target_velocity), braking_limit), direction);
    if (remaining_distance <= 0.05) {
      target_velocity = 0.0;
    }
  }

  reference->target_velocity = target_velocity;
  reference->remaining_distance = remaining_distance;
  reference->requests_stop = requests_stop;
  return true;
}

double OpenSpaceLongitudinalController::computeAccelerationCommand(
  double target_velocity,
  double current_velocity,
  double remaining_distance,
  bool requests_stop,
  double dt)
{
  const bool reverse = target_velocity < -1.0e-3 ||
    (std::fabs(target_velocity) <= 1.0e-3 && current_velocity < 0.0);
  const double kp = reverse ? reverse_speed_kp_ : forward_speed_kp_;
  const double ki = reverse ? reverse_speed_ki_ : forward_speed_ki_;
  const double speed_error = target_velocity - current_velocity;
  const bool braking =
    std::fabs(target_velocity) < std::fabs(current_velocity) ||
    current_velocity * target_velocity < 0.0;
  const double acceleration_limit =
    braking ? max_deceleration_ : max_acceleration_;

  // 先预测本周期积分结果；输出已饱和且误差继续推向饱和方向时停止积分。
  const double candidate_integral = clampValue(
    speed_error_integral_ + speed_error * dt,
    -speed_integral_limit_, speed_integral_limit_);
  const double candidate_command = kp * speed_error + ki * candidate_integral;
  const bool saturated_high = candidate_command > acceleration_limit;
  const bool saturated_low = candidate_command < -acceleration_limit;
  if ((!saturated_high && !saturated_low) ||
    (saturated_high && speed_error < 0.0) ||
    (saturated_low && speed_error > 0.0))
  {
    speed_error_integral_ = candidate_integral;
  }

  double acceleration_command = clampValue(
    kp * speed_error + ki * speed_error_integral_,
    -acceleration_limit, acceleration_limit);

  if (requests_stop && std::fabs(current_velocity) > 1.0e-3) {
    // 比较实际所需停车距离与剩余弧长，必要时优先制动并清除速度积分。
    // 倒车用实测滑行减速度的常量减速距离，前进沿用 jerk 受限估算。
    const double required_stopping_distance = reverse ?
      current_velocity * current_velocity /
          (2.0 * reverse_coast_deceleration_) :
      jerkLimitedStoppingDistance(
          current_velocity, max_deceleration_, max_jerk_);
    if (required_stopping_distance >= remaining_distance) {
      acceleration_command =
        -std::copysign(max_deceleration_, current_velocity);
      speed_error_integral_ = 0.0;
    }
  }

  if (max_jerk_ > 0.0) {
    // 限制相邻控制周期的加速度变化，模拟制动力建立过程并减小纵向冲击。
    const double max_delta = max_jerk_ * dt;
    acceleration_command = clampValue(
      acceleration_command,
      previous_acceleration_command_ - max_delta,
      previous_acceleration_command_ + max_delta);
  }
  previous_acceleration_command_ = acceleration_command;
  return acceleration_command;
}

double OpenSpaceLongitudinalController::currentSignedVelocity(
  const car::control::InputData &input) const
{
  // 部分底盘只反馈速度绝对值，泊车段方向用于恢复控制计算所需的速度符号。
  const double direction =
    input.trajectory_data.is_forward_shift ? 1.0 : -1.0;
  return direction * std::fabs(input.chassis_data.current_velocity);
}

double OpenSpaceLongitudinalController::controlPeriod()
{
  const ros::Time now = ros::Time::now();
  double dt = nominal_control_period_;
  if (!previous_compute_time_.isZero()) {
    const double measured_dt = (now - previous_compute_time_).toSec();
    // 回调间隔异常时退回标称周期，防止一次时间跳变放大积分量和jerk步长。
    if (measured_dt > 1.0e-3 && measured_dt < 0.2) {
      dt = measured_dt;
    }
  }
  previous_compute_time_ = now;
  return dt;
}

OpenSpaceLongitudinalController::OutputMode
OpenSpaceLongitudinalController::parseOutputMode(const std::string &mode)
{
  if (mode == "ideal_acceleration") {
    return OutputMode::IDEAL_ACCELERATION;
  }
  if (mode == "carla_pedal") {
    return OutputMode::CARLA_PEDAL;
  }
  if (mode != "vehicle") {
    ROS_WARN_STREAM("[open_space_lon] unknown output mode '" << mode
      << "', fallback to safe vehicle interface");
  }
  return OutputMode::VEHICLE;
}

const char *OpenSpaceLongitudinalController::outputModeName(OutputMode mode)
{
  switch (mode) {
    case OutputMode::IDEAL_ACCELERATION:
      return "ideal_acceleration";
    case OutputMode::CARLA_PEDAL:
      return "carla_pedal";
    case OutputMode::VEHICLE:
      return "vehicle";
  }
  return "unknown";
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
