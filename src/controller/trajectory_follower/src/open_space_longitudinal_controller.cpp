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
    "open_space_longitudinal_control_mode", output_mode, "disabled");
  output_mode_ = parseOutputMode(output_mode);

  node.param<double>(
    "open_space_lookahead_distance", lookahead_distance_, 0.5);
  node.param<double>(
    "open_space_throttle_lpf_cutoff_hz", throttle_lpf_cutoff_hz_, 5.0);
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
  node.param<double>(
    "open_space_standstill_deceleration",
    vehicle_standstill_deceleration_, 0.9);
  // 前进/倒车独立的执行器标定模型（2026-08 精简：移除旧共用参数兼容读取）。
  // 油门：T(v,a)=T_hold(v)+a/G，2D 表优先，表缺失回退仿射模型；
  // 刹车：D=G_b*B-offset。前进倒车传动差异大，故独立标定。
  node.param<double>(
    "open_space_carla_forward_throttle_acceleration_gain",
    carla_forward_throttle_acceleration_gain_, 3.0);
  node.param<double>(
    "open_space_carla_reverse_throttle_acceleration_gain",
    carla_reverse_throttle_acceleration_gain_, 3.0);
  node.param<double>(
    "open_space_carla_forward_throttle_offset",
    carla_forward_throttle_offset_, 0.15);
  node.param<double>(
    "open_space_carla_reverse_throttle_offset",
    carla_reverse_throttle_offset_, 0.15);
  node.param<double>(
    "open_space_carla_forward_brake_deceleration_gain",
    carla_forward_brake_deceleration_gain_, 6.0);
  node.param<double>(
    "open_space_carla_reverse_brake_deceleration_gain",
    carla_reverse_brake_deceleration_gain_, 6.0);
  node.param<double>(
    "open_space_carla_forward_brake_offset",
    carla_forward_brake_offset_, 0.15);
  node.param<double>(
    "open_space_carla_reverse_brake_offset",
    carla_reverse_brake_offset_, 0.15);
  node.param<double>(
    "open_space_carla_acceleration_deadband",
    carla_acceleration_deadband_, 0.02);
  // 油门↔刹车混合区宽度：a 越过死区后的一段区间内油门渐隐、刹车渐入，
  // 消除"1帧油门突降"的分支硬切换。设为 0 则退化为纯硬切换行为。
  node.param<double>(
    "open_space_carla_throttle_brake_blend",
    carla_throttle_brake_blend_, 0.15);
  node.param<double>(
    "open_space_carla_stop_brake_pedal",
    carla_stop_brake_pedal_, 20.0);
  node.param<double>(
    "open_space_stop_speed_tolerance", stop_speed_tolerance_, 0.05);
  node.param<double>(
    "open_space_carla_reverse_coast_deceleration",
    reverse_coast_deceleration_, 1.8);
  node.param<double>(
    "open_space_carla_forward_stop_deceleration",
    forward_stop_deceleration_, 1.1);
  // 2D 油门标定表：T(v,a)=T_hold(v)+a/G。读取失败或尺寸不一致时回退仿射模型。
  node.getParam(
    "open_space_carla_forward_throttle_speed_table",
    forward_throttle_speed_table_);
  node.getParam(
    "open_space_carla_forward_throttle_hold_table",
    forward_throttle_hold_table_);
  node.getParam(
    "open_space_carla_reverse_throttle_speed_table",
    reverse_throttle_speed_table_);
  node.getParam(
    "open_space_carla_reverse_throttle_hold_table",
    reverse_throttle_hold_table_);
  node.param<double>(
    "open_space_carla_forward_throttle_gain",
    forward_throttle_gain_, 8.0);
  node.param<double>(
    "open_space_carla_reverse_throttle_gain",
    reverse_throttle_gain_, 5.75);
  if (forward_throttle_speed_table_.size() != forward_throttle_hold_table_.size() ||
      forward_throttle_speed_table_.empty()) {
    forward_throttle_speed_table_.clear();
    forward_throttle_hold_table_.clear();
  }
  if (reverse_throttle_speed_table_.size() != reverse_throttle_hold_table_.size() ||
      reverse_throttle_speed_table_.empty()) {
    reverse_throttle_speed_table_.clear();
    reverse_throttle_hold_table_.clear();
  }

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
  carla_throttle_brake_blend_ = std::max(0.0, carla_throttle_brake_blend_);
  carla_stop_brake_pedal_ =
    clampValue(carla_stop_brake_pedal_, 0.0, 100.0);
  stop_speed_tolerance_ = std::max(0.0, stop_speed_tolerance_);
  reverse_coast_deceleration_ = std::max(0.1, reverse_coast_deceleration_);
  forward_stop_deceleration_ = std::max(0.1, forward_stop_deceleration_);

  // 公共泊车速度环只生成加速度参考，车型层只负责油门和 XBR 映射。
  if (output_mode_ == OutputMode::VEHICLE) {
    vehicle_controller_.reset(new ZhitoParkingLongitudinalController(
      node, max_deceleration_, vehicle_standstill_deceleration_,
      stop_speed_tolerance_));
  }

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
    case OutputMode::DISABLED:
      return false;
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
  throttle_lpf_valid_ = false;
  if (vehicle_controller_) {
    vehicle_controller_->Reset();
  }
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
    //
    // 错开混合区（2026-08-05）：模拟"先松油门、不够再刹车"的驾驶习惯，
    // 油门和刹车分时输出（不共存）。混合区宽度 carla_throttle_brake_blend_
    // 被劈成两半：
    //   前半 [deadband, deadband+half]：只松油门（throttle 渐隐，brake=0）
    //   后半 [deadband+half, deadband+width]：油门已归0，刹车渐入
    // 避免对称混合区"油门刹车同时输出"的违和感，同时保留渐隐/渐入的
    // 平滑过渡（无硬切换的油门瞬时归零）。
    const double d = -drive_direction_acceleration;  // 减速需求（>0）
    const double half_blend =
      std::max(1.0e-3, 0.5 * carla_throttle_brake_blend_);
    const double throttle_fade = clampValue(
      (d - carla_acceleration_deadband_) / half_blend, 0.0, 1.0);
    const double brake_fade = clampValue(
      (d - carla_acceleration_deadband_ - half_blend) / half_blend,
      0.0, 1.0);
    brake = clampValue(
      (d - brake_offset) / brake_gain, 0.0, 1.0) * brake_fade;
    throttle = throttle2D(
      is_forward, std::fabs(current_velocity),
      drive_direction_acceleration) * (1.0 - throttle_fade);
    // 停车阶段保证最小制动力，避免接近零速时滑行溜车。
    if (target_stopped) {
      brake = std::max(brake, 0.05);
    }
  } else if (!target_stopped) {
    // 加速或保持速度 → 2D 油门查表 T(v,a)=T_hold(v)+a/G。
    // T_hold(v) 随速度变化，避免一维仿射模型在巡航时的油门猎振。
    throttle = throttle2D(
      is_forward, std::fabs(current_velocity),
      drive_direction_acceleration);
  } else {
    // 目标停车且车辆仍在缓慢移动、减速命令落在死区内时，
    // 直接施加最小制动力，确保停稳而不是滑行。
    brake = 0.05;
  }

  // 油门低通滤波（2026-08-05）：一阶低通平滑油门输出。倒车大转向时实际
  // 速度 v 被轮胎阻力拖着高频波动，速度环跟随放大到油门。低通滤掉高频分量，
  // 只保留低频趋势。截止频率 throttle_lpf_cutoff_hz_（默认 5Hz）。
  if (throttle_lpf_cutoff_hz_ > 0.0) {
    if (throttle_lpf_valid_) {
      const double tau =
        1.0 / (2.0 * 3.141592653589793 * throttle_lpf_cutoff_hz_);
      const double dt = controlPeriod();
      const double alpha = dt / (tau + dt);
      throttle = alpha * throttle + (1.0 - alpha) * throttle_lpf_value_;
    } else {
      throttle_lpf_valid_ = true;
    }
    throttle_lpf_value_ = throttle;
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
  MotionReference reference;
  if (!vehicle_controller_ || !computeMotionReference(input, &reference)) {
    return false;
  }
  *remaining_distance = reference.remaining_distance;

  // 倒车公共速度环使用负速度；车型层改用沿行驶方向的正加速、负制动约定。
  const double dt = controlPeriod();
  const double signed_acceleration = computeAccelerationCommand(
    reference.target_velocity, currentSignedVelocity(input),
    reference.remaining_distance, false, dt);
  const double direction = input.trajectory_data.is_forward_shift ? 1.0 : -1.0;
  return vehicle_controller_->Compute(
    std::fabs(reference.target_velocity),
    std::fabs(input.chassis_data.current_velocity),
    input.trajectory_data.is_forward_shift,
    direction * signed_acceleration, dt, command);
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
  if (requests_stop || output_mode_ == OutputMode::VEHICLE) {
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
  if (requests_stop || output_mode_ == OutputMode::VEHICLE) {
    // 随剩余距离收紧目标速度，并在末端小范围内明确下发零速。
    // 减速能力用实测停车减速度（前进=forward_stop_deceleration 1.1，
    // 倒车=max_deceleration 1.1），使目标速度包络与实际制动能力一致，
    // 避免"刹停过早→距终点仍有余量→再启动"。
    const double stop_deceleration = output_mode_ == OutputMode::VEHICLE ?
      max_deceleration_ :
      (trajectory.is_forward_shift ? forward_stop_deceleration_ :
                                     max_deceleration_);
    const double braking_limit = std::sqrt(
      std::max(0.0, 2.0 * stop_deceleration * remaining_distance));
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
    // 用"实测停车减速度"的常量减速距离：前进=forward_stop_deceleration(1.1，
    // 刹车+滑行)，倒车=reverse_coast_deceleration(1.8，倒车自然滑行强)。
    // 避免刹停过早/停不到终点。
    const double required_stopping_distance =
      current_velocity * current_velocity /
          (2.0 * (reverse ? reverse_coast_deceleration_ :
                            forward_stop_deceleration_));
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

double OpenSpaceLongitudinalController::throttle2D(
  const bool is_forward, const double speed, const double drive_accel) const
{
  const std::vector<double> &speed_table =
    is_forward ? forward_throttle_speed_table_ : reverse_throttle_speed_table_;
  const std::vector<double> &hold_table =
    is_forward ? forward_throttle_hold_table_ : reverse_throttle_hold_table_;
  const double gain = is_forward ? forward_throttle_gain_ : reverse_throttle_gain_;

  // 标定表缺失或损坏时回退到旧的仿射模型 (a = G*T - offset)。
  if (speed_table.size() < 2 || speed_table.size() != hold_table.size()) {
    const double offset =
      is_forward ? carla_forward_throttle_offset_ : carla_reverse_throttle_offset_;
    const double g = is_forward ?
      carla_forward_throttle_acceleration_gain_ :
      carla_reverse_throttle_acceleration_gain_;
    return clampValue((drive_accel + offset) / std::max(1.0e-3, g), 0.0, 1.0);
  }

  // 分段线性插值 T_hold(v)，速度区间外线性外推。
  double t_hold = 0.0;
  if (speed <= speed_table.front()) {
    t_hold = hold_table.front();
  } else if (speed >= speed_table.back()) {
    t_hold = hold_table.back();
  } else {
    for (std::size_t i = 0; i + 1 < speed_table.size(); ++i) {
      if (speed >= speed_table[i] && speed <= speed_table[i + 1]) {
        const double denom = speed_table[i + 1] - speed_table[i];
        const double ratio = denom > 1.0e-9 ?
          (speed - speed_table[i]) / denom : 0.0;
        t_hold = hold_table[i] +
          ratio * (hold_table[i + 1] - hold_table[i]);
        break;
      }
    }
  }
  return clampValue(
    t_hold + drive_accel / std::max(0.1, gain), 0.0, 1.0);
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
  if (mode == "vehicle") {
    return OutputMode::VEHICLE;
  }
  ROS_WARN_STREAM("[open_space_lon] output mode '" << mode
    << "' is disabled; no longitudinal command will be generated");
  return OutputMode::DISABLED;
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
    case OutputMode::DISABLED:
      return "disabled";
  }
  return "unknown";
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
