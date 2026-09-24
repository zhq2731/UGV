#include "trajectory_follower/zhito_parking_longitudinal_controller.hpp"

#include <algorithm>
#include <cmath>

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

double clampValue(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

bool validTable(
  const std::vector<double> &input_table,
  const std::vector<double> &output_table)
{
  // 至少两个点且横纵坐标等长，才能执行确定的一维插值。
  if (input_table.size() < 2 || input_table.size() != output_table.size()) {
    return false;
  }
  for (std::size_t index = 1; index < input_table.size(); ++index) {
    if (input_table[index] <= input_table[index - 1]) {
      return false;
    }
  }
  return true;
}

}  // namespace

ZhitoParkingLongitudinalController::ZhitoParkingLongitudinalController(
  const ros::NodeHandle &node,
  const double maximum_deceleration_mps2,
  const double standstill_deceleration_mps2,
  const double stop_speed_tolerance_mps)
: maximum_deceleration_mps2_(std::max(0.0, maximum_deceleration_mps2)),
  standstill_deceleration_mps2_(std::max(0.0, standstill_deceleration_mps2)),
  stop_speed_tolerance_mps_(std::max(0.0, stop_speed_tolerance_mps))
{
  // 车型层只读取执行器标定，不再保存另一套速度PI、制动阈值或jerk参数。
  node.getParam(
    "open_space_vehicle_forward_throttle_speed_table",
    forward_throttle_speed_table_);
  node.getParam(
    "open_space_vehicle_forward_throttle_command_table",
    forward_throttle_command_table_);
  node.getParam(
    "open_space_vehicle_reverse_throttle_speed_table",
    reverse_throttle_speed_table_);
  node.getParam(
    "open_space_vehicle_reverse_throttle_command_table",
    reverse_throttle_command_table_);
  node.getParam(
    "open_space_vehicle_brake_deceleration_table",
    brake_deceleration_table_);
  node.getParam(
    "open_space_vehicle_brake_command_table",
    brake_command_table_);
  node.param<double>(
    "open_space_vehicle_throttle_acceleration_gain_pct_per_mps2",
    throttle_acceleration_gain_pct_per_mps2_, 5.0);
  node.param<double>(
    "open_space_vehicle_maximum_throttle_pct", maximum_throttle_pct_, 10.0);
  node.param<double>(
    "open_space_vehicle_throttle_rise_rate_pctps",
    throttle_rise_rate_pctps_, 20.0);
  node.param<double>(
    "open_space_vehicle_throttle_brake_transition_mps2",
    throttle_brake_transition_mps2_, 0.06);

  // 缺表时使用保守的低速初值，保证配置错误不会恢复旧TankController路径。
  if (!validTable(
      forward_throttle_speed_table_, forward_throttle_command_table_)) {
    forward_throttle_speed_table_ = {0.0, 1.0};
    forward_throttle_command_table_ = {3.0, 4.7};
    ROS_WARN("[zhito_parking_lon] invalid forward throttle table; use defaults");
  }
  if (!validTable(
      reverse_throttle_speed_table_, reverse_throttle_command_table_)) {
    reverse_throttle_speed_table_ = {0.0, 1.0};
    reverse_throttle_command_table_ = {3.0, 4.7};
    ROS_WARN("[zhito_parking_lon] invalid reverse throttle table; use defaults");
  }
  if (!validTable(brake_deceleration_table_, brake_command_table_)) {
    brake_deceleration_table_ = {0.0, maximum_deceleration_mps2_};
    brake_command_table_ = {0.0, maximum_deceleration_mps2_};
    ROS_WARN("[zhito_parking_lon] invalid XBR table; use identity mapping");
  }

  throttle_acceleration_gain_pct_per_mps2_ =
    std::max(0.0, throttle_acceleration_gain_pct_per_mps2_);
  maximum_throttle_pct_ = clampValue(maximum_throttle_pct_, 0.0, 100.0);
  throttle_rise_rate_pctps_ = std::max(0.0, throttle_rise_rate_pctps_);
  throttle_brake_transition_mps2_ =
    std::max(0.0, throttle_brake_transition_mps2_);
}

bool ZhitoParkingLongitudinalController::Compute(
  const double target_speed_mps,
  const double current_speed_mps,
  const bool is_forward,
  const double drive_acceleration_mps2,
  const double dt,
  driver_msgs::DriveCmd *command)
{
  if (command == nullptr) {
    return false;
  }

  const double target_speed = std::fabs(target_speed_mps);
  const double current_speed = std::fabs(current_speed_mps);
  const double direction = is_forward ? 1.0 : -1.0;
  command->velocity_target = direction * target_speed;
  command->engine_torque_target = 0.0;
  command->brake_pedal = 0.0;  // 智拓通过负acc_target请求XBR制动。

  // 停稳保持属于安全动作，不经过动态PI和执行器过渡区。
  if (target_speed <= 1.0e-3 && current_speed <= stop_speed_tolerance_mps_) {
    command->throttle_pedal = 0.0;
    command->acc_target = -standstill_deceleration_mps2_;
    Reset();
    return true;
  }

  const std::vector<double> &speed_table = is_forward ?
    forward_throttle_speed_table_ : reverse_throttle_speed_table_;
  const std::vector<double> &throttle_table = is_forward ?
    forward_throttle_command_table_ : reverse_throttle_command_table_;
  // 匀速油门是执行器前馈，不参与公共PI积分，也不会改变加速度限幅。
  // 目标为零时基础油门必须为零，否则会抵消公共PI给出的停车减速度。
  const double hold_throttle = target_speed > 1.0e-3 ?
    interpolateTable(target_speed, speed_table, throttle_table) : 0.0;

  double requested_throttle = 0.0;
  double requested_xbr = 0.0;
  if (drive_acceleration_mps2 >= 0.0) {
    // 正加速度：速度标定表提供匀速油门，期望加速度提供附加油门。
    // PI只决定动态修正量，因此更换车型时主要重新标定此处映射即可。
    requested_throttle = hold_throttle +
      throttle_acceleration_gain_pct_per_mps2_ * drive_acceleration_mps2;
  } else {
    const double deceleration = -drive_acceleration_mps2;
    const double half_transition =
      std::max(1.0e-6, 0.5 * throttle_brake_transition_mps2_);
    // 前半区仅将油门由保持值渐退到零；后半区油门保持零、XBR渐入。
    // 两个fade区间错开，任一周期都不会同时请求油门和制动。
    // deceleration=0时throttle_fade=1；到达半区宽度时油门恰好为零。
    // brake_fade从半区宽度处的0增长到完整过渡宽度处的1。
    const double throttle_fade = clampValue(
      1.0 - deceleration / half_transition, 0.0, 1.0);
    const double brake_fade = clampValue(
      (deceleration - half_transition) / half_transition, 0.0, 1.0);
    requested_throttle = hold_throttle * throttle_fade;
    const double calibrated_brake = interpolateTable(
      deceleration, brake_deceleration_table_, brake_command_table_);
    // 标定表允许补偿底盘实际XBR响应与请求减速度之间的静态偏差。
    // 最终再次受公共最大减速度约束，错误标定不能突破泊车动态上限。
    requested_xbr = -std::min(
      maximum_deceleration_mps2_, calibrated_brake * brake_fade);
  }

  // 油门上升受限而下降立即生效，目标降低或进入制动区时不会延迟收油。
  requested_throttle = clampValue(
    requested_throttle, 0.0, maximum_throttle_pct_);
  command->throttle_pedal = std::min(
    requested_throttle,
    previous_throttle_pct_ + throttle_rise_rate_pctps_ * dt);
  command->acc_target = requested_xbr;
  previous_throttle_pct_ = command->throttle_pedal;

  ROS_DEBUG_THROTTLE(
    0.2,
    "[zhito_parking_lon] dir=%s target=%.3f current=%.3f accel=%.3f "
    "throttle=%.2f%% xbr=%.3f",
    is_forward ? "D" : "R", target_speed, current_speed,
    drive_acceleration_mps2, command->throttle_pedal, command->acc_target);
  return true;
}

void ZhitoParkingLongitudinalController::Reset()
{
  // PI和jerk历史位于公共纵控；车型执行器层只需清除油门斜率历史。
  previous_throttle_pct_ = 0.0;
}

double ZhitoParkingLongitudinalController::interpolateTable(
  const double input,
  const std::vector<double> &input_table,
  const std::vector<double> &output_table)
{
  // 表外不做线性外推，避免低速试验表被意外用于更高速度或减速度。
  if (input <= input_table.front()) {
    return output_table.front();
  }
  if (input >= input_table.back()) {
    return output_table.back();
  }
  for (std::size_t index = 0; index + 1 < input_table.size(); ++index) {
    if (input <= input_table[index + 1]) {
      // 在相邻标定点之间按输入距离比例插值，保证执行器命令连续。
      const double ratio = (input - input_table[index]) /
        (input_table[index + 1] - input_table[index]);
      return output_table[index] +
        ratio * (output_table[index + 1] - output_table[index]);
    }
  }
  return output_table.back();
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
