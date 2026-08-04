#ifndef TRAJECTORY_FOLLOWER__OPEN_SPACE_LONGITUDINAL_CONTROLLER_HPP_
#define TRAJECTORY_FOLLOWER__OPEN_SPACE_LONGITUDINAL_CONTROLLER_HPP_

#include <cstddef>
#include <string>

#include <ros/ros.h>

#include "control/controller/lon_controller.h"
#include "driver_msgs/DriveCmd.h"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{

/**
 * @brief 开放空间泊车专用纵向控制器
 *
 * 控制器读取与参考线纵控相同的输入，内部选择理想加速度仿真、CARLA踏板
 * 或实车执行后端。泊车流程状态机仍由控制节点负责，本类只在轨迹执行阶段
 * 计算纵向命令。
 */
class OpenSpaceLongitudinalController
{
public:
  explicit OpenSpaceLongitudinalController(const ros::NodeHandle &node);

  /**
   * @brief 计算当前周期的泊车纵向命令
   * @param input 轨迹、定位、底盘和运动开始命令
   * @param command 输出的速度、加速度或踏板命令
   * @param remaining_distance 当前投影位置到轨迹末端的剩余弧长
   * @return 当前执行后端已实现且命令有效时返回true
   */
  bool Compute(
    const car::control::InputData &input,
    driver_msgs::DriveCmd *command,
    double *remaining_distance);

  /** @brief 新轨迹段、换挡或任务重置时清除纵控内部历史。 */
  void Reset();

private:
  // 输出后端由控制器内部选择，控制节点无需区分仿真、CARLA和实车。
  enum class OutputMode
  {
    IDEAL_ACCELERATION,  // 输出目标速度和目标加速度，供当前轻量仿真节点积分。
    CARLA_PEDAL,         // 将期望加速度映射为CARLA/Lite使用的油门、制动百分比。
    VEHICLE,             // 输出实车执行器命令，暂留具体车型适配接口。
  };

  /**
   * @brief 从轨迹几何和当前车辆位置提取出的单周期纵向参考
   *
   * target_velocity已经带有前进为正、倒车为负的符号；remaining_distance
   * 是车辆在当前轨迹段上的投影点到末点的弧长，而不是车辆到末点的直线距离。
   */
  struct MotionReference
  {
    double target_velocity{0.0};
    double remaining_distance{0.0};
    bool requests_stop{true};
  };

  /** @brief 当前轻量仿真后端：输出目标速度及经过PI、加速度、jerk限制的加速度。 */
  bool computeIdealAccelerationCommand(
    const car::control::InputData &input,
    driver_msgs::DriveCmd *command,
    double *remaining_distance);
  /**
   * @brief CARLA踏板控制后端
   *
   * 先复用统一的轨迹投影与速度闭环得到有符号期望加速度，再依据当前档位
   * 转成沿行驶方向的加减速度，最后通过可配置执行器增益反算油门和制动。
   */
  bool computeCarlaPedalCommand(
    const car::control::InputData &input,
    driver_msgs::DriveCmd *command,
    double *remaining_distance);
  /** @brief 实车纵向执行器后端预留入口；后续在此完成车型相关命令映射。 */
  bool computeVehicleCommand(
    const car::control::InputData &input,
    driver_msgs::DriveCmd *command,
    double *remaining_distance);
  /**
   * @brief 将车辆位置投影到当前轨迹段，并按弧长生成预瞄速度与停车参考
   *
   * 处理流程为：最近线段投影 -> 单调进度更新 -> 剩余弧长累计 ->
   * 空间预瞄 -> 前倒车符号恢复 -> 末端停车速度包络。
   */
  bool computeMotionReference(
    const car::control::InputData &input,
    MotionReference *reference);
  /**
   * @brief 根据速度误差计算加速度命令
   *
   * 前进和倒车分别选择参数，随后依次执行PI、抗积分饱和、加减速度限幅、
   * jerk受限停车保护以及相邻周期jerk限幅。
   */
  double computeAccelerationCommand(
    double target_velocity,
    double current_velocity,
    double remaining_distance,
    bool requests_stop,
    double dt);
  /** @brief 根据当前轨迹方向把底盘速度绝对值恢复成控制器使用的有符号速度。 */
  double currentSignedVelocity(const car::control::InputData &input) const;
  /** @brief 获取本周期实际时间间隔；异常间隔回退到配置的标称控制周期。 */
  double controlPeriod();
  /** @brief 解析纵控后端名称；未知名称回退到尚未实现的安全实车接口。 */
  static OutputMode parseOutputMode(const std::string &mode);
  static const char *outputModeName(OutputMode mode);

  OutputMode output_mode_{OutputMode::VEHICLE};
  // 轨迹参考生成参数。
  double lookahead_distance_{0.5};
  // 前进、倒车独立速度闭环参数，便于后续针对不同传动特性分别标定。
  double forward_speed_kp_{1.5};
  double forward_speed_ki_{0.0};
  double reverse_speed_kp_{1.5};
  double reverse_speed_ki_{0.0};
  double speed_integral_limit_{0.5};
  // 纵向舒适性和执行器能力约束。
  double max_acceleration_{0.5};
  double max_deceleration_{1.0};
  double max_jerk_{2.0};
  double nominal_control_period_{0.02};
  // CARLA/Lite归一化踏板执行器模型。Cybertruck前进和倒车的传动、
  // 发动机制动差异很大，因此两方向独立标定。DriveCmd仍使用0～100百分比。
  double carla_forward_throttle_acceleration_gain_{3.0};
  double carla_reverse_throttle_acceleration_gain_{3.0};
  double carla_forward_throttle_offset_{0.15};
  double carla_reverse_throttle_offset_{0.15};
  double carla_forward_brake_deceleration_gain_{6.0};
  double carla_reverse_brake_deceleration_gain_{6.0};
  double carla_forward_brake_offset_{0.15};
  double carla_reverse_brake_offset_{0.15};
  double carla_acceleration_deadband_{0.02};
  double carla_stop_brake_pedal_{20.0};
  double stop_speed_tolerance_{0.05};

  // 以下状态只属于当前已激活轨迹段，切换轨迹段时由Reset统一清除。
  std::size_t progress_index_{0};
  double speed_error_integral_{0.0};
  double previous_acceleration_command_{0.0};
  // Cybertruck倒车一旦进入末端停车包络便锁存滑行状态，避免减速阶段
  // 继续使用标定有效区间之外的小油门；新轨迹段由Reset解除锁存。
  bool reverse_terminal_coast_active_{false};
  ros::Time previous_compute_time_;
};

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__OPEN_SPACE_LONGITUDINAL_CONTROLLER_HPP_
