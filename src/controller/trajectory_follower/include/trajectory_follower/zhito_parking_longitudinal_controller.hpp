#ifndef TRAJECTORY_FOLLOWER__ZHITO_PARKING_LONGITUDINAL_CONTROLLER_HPP_
#define TRAJECTORY_FOLLOWER__ZHITO_PARKING_LONGITUDINAL_CONTROLLER_HPP_

#include <vector>

#include <ros/ros.h>

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
 * @brief 智拓底盘的单车泊车纵向执行器
 *
 * 本类不读取轨迹，也不负责换挡和泊车流程状态机。上层每周期给出当前段的
 * 目标速度和已经限幅、限jerk的期望加速度，本类只完成油门/XBR标定映射。
 *
 * 设计边界：
 * 1. 轨迹匹配、剩余距离和停车速度包络由OpenSpaceLongitudinalController负责；
 * 2. 档位确认、转角准备和段间切换由泊车控制节点状态机负责；
 * 3. 本类不缓存轨迹，目标速度更新在下一次Compute调用立即生效；
 * 4. 本类不引入速度闭环和控制状态枚举，避免与公共PI形成重复控制。
 *
 * 执行器映射分为三个连续区域：
 * - 正加速度：查目标速度对应的匀速油门，再叠加加速度修正；
 * - 小负加速度：先将匀速油门平滑退到零，让车辆自然滑行；
 * - 较大负加速度：保持油门为零，再按减速度标定表渐入XBR。
 * 油门区和XBR区不重叠，因此不需要额外的驱动/制动状态机。
 */
class ZhitoParkingLongitudinalController
{
public:
  /**
   * @param node 泊车控制节点私有句柄，用于读取少量实车标定参数
   * @param maximum_deceleration_mps2 允许下发的最大XBR减速度绝对值
   * @param standstill_deceleration_mps2 零速目标下的制动保持减速度绝对值
   * @param stop_speed_tolerance_mps 判定车辆已经停稳的速度阈值
   */
  ZhitoParkingLongitudinalController(
    const ros::NodeHandle &node,
    double maximum_deceleration_mps2,
    double standstill_deceleration_mps2,
    double stop_speed_tolerance_mps);

  /**
   * @brief 将单周期速度参考转换为智拓底盘命令
   * @param target_speed_mps 沿行驶方向的目标速度大小，单位m/s
   * @param current_speed_mps 当前车速大小，单位m/s
   * @param is_forward true表示前进段，false表示倒车段
   * @param drive_acceleration_mps2 沿行驶方向的期望加速度，正加速、负减速
   * @param dt 本次控制周期，单位s
   * @param command 输出；油门为0~100%，制动使用负的acc_target
   */
  bool Compute(
    double target_speed_mps,
    double current_speed_mps,
    bool is_forward,
    double drive_acceleration_mps2,
    double dt,
    driver_msgs::DriveCmd *command);

  /** @brief 切换轨迹段或退出执行态时清除积分量和输出历史。 */
  void Reset();

private:
  /** @brief 对单调一维标定表进行线性插值，表外使用端点值。 */
  static double interpolateTable(
    double input,
    const std::vector<double> &input_table,
    const std::vector<double> &output_table);

  // 目标速度到匀速油门的标定表：输入m/s，输出油门百分比。
  // 前进、倒车分别适配传动、轮胎与转向负载差异。
  std::vector<double> forward_throttle_speed_table_;
  std::vector<double> forward_throttle_command_table_;
  std::vector<double> reverse_throttle_speed_table_;
  std::vector<double> reverse_throttle_command_table_;

  // 正加速度在匀速油门基础上增加油门，最终仍受最大值和上升率保护。
  // gain单位为“油门百分比/(m/s²)”。
  double throttle_acceleration_gain_pct_per_mps2_{5.0};
  double maximum_throttle_pct_{10.0};
  double throttle_rise_rate_pctps_{20.0};

  // 减速度到XBR目标值的标定表：输入、输出单位均为m/s²。
  // 过渡区前半段收油，后半段渐入XBR，中心点为纯滑行输出。
  std::vector<double> brake_deceleration_table_;
  std::vector<double> brake_command_table_;
  double throttle_brake_transition_mps2_{0.06};
  double maximum_deceleration_mps2_{0.9};
  double standstill_deceleration_mps2_{0.9};
  double stop_speed_tolerance_mps_{0.05};

  // 执行器层只保存上一周期油门，用于限制油门上升；减小油门立即生效。
  double previous_throttle_pct_{0.0};
};

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__ZHITO_PARKING_LONGITUDINAL_CONTROLLER_HPP_
