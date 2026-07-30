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

#ifndef TRAJECTORY_FOLLOWER__OPEN_SPACE_MPC_LATERAL_CONTROLLER_HPP_
#define TRAJECTORY_FOLLOWER__OPEN_SPACE_MPC_LATERAL_CONTROLLER_HPP_

#include "trajectory_follower/lateral_controller_base.hpp"
#include "trajectory_follower/mpc.hpp"
#include "trajectory_follower/visibility_control.hpp"

#include "autoware_msgs/AckermannLateralCommand.h"
#include "autoware_msgs/SteeringReport.h"
#include "autoware_msgs/TrajectoryPointArray.h"
#include "nav_msgs/Odometry.h"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <deque>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{

/**
 * @brief 开放空间泊车专用横向 MPC 控制器
 *
 * 该类拥有独立的轨迹缓存、控制历史和 MPC 实例。控制节点仅在
 * open_space_execution_mode=true 时创建本控制器，参考线场景继续使用原有
 * MpcLateralController。后续泊车的倒车模型、预瞄方式和参数调整均应优先
 * 在本类中实现，避免改变参考线横向控制行为。
 *
 * 当前阶段仍复用通用 MPC 数学模块、车辆模型和 QP 求解器实现；复用的是
 * 无状态算法代码，不会与参考线控制器共享运行时状态。
 */
class TRAJECTORY_FOLLOWER_PUBLIC OpenSpaceMpcLateralController
  : public LateralControllerBase
{
public:
  explicit OpenSpaceMpcLateralController(ros::NodeHandle & nh);
  ~OpenSpaceMpcLateralController() = default;

  /**
   * @brief 激活新泊车轨迹段前，以底盘实际前轮角初始化本段控制历史
   *
   * 只清理上一泊车段遗留的控制指令、输入延迟缓冲、滤波输出和轨迹形状
   * 缓存，不会修改车辆实际前轮角，也不会接触参考线控制器状态。
   */
  void resetForNewTrajectory(
    const autoware_msgs::SteeringReport & current_steer);

private:
  ros::NodeHandle * node_;
  vehicle_info_util::VehicleInfoUtil * vehicle_info_;

  bool enable_path_smoothing_;
  int path_filter_moving_average_num_;
  int trajectory_curvature_smoothing_num_;
  int reference_steer_curvature_smoothing_num_;
  double trajectory_resample_distance_;

  double stop_state_entry_ego_speed_;
  double stop_state_entry_target_speed_;
  double converged_steer_rad_;
  double new_trajectory_end_distance_;
  bool keep_steer_control_until_converged_;

  double ego_nearest_distance_threshold_;
  double ego_nearest_yaw_threshold_;

  std::deque<autoware_msgs::TrajectoryPointArray> trajectory_buffer_;
  MPC mpc_;

  nav_msgs::Odometry::Ptr current_odometry_;
  autoware_msgs::SteeringReport::Ptr current_steering_;
  autoware_msgs::TrajectoryPointArray::Ptr current_trajectory_;

  double previous_steer_command_{0.0};
  bool previous_control_initialized_{false};
  autoware_msgs::AckermannLateralCommand previous_control_command_;
  bool received_first_trajectory_{false};

  boost::optional<LateralOutput> run() override;
  void setInputData(InputData const & input_data) override;

  void setTrajectory(const autoware_msgs::TrajectoryPointArray::Ptr & trajectory);
  bool checkData() const;
  bool isValidTrajectory(
    const autoware_msgs::TrajectoryPointArray & trajectory) const;
  bool isStoppedState() const;
  bool isSteerConverged(
    const autoware_msgs::AckermannLateralCommand & command) const;
  bool isTrajectoryShapeChanged() const;

  autoware_msgs::AckermannLateralCommand createControlCommand(
    autoware_msgs::AckermannLateralCommand command);
  autoware_msgs::AckermannLateralCommand getStopControlCommand() const;
  autoware_msgs::AckermannLateralCommand getInitialControlCommand() const;

  /** @brief 读取当前泊车 MPC 参数；参数名暂与既有配置兼容。 */
  void loadMpcParameters();
};

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__OPEN_SPACE_MPC_LATERAL_CONTROLLER_HPP_
