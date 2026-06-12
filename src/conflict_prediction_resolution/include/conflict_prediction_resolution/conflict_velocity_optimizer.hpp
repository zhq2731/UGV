#pragma once

#include <cstddef>
#include <limits>

#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/node_handle.h>

namespace conflict_prediction_resolution
{

class ConflictVelocityOptimizer
{
public:
  // 从冲突消解参数文件读取 QP 权重、动力学边界和求解器设置。
  void loadParam(ros::NodeHandle& private_nh);

  // 在规则冲突速度规划之后调用：
  // 1. trajectory 已经经过规则限速/停车处理，是一条可行但可能不够平滑的粗解；
  // 2. fixed_prefix_end_index 是拼接段末点，QP 只优化该点及其之后的非拼接段；
  // 3. speed_cap 是超时保守限速或其他外部速度上限，若为 inf 则表示无额外限速；
  // 4. has_entry_time_constraint/yield_entry_s/target_entry_time_from_now 用于表达
  //    “不早于目标时间进入冲突区”的线性化约束；
  // 5. 若 QP 成功，trajectory 会被优化后的 s/v/a 覆盖；若 QP 失败，返回 false，
  //    调用方继续保留规则速度粗解作为安全兜底。
  bool optimize(planning_msgs::TrajectoryPointArray& trajectory,
                size_t fixed_prefix_end_index,
                double speed_cap,
                bool has_entry_time_constraint,
                double yield_entry_s,
                double target_entry_time_from_now) const;

  // 检查一条轨迹是否满足“不早于目标时间进入冲突区”。
  // 该检查服务于 QP/规则粗解的统一验收：如果轨迹没有走到冲突入口，视为仍在入口前等待。
  bool satisfiesEntryTimeConstraint(const planning_msgs::TrajectoryPointArray& trajectory,
                                    double yield_entry_s,
                                    double target_entry_time_from_now,
                                    double* checked_entry_time = nullptr) const;

private:
  bool enabled_ = true;

  // 目标函数权重：分别约束 s 贴近粗解、v 贴近规则速度、a 不过大、jerk 平滑。
  double weight_s_ref_ = 0.1;
  double weight_v_ref_ = 3.0;
  double weight_acc_ = 0.5;
  double weight_jerk_ = 10.0;

  // 车辆纵向动力学边界。
  double min_acc_ = -1.5;
  double max_acc_ = 1.0;
  double max_speed_ = 6.0;

  // 曲率速度约束使用的横向加速度上限：v <= sqrt(a_lat_max / |kappa|)。
  double max_lateral_acc_ = 1.5;

  // “不早于冲突时间进入”约束的空间余量：t < target_entry_time 时，s 不超过入口前该距离。
  double not_early_s_margin_ = 0.05;

  // OSQP 求解器设置。
  int max_iter_ = 4000;
  double eps_abs_ = 1.0e-5;
  double eps_rel_ = 1.0e-4;
  bool verbose_ = false;
};

}  // namespace conflict_prediction_resolution
