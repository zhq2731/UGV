#include "conflict_prediction_resolution/conflict_velocity_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <ros/ros.h>

#include "velocity_planner/solver/osqp_interface/osqp_interface.h"

namespace conflict_prediction_resolution
{
namespace
{

size_t sIndex(const size_t i)
{
  // 优化变量按块排列：
  // x = [s0 ... sN-1, v0 ... vN-1, a0 ... aN-1]^T。
  // sIndex(i) 返回第 i 个时间节点的纵向进度变量 s_i 在 x 中的位置。
  return i;
}

size_t vIndex(const size_t n, const size_t i)
{
  // 速度变量 v_i 紧跟在全部 s 变量之后，因此偏移量是 N。
  return n + i;
}

size_t aIndex(const size_t n, const size_t i)
{
  // 加速度变量 a_i 位于第三个变量块，因此偏移量是 2N。
  return 2 * n + i;
}

double clampValue(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

double curvatureSpeedLimit(const double kappa, const double max_lateral_acc, const double max_speed)
{
  // 曲率约束来自横向加速度近似：a_lat = v^2 * |kappa|。
  // kappa 很小时认为曲率不限制速度，只使用全局最大速度。
  const double abs_kappa = std::fabs(kappa);
  if (abs_kappa < 1.0e-5 || max_lateral_acc <= 0.0)
  {
    return max_speed;
  }
  return std::min(max_speed, std::sqrt(std::max(0.0, max_lateral_acc / abs_kappa)));
}

bool isFiniteTrajectoryPoint(const planning_msgs::TrajectoryPoint& point)
{
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z) &&
         std::isfinite(point.theta) &&
         std::isfinite(point.s) &&
         std::isfinite(point.v) &&
         std::isfinite(point.a) &&
         std::isfinite(point.relative_time);
}

bool validateOptimizedTrajectory(const planning_msgs::TrajectoryPointArray& trajectory,
                                 const size_t start_index,
                                 const double min_s_gap,
                                 const double min_xy_gap)
{
  // 控制器会对轨迹点做样条/时间插值，因此 QP 输出必须满足几个基本条件：
  // 1. 数值全部有限；
  // 2. relative_time 严格递增；
  // 3. s 不允许回退；
  // 4. 非拼接段不能被压成大量重复几何点，否则控制器去重后可能点数不足而崩溃。
  if (trajectory.points.empty() || start_index + 1 >= trajectory.points.size())
  {
    return false;
  }

  size_t unique_count = 1;
  for (size_t i = start_index; i < trajectory.points.size(); ++i)
  {
    const auto& current = trajectory.points[i];
    if (!isFiniteTrajectoryPoint(current) || current.v < -1.0e-3)
    {
      return false;
    }

    if (i == start_index)
    {
      continue;
    }

    const auto& previous = trajectory.points[i - 1];
    const double dt = current.relative_time - previous.relative_time;
    if (dt <= 1.0e-4)
    {
      return false;
    }

    const double ds = current.s - previous.s;
    if (ds < -1.0e-4)
    {
      return false;
    }

    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    if (ds <= min_s_gap && std::hypot(dx, dy) <= min_xy_gap)
    {
      return false;
    }
    if (ds > min_s_gap || std::hypot(dx, dy) > min_xy_gap)
    {
      ++unique_count;
    }
  }

  return unique_count >= 2;
}

planning_msgs::TrajectoryPoint interpolatePointByS(const planning_msgs::TrajectoryPointArray& reference,
                                                   const size_t start_index,
                                                   const double query_s)
{
  // QP 优化变量包含 s。为了让输出轨迹的 x/y/theta/kappa 与优化后的 s 保持一致，
  // 这里沿规则粗解轨迹做一次按 s 的几何插值。
  // 这样 QP 只负责纵向时间参数化，不重新做横向路径规划；横向几何仍沿用当前 planner
  // 已生成的路径，符合“路径规划后再做冲突速度优化”的模块边界。
  if (reference.points.empty())
  {
    return planning_msgs::TrajectoryPoint();
  }

  const size_t begin = std::min(start_index, reference.points.size() - 1);
  if (query_s <= reference.points[begin].s)
  {
    return reference.points[begin];
  }

  for (size_t i = begin + 1; i < reference.points.size(); ++i)
  {
    const auto& prev = reference.points[i - 1];
    const auto& next = reference.points[i];
    if (query_s > next.s)
    {
      continue;
    }

    const double ds = next.s - prev.s;
    const double ratio = ds > 1.0e-6 ? clampValue((query_s - prev.s) / ds, 0.0, 1.0) : 0.0;

    planning_msgs::TrajectoryPoint out = prev;
    out.x = prev.x + (next.x - prev.x) * ratio;
    out.y = prev.y + (next.y - prev.y) * ratio;
    out.z = prev.z + (next.z - prev.z) * ratio;
    out.theta = prev.theta + (next.theta - prev.theta) * ratio;
    out.s = query_s;
    out.kappa = prev.kappa + (next.kappa - prev.kappa) * ratio;
    out.dkappa = prev.dkappa + (next.dkappa - prev.dkappa) * ratio;
    return out;
  }

  auto out = reference.points.back();
  out.s = std::min(query_s, reference.points.back().s);
  return out;
}

}  // namespace

void ConflictVelocityOptimizer::loadParam(ros::NodeHandle& private_nh)
{
  // 所有 QP 参数都放在 conflict_resolution.yaml 中，便于把“冲突判定参数”
  // 和“冲突速度消解参数”分开调试。
  private_nh.param<bool>("enable_conflict_velocity_qp", enabled_, true);
  private_nh.param<double>("conflict_qp_weight_s_ref", weight_s_ref_, weight_s_ref_);
  private_nh.param<double>("conflict_qp_weight_v_ref", weight_v_ref_, weight_v_ref_);
  private_nh.param<double>("conflict_qp_weight_acc", weight_acc_, weight_acc_);
  private_nh.param<double>("conflict_qp_weight_jerk", weight_jerk_, weight_jerk_);
  private_nh.param<double>("conflict_qp_min_acc", min_acc_, min_acc_);
  private_nh.param<double>("conflict_qp_max_acc", max_acc_, max_acc_);
  private_nh.param<double>("conflict_qp_max_speed", max_speed_, max_speed_);
  private_nh.param<double>("conflict_qp_max_lateral_acc", max_lateral_acc_, max_lateral_acc_);
  private_nh.param<double>("conflict_qp_not_early_s_margin", not_early_s_margin_, not_early_s_margin_);
  private_nh.param<bool>("conflict_qp_enable_output_validation",
                         enable_output_validation_,
                         enable_output_validation_);
  private_nh.param<double>("conflict_qp_min_output_s_gap", min_output_s_gap_, min_output_s_gap_);
  private_nh.param<double>("conflict_qp_min_output_xy_gap", min_output_xy_gap_, min_output_xy_gap_);
  private_nh.param<int>("conflict_qp_max_points", max_points_, max_points_);
  private_nh.param<double>("conflict_qp_min_dt", min_dt_, min_dt_);
  private_nh.param<int>("conflict_qp_max_iter", max_iter_, max_iter_);
  private_nh.param<double>("conflict_qp_eps_abs", eps_abs_, eps_abs_);
  private_nh.param<double>("conflict_qp_eps_rel", eps_rel_, eps_rel_);
  private_nh.param<bool>("conflict_qp_verbose", verbose_, verbose_);
}

bool ConflictVelocityOptimizer::optimize(planning_msgs::TrajectoryPointArray& trajectory,
                                         const size_t fixed_prefix_end_index,
                                         const double speed_cap,
                                         const bool has_entry_time_constraint,
                                         const double yield_entry_s,
                                         const double target_entry_time_from_now) const
{
  // 入口保护：
  // - QP 关闭、轨迹为空、非拼接段点数不足时不优化；
  // - 返回 false 后，调用方会保留规则速度规划粗解，不会发布半成品 QP 结果。
  if (!enabled_ || trajectory.points.empty() || fixed_prefix_end_index + 2 >= trajectory.points.size())
  {
    return false;
  }

  // fixed_prefix_end_index 是拼接段末点。它本身作为 QP 的第 0 个节点参与优化，
  // 但 s0/v0/a0 会被等式约束固定住，用来保证优化后的非拼接段与拼接段连续。
  const size_t start = fixed_prefix_end_index;
  const size_t n = trajectory.points.size() - start;
  if (n < 3)
  {
    return false;
  }
  if (max_points_ > 0 && n > static_cast<size_t>(max_points_))
  {
    // 当前实现复用 velocity_planner 的 dense OSQPInterface，矩阵规模随 N^2 增长。
    // 低速让行时如果固定时间粗轨迹被采成数千点，会导致内存/耗时急剧放大，
    // 甚至把 planner 打崩。因此在建矩阵前直接拒绝超大问题，回退规则粗解。
    ROS_WARN_THROTTLE(1.0,
                      "conflict velocity QP skipped: too many points n=%zu max=%d",
                      n,
                      max_points_);
    return false;
  }

  const planning_msgs::TrajectoryPointArray reference = trajectory;
  // reference 是规则冲突速度规划输出的粗解。QP 的目标函数会尽量贴近它，
  // 因此规则解提供“可行趋势”，QP 负责平滑和精确满足约束。
  const double start_s = reference.points[start].s;
  const double end_s = std::max(start_s, reference.points.back().s);
  const size_t var_size = 3 * n;
  // 约束行数：
  // 1. n 行 s 边界；
  // 2. n 行 v 边界；
  // 3. n 行 a 边界；
  // 4. n-1 行 s 动力学等式；
  // 5. n-1 行 v 动力学等式。
  const size_t constraint_size = 3 * n + 2 * (n - 1);

  // OSQP 使用标准二次规划形式：
  //   min  1/2 * x^T P x + q^T x
  //   s.t. l <= A x <= u
  // 这里 hessian 对应 P，gradient 对应 q。
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(var_size, var_size);
  std::vector<double> gradient(var_size, 0.0);

  for (size_t i = 0; i < n; ++i)
  {
    const auto& point = reference.points[start + i];
    const size_t si = sIndex(i);
    const size_t vi = vIndex(n, i);
    const size_t ai = aIndex(n, i);

    // s/v 贴近规则粗解。s 项防止 QP 为了满足时间约束生成不贴合路径进度的怪解；
    // v 项让优化结果继承规则速度规划的“可行粗解”形状。
    // 对于 w * (x - x_ref)^2，展开为：
    //   w*x^2 - 2*w*x_ref*x + 常数
    // 在 OSQP 的 1/2*x^T P*x + q^T*x 形式下，需要写入：
    //   P += 2*w，q += -2*w*x_ref。
    hessian(si, si) += 2.0 * std::max(0.0, weight_s_ref_);
    gradient[si] += -2.0 * std::max(0.0, weight_s_ref_) * point.s;

    hessian(vi, vi) += 2.0 * std::max(0.0, weight_v_ref_);
    gradient[vi] += -2.0 * std::max(0.0, weight_v_ref_) * std::max(0.0, point.v);

    // 直接惩罚加速度，抑制急加速/急减速。
    // 这里没有参考加速度项，目标是让 a 尽量靠近 0。
    hessian(ai, ai) += 2.0 * std::max(0.0, weight_acc_);
  }

  for (size_t i = 0; i + 1 < n; ++i)
  {
    const size_t ai = aIndex(n, i);
    const size_t aj = aIndex(n, i + 1);
    const double dt = reference.points[start + i + 1].relative_time -
                      reference.points[start + i].relative_time;
    if (dt < min_dt_)
    {
      ROS_WARN_THROTTLE(1.0, "conflict velocity QP skipped: dt %.4f is too small", dt);
      return false;
    }

    // jerk 平滑项：(a[i+1] - a[i])^2 / dt。
    // 展开：
    //   w/dt * (a_i^2 - 2*a_i*a_{i+1} + a_{i+1}^2)
    // 因为 OSQP 目标函数前面有 1/2，所以矩阵里写入 2*w/dt 和 -2*w/dt。
    const double jerk_weight = std::max(0.0, weight_jerk_) / std::max(min_dt_, dt);
    hessian(ai, ai) += 2.0 * jerk_weight;
    hessian(aj, aj) += 2.0 * jerk_weight;
    hessian(ai, aj) += -2.0 * jerk_weight;
    hessian(aj, ai) += -2.0 * jerk_weight;
  }

  Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
  std::vector<double> lower_bound(constraint_size, 0.0);
  std::vector<double> upper_bound(constraint_size, 0.0);
  // row 指向当前正在填写的约束行。每填完一行就自增，最后检查是否等于 constraint_size，
  // 防止后续维护时漏填或多填约束。
  size_t row = 0;

  // 变量边界：s/v/a。初始节点直接固定到规则粗解，保证和拼接前缀连续。
  for (size_t i = 0; i < n; ++i, ++row)
  {
    const auto& point = reference.points[start + i];
    const double node_time = point.relative_time;
    const size_t si = sIndex(i);
    constraint_matrix(row, si) = 1.0;

    double lower_s = start_s;
    double upper_s = end_s + 1.0;
    if (has_entry_time_constraint &&
        std::isfinite(yield_entry_s) &&
        std::isfinite(target_entry_time_from_now) &&
        node_time + 1.0e-6 < target_entry_time_from_now)
    {
      // “不早于目标时间进入冲突区”转成线性空间约束：
      // 在目标时间之前的所有固定时间节点，都不能越过冲突入口。
      // 原始约束是时间-空间耦合的：车辆进入冲突区的时刻 >= target_entry_time。
      // 由于这里采用定时间节点，t_i 已知，因此可线性化为：
      //   若 t_i < target_entry_time，则 s_i <= yield_entry_s - margin。
      // 这也是“规则粗解 + 定时间 QP”的关键：把复杂的时空关系变成线性 s 边界。
      upper_s = std::min(upper_s, yield_entry_s - std::max(0.0, not_early_s_margin_));
    }

    if (i == 0)
    {
      // 第 0 个 QP 节点对应拼接段末点，必须固定，不能被优化器移动。
      lower_s = point.s;
      upper_s = point.s;
    }

    lower_bound[row] = lower_s;
    upper_bound[row] = std::max(lower_s, upper_s);
  }

  for (size_t i = 0; i < n; ++i, ++row)
  {
    const auto& point = reference.points[start + i];
    const size_t vi = vIndex(n, i);
    constraint_matrix(row, vi) = 1.0;

    const double curv_limit = curvatureSpeedLimit(point.kappa, max_lateral_acc_, max_speed_);
    double upper_v = std::min(max_speed_, curv_limit);
    if (std::isfinite(speed_cap))
    {
      // 决策超时等场景会传入 speed_cap。此时不继续使用旧冲突入口/旧时间，
      // 但通过全局低速上限保证车辆不会因为决策中断而恢复高速。
      upper_v = std::min(upper_v, std::max(0.0, speed_cap));
    }

    if (i == 0)
    {
      // 固定初始速度，保证与拼接段末点连续。
      lower_bound[row] = std::max(0.0, point.v);
      upper_bound[row] = std::max(0.0, point.v);
    }
    else
    {
      lower_bound[row] = 0.0;
      upper_bound[row] = std::max(0.0, upper_v);
    }
  }

  for (size_t i = 0; i < n; ++i, ++row)
  {
    const auto& point = reference.points[start + i];
    const size_t ai = aIndex(n, i);
    constraint_matrix(row, ai) = 1.0;
    if (i == 0)
    {
      // 固定初始加速度，避免在拼接边界产生加速度突变。
      lower_bound[row] = point.a;
      upper_bound[row] = point.a;
    }
    else
    {
      lower_bound[row] = std::min(min_acc_, max_acc_);
      upper_bound[row] = std::max(min_acc_, max_acc_);
    }
  }

  for (size_t i = 0; i + 1 < n; ++i, ++row)
  {
    const double dt = reference.points[start + i + 1].relative_time -
                      reference.points[start + i].relative_time;
    // s[i+1] = s[i] + v[i] * dt + 0.5 * a[i] * dt^2。
    // 写成 A*x = 0 的形式：
    //   s[i+1] - s[i] - dt*v[i] - 0.5*dt^2*a[i] = 0
    // 这是定时间域 QP 的位置递推约束。
    constraint_matrix(row, sIndex(i + 1)) = 1.0;
    constraint_matrix(row, sIndex(i)) = -1.0;
    constraint_matrix(row, vIndex(n, i)) = -dt;
    constraint_matrix(row, aIndex(n, i)) = -0.5 * dt * dt;
    lower_bound[row] = 0.0;
    upper_bound[row] = 0.0;
  }

  for (size_t i = 0; i + 1 < n; ++i, ++row)
  {
    const double dt = reference.points[start + i + 1].relative_time -
                      reference.points[start + i].relative_time;
    // v[i+1] = v[i] + a[i] * dt。
    // 写成 A*x = 0 的形式：
    //   v[i+1] - v[i] - dt*a[i] = 0
    // 该约束把速度和加速度变量绑定起来，避免出现“速度曲线平滑但加速度不一致”的结果。
    constraint_matrix(row, vIndex(n, i + 1)) = 1.0;
    constraint_matrix(row, vIndex(n, i)) = -1.0;
    constraint_matrix(row, aIndex(n, i)) = -dt;
    lower_bound[row] = 0.0;
    upper_bound[row] = 0.0;
  }

  if (row != constraint_size)
  {
    ROS_WARN("conflict velocity QP internal row mismatch: row=%zu expected=%zu", row, constraint_size);
    return false;
  }

  osqp::OSQPInterface qp_solver(eps_abs_);
  // 复用 velocity_planner 中已有的 OSQPInterface，而不是在冲突模块里重复手写
  // OSQP 的 CSC 矩阵构造和内存管理。
  qp_solver.updateMaxIter(max_iter_);
  qp_solver.updateEpsRel(eps_rel_);
  qp_solver.updateEpsAbs(eps_abs_);
  qp_solver.updateVerbose(verbose_);
  qp_solver.updateRhoInterval(0);

  const auto result = qp_solver.optimize(hessian, constraint_matrix, gradient, lower_bound, upper_bound);
  const auto& solution = std::get<0>(result);
  const int status = std::get<3>(result);
  if ((status != 1 && status != 2) || solution.size() != var_size)
  {
    // OSQP 状态 1/2 分别表示求解成功/有一定精度的成功。其他状态都视作失败。
    // 失败时不修改 trajectory，让调用方继续使用规则速度规划粗解。
    ROS_WARN_THROTTLE(1.0,
                      "conflict velocity QP failed, keep rule-based profile. status=%d size=%zu expected=%zu",
                      status,
                      solution.size(),
                      var_size);
    return false;
  }

  auto optimized_trajectory = trajectory;
  for (size_t i = 0; i < n; ++i)
  {
    const size_t global_index = start + i;
    const double opt_s = clampValue(solution[sIndex(i)], start_s, end_s);
    const double opt_v = std::max(0.0, solution[vIndex(n, i)]);
    const double opt_a = clampValue(solution[aIndex(n, i)], min_acc_, max_acc_);

    auto out = interpolatePointByS(reference, start, opt_s);
    // QP 优化了 s/v/a，但没有重新优化横向路径。这里用 opt_s 回到规则粗解路径上
    // 插值得到 x/y/theta/kappa，然后写回优化后的 v/a 和原固定时间戳。
    out.relative_time = reference.points[global_index].relative_time;
    out.v = opt_v;
    out.a = std::fabs(opt_a) < 1.0e-3 ? 0.0 : opt_a;
    optimized_trajectory.points[global_index] = out;
  }

  if (enable_output_validation_ &&
      !validateOptimizedTrajectory(optimized_trajectory,
                                   start,
                                   std::max(0.0, min_output_s_gap_),
                                   std::max(0.0, min_output_xy_gap_)))
  {
    ROS_WARN_THROTTLE(1.0,
                      "conflict velocity QP output rejected, keep rule-based profile. "
                      "reason=invalid_time_or_duplicate_points");
    return false;
  }

  trajectory = optimized_trajectory;
  return true;
}

}  // namespace conflict_prediction_resolution
