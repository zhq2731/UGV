// Copyright 2018-2021 The Autoware Foundation
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

#include "trajectory_follower/qp_solver/qp_solver_osqp.hpp"

#include <ros/ros.h>

#include <string>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
QPSolverOSQP::QPSolverOSQP() {}
bool QPSolverOSQP::solve(
  const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
  const Eigen::VectorXd & ub_a, Eigen::VectorXd & u)
{
  const Eigen::Index raw_a = a.rows();
  const Eigen::Index col_a = a.cols();
  const Eigen::Index dim_u = ub.size();
  Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(dim_u, dim_u);

  // convert matrix to vector for osqpsolver
  std::vector<double> f(&f_vec(0), f_vec.data() + f_vec.cols() * f_vec.rows());

  std::vector<double> lower_bound;
  std::vector<double> upper_bound;

  for (int i = 0; i < dim_u; ++i) {
    lower_bound.push_back(lb(i));
    upper_bound.push_back(ub(i));
  }

  for (int i = 0; i < col_a; ++i) {
    lower_bound.push_back(lb_a(i));
    upper_bound.push_back(ub_a(i));
  }

  Eigen::MatrixXd osqpA = Eigen::MatrixXd(dim_u + col_a, raw_a);
  osqpA << Identity, a;

  /* execute optimization */
  auto result = osqpsolver_.optimize(h_mat, osqpA, f, lower_bound, upper_bound);

  const int status_val = std::get<3>(result);
  // OSQP 的主求解状态决定本次优化是否可用。SOLVED_INACCURATE 仍然提供
  // 满足工程容差的主解，允许 MPC 使用；其余状态必须明确返回失败，不能再由
  // polish 状态误判为成功。
  constexpr int kOsqpSolved = 1;
  constexpr int kOsqpSolvedInaccurate = 2;
  if (status_val != kOsqpSolved &&
    status_val != kOsqpSolvedInaccurate)
  {
    // OSQP 连续失败时只限频输出，避免控制周期刷屏。
    ROS_WARN_THROTTLE(2.0, "[mpc] OSQP status: %s",
      osqpsolver_.getStatusMessage().c_str());
    return false;
  }

  const std::vector<double> & osqp_solution = std::get<0>(result);
  if (osqp_solution.size() != static_cast<size_t>(dim_u)) {
    ROS_WARN_THROTTLE(
      2.0, "[mpc] OSQP returned invalid solution size: %zu, expected: %ld",
      osqp_solution.size(), static_cast<long>(dim_u));
    return false;
  }

  u = Eigen::Map<const Eigen::VectorXd>(
    osqp_solution.data(), static_cast<Eigen::Index>(osqp_solution.size()));
  if (!u.allFinite()) {
    ROS_WARN_THROTTLE(2.0, "[mpc] OSQP returned non-finite solution");
    return false;
  }

  // polish 只是对已经成功的主解做精修：1=成功、0=未执行、-1=失败。
  // 即使精修失败，主解仍然有效，不能因此让横向控制回退为上一转角命令。
  return true;
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
