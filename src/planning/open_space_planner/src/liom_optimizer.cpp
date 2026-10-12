// Based on yakunouyang/liom_local_planner, commit 385cbae86293ee9437ba539167ce5403fd387365.
// Copyright (c) 2023, Yakun Ouyang and Bai Li. BSD-3-Clause, see LICENSE.LIOM.
#include "hybrid_a_star/liom_optimizer.h"
#include "hybrid_a_star/hybrid_a_star.h"
#include "hybrid_a_star/liom_grid.h"

#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include <adolc/adolc.h>
#include <adolc/adolc_sparse.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <vector>

namespace open_space {
namespace {
constexpr int kStates = 7;  // x, y, 车身航向, 有符号速度, 前轮角, 加速度, 转角速率。
constexpr short kTape = 25101;
constexpr double kInfinity = 1e19;
constexpr double kResidualLimit = 1e-3;
constexpr double kOptimizedCollisionMargin = 0.2;  // 优化结果交付前的基础复核余量，单位 m。
std::timed_mutex solver_mutex;  // ADOL-C tape 共享；预算也涵盖等待其他求解的时间。
using Clock = std::chrono::steady_clock;

double MillisecondsSince(Clock::time_point started) {
  return std::chrono::duration<double,std::milli>(Clock::now()-started).count();
}

Clock::time_point Deadline(Clock::time_point started,const LiomOptions &options) {
  return std::isfinite(options.max_runtime_ms) && options.max_runtime_ms > 0 ?
    started+std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double,std::milli>(options.max_runtime_ms)) : Clock::time_point::max();
}

double WrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

class LiomProblem : public Ipopt::TNLP {
public:
  LiomProblem(std::vector<double> guess, const OpenSpace_config &vehicle,
              bool forward, double penalty,int count,const std::vector<LiomCorridor> &corridors,
              Clock::time_point deadline)
      : result(std::move(guess)), vehicle_(vehicle), forward_(forward), penalty_(penalty),
        count_(count), lower_(result.size(), -kInfinity),
        upper_(result.size(), kInfinity), corridors_(corridors), deadline_(deadline) {
    lower_[0] = 0.1;
    const double speed = forward_ ? vehicle_.forward_speed : vehicle_.reverse_speed;
    // 用最高速度得到保守横向加速度限角；当前低速参数下通常由机械限角决定。
    const double steering = std::min(vehicle_.steering_angle*M_PI/180.0,
      std::atan(vehicle_.max_lateral_acceleration*vehicle_.wheel_base/(speed*speed)));
    for (int i = 0; i < count_; ++i) {
      Bound(i, 3, forward_ ? 0.0 : -speed, forward_ ? speed : 0.0);
      Bound(i, 4, -steering, steering);
      Bound(i, 5, forward_ ? -vehicle_.max_deceleration : -vehicle_.max_acceleration,
                   forward_ ? vehicle_.max_acceleration : vehicle_.max_deceleration);
      Bound(i, 6, -vehicle_.max_front_tire_steering_rate, vehicle_.max_front_tire_steering_rate);
    }
    for (int i : {0, count_-1}) {
      for (int j = 0; j < 3; ++j) Bound(i, j, result[Index(i,j)], result[Index(i,j)]);
      Bound(i, 3, 0.0, 0.0);
    }
    if (!corridors_.empty()) {
      const int size = 1+kStates*count_+2*kLiomCorners*count_;
      result.resize(size); lower_.resize(size,-kInfinity); upper_.resize(size,kInfinity);
      for (int i = 0; i < count_; ++i) {
        for (int j = 0; j < kLiomCorners; ++j) {
          const auto &box = corridors_[i].box;
          const int index = CornerIndex(i,j);
          const auto corner = CornerPosition(result.data(),i,j);
          result[index] = corner[0]; result[index+1] = corner[1];
          lower_[index] = box.xmin; upper_[index] = box.xmax;
          lower_[index+1] = box.ymin; upper_[index+1] = box.ymax;
        }
      }
    }
    // 段端转角不固定为零；停车后的调轮由现有执行状态机负责。
    for (size_t j = 0; j < result.size(); ++j)
      result[j] = std::clamp(result[j], lower_[j], upper_[j]);
  }

  ~LiomProblem() override {
    std::free(rows_); std::free(cols_); std::free(hessian_);
  }

  static int Index(int i, int j) { return 1+i*kStates+j; }

  int CornerIndex(int i,int j) const { return 1+kStates*count_+2*(i*kLiomCorners+j); }
  template<class T> std::array<T,2> CornerPosition(const T *x,int i,int j) const {
    const auto &corridor = corridors_[i];
    const double margin = vehicle_.search_collision_margin+corridor.padding;
    const double longitudinal = (j == 1 || j == 2) ? vehicle_.car_length-vehicle_.rear_overhang+margin :
                                                   -vehicle_.rear_overhang-margin;
    const double lateral = (j < 2 ? 1 : -1)*(vehicle_.car_width/2+margin);
    const T angle = x[Index(i,2)]-corridor.theta;
    const T dx = x[Index(i,0)]-corridor.x, dy = x[Index(i,1)]-corridor.y;
    // 在走廊的固定局部坐标中表达角点，仍然只使用变量上下界硬约束。
    return {{dx*std::cos(corridor.theta)+dy*std::sin(corridor.theta)+longitudinal*cos(angle)-lateral*sin(angle),
             -dx*std::sin(corridor.theta)+dy*std::cos(corridor.theta)+longitudinal*sin(angle)+lateral*cos(angle)}};
  }
  template<class T> std::array<T,2> CornerResidual(const T *x,int i,int j) const {
    const int index = CornerIndex(i,j);
    const auto corner = CornerPosition(x,i,j);
    return {{x[index]-corner[0],x[index+1]-corner[1]}};
  }

  template<class T> std::array<T, 5> Residual(const T *x, int i) const {
    const T dt = x[0]/(count_-1);
    const T *p = x+Index(i,0), *q = x+Index(i+1,0);
    // 梯形离散允许从零速正常起步，避免前向欧拉产生首个重复位置点。
    return {{q[0]-p[0]-dt*(p[3]*cos(p[2])+q[3]*cos(q[2]))/2,
             q[1]-p[1]-dt*(p[3]*sin(p[2])+q[3]*sin(q[2]))/2,
             q[2]-p[2]-dt*(p[3]*tan(p[4])+q[3]*tan(q[4]))/(2*vehicle_.wheel_base),
             q[3]-p[3]-dt*(p[5]+q[5])/2,
             q[4]-p[4]-dt*(p[6]+q[6])/2}};
  }

  template<class T> T Objective(const T *x) const {
    T cost = x[0];
    for (int i = 0; i < count_; ++i) {
      cost += x[Index(i,5)]*x[Index(i,5)]+x[Index(i,6)]*x[Index(i,6)];
      if (i+1 < count_)
        for (const T &r : Residual(x,i)) cost += penalty_*r*r;
      if (!corridors_.empty())
        for (int j = 0; j < kLiomCorners; ++j)
          for (const T &r : CornerResidual(x,i,j)) cost += penalty_*r*r;
    }
    return cost;
  }

  bool Feasible(LiomReport &report) const {
    report.residual_squared = 0;
    report.max_residual = 0;
    if (!finalized_) return false;
    for (size_t i = 0; i < result.size(); ++i)
      if (!std::isfinite(result[i]) || result[i] < lower_[i]-1e-7 || result[i] > upper_[i]+1e-7)
        return false;
    for (int i = 0; i+1 < count_; ++i) {
      for (double r : Residual(result.data(), i)) {
        report.residual_squared += r*r;
        report.max_residual = std::max(report.max_residual, std::abs(r));
      }
    }
    for (int i = 0; i < count_ && !corridors_.empty(); ++i) {
      for (int j = 0; j < kLiomCorners; ++j) {
        for (double r : CornerResidual(result.data(),i,j)) {
          report.residual_squared += r*r;
          report.max_residual = std::max(report.max_residual,std::abs(r));
        }
      }
    }
    return report.residual_squared <= 1e-4 && report.max_residual <= kResidualLimit;
  }

  bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &jac,
                    Ipopt::Index &hess, IndexStyleEnum &style) override {
    n = result.size(); m = 0; jac = 0; style = C_STYLE;
    std::vector<adouble> active(n);
    trace_on(kTape);
    for (int i = 0; i < n; ++i) active[i] <<= result[i];
    adouble value = Objective(active.data());
    double ignored;
    value >>= ignored;
    trace_off();
    int options[2] = {0, 1};
    if (sparse_hess(kTape,n,0,result.data(),&nnz_,&rows_,&cols_,&hessian_,options) < 0) return false;
    hess = nnz_;
    return true;
  }

  bool get_bounds_info(Ipopt::Index n, double *lower, double *upper,
                        Ipopt::Index, double *, double *) override {
    std::copy(lower_.begin(),lower_.end(),lower);
    std::copy(upper_.begin(),upper_.end(),upper);
    return true;
  }
  bool get_starting_point(Ipopt::Index, bool, double *x, bool, double *, double *,
                          Ipopt::Index, bool, double *) override {
    std::copy(result.begin(),result.end(),x);
    return true;
  }
  bool eval_f(Ipopt::Index, const double *x, bool, double &f) override {
    f = Objective(x); return std::isfinite(f);
  }
  bool eval_grad_f(Ipopt::Index n, const double *x, bool, double *grad) override {
    return gradient(kTape,n,x,grad) >= 0;
  }
  bool eval_g(Ipopt::Index,const double *,bool,Ipopt::Index,double *) override { return true; }
  bool eval_jac_g(Ipopt::Index,const double *,bool,Ipopt::Index,Ipopt::Index,
                  Ipopt::Index *,Ipopt::Index *,double *) override { return true; }
  bool eval_h(Ipopt::Index n, const double *x, bool, double scale, Ipopt::Index,
               const double *, bool, Ipopt::Index, Ipopt::Index *rows,
               Ipopt::Index *cols, double *values) override {
    if (!values) {
      for (int i = 0; i < nnz_; ++i) { rows[i] = rows_[i]; cols[i] = cols_[i]; }
    } else {
      int options[2] = {0,1};
      if (sparse_hess(kTape,n,1,const_cast<double *>(x),&nnz_,&rows_,&cols_,&hessian_,options) < 0)
        return false;
      for (int i = 0; i < nnz_; ++i) values[i] = scale*hessian_[i];
    }
    return true;
  }
  void finalize_solution(Ipopt::SolverReturn,Ipopt::Index n,const double *x,
      const double *,const double *,Ipopt::Index,const double *,const double *,double,
      const Ipopt::IpoptData *,Ipopt::IpoptCalculatedQuantities *) override {
    result.assign(x,x+n);
    finalized_ = true;
  }

  bool intermediate_callback(Ipopt::AlgorithmMode,Ipopt::Index,double,double,double,double,
      double,double,double,double,Ipopt::Index,const Ipopt::IpoptData *,
      Ipopt::IpoptCalculatedQuantities *) override {
    // 迭代边界协作终止，不能强制中断一次正在运行的导数计算或矩阵分解。
    return Clock::now() < deadline_;
  }

  std::vector<double> result;
private:
  void Bound(int i,int j,double lower,double upper) {
    lower_[Index(i,j)] = lower; upper_[Index(i,j)] = upper;
  }
  OpenSpace_config vehicle_;
  bool forward_;
  double penalty_;
  int count_;
  std::vector<double> lower_, upper_;
  std::vector<LiomCorridor> corridors_;
  Clock::time_point deadline_;
  bool finalized_ = false;
  int nnz_ = 0;
  unsigned int *rows_ = nullptr, *cols_ = nullptr;
  double *hessian_ = nullptr;
};

// 原轨迹按距离采样；转为等时间采样并展开航向角，避免 ±π 附近插值跳变。
std::vector<double> MakeGuess(const planning_msgs::TrajectoryPointArray &seed,
                              double wheel_base) {
  const auto &points = seed.points;
  const int n = points.size();
  const double direction = seed.is_forward_shift ? 1.0 : -1.0;
  std::vector<double> x(1+kStates*n), heading(n);
  heading[0] = points[0].theta;
  for (int i = 1; i < n; ++i)
    heading[i] = heading[i-1]+WrapAngle(points[i].theta-points[i-1].theta);
  x[0] = points.back().relative_time-points.front().relative_time;
  size_t interval = 0;
  for (int i = 0; i < n; ++i) {
    const double t = points.front().relative_time+x[0]*i/(n-1);
    while (interval+2 < points.size() && points[interval+1].relative_time < t) ++interval;
    const auto &p = points[interval], &q = points[interval+1];
    const double ratio = (t-p.relative_time)/(q.relative_time-p.relative_time);
    const auto lerp = [ratio](double a,double b) { return a+ratio*(b-a); };
    double *state = x.data()+LiomProblem::Index(i,0);
    // 平移到起点附近，避免世界坐标很大时损失数值精度。
    state[0] = lerp(p.x,q.x)-points.front().x;
    state[1] = lerp(p.y,q.y)-points.front().y;
    state[2] = lerp(heading[interval],heading[interval+1]);
    state[3] = lerp(p.v,q.v);
    state[4] = std::atan(wheel_base*direction*lerp(p.kappa,q.kappa));
    state[5] = lerp(p.a,q.a);
  }
  for (int i = 0; i+1 < n; ++i)
    x[LiomProblem::Index(i,6)] = (x[LiomProblem::Index(i+1,4)]-x[LiomProblem::Index(i,4)])/(x[0]/(n-1));
  return x;
}

planning_msgs::TrajectoryPointArray ToMessage(const std::vector<double> &x,
    const planning_msgs::TrajectoryPointArray &seed,double wheel_base) {
  auto output = seed;
  const double direction = seed.is_forward_shift ? 1.0 : -1.0;
  const int n = output.points.size();
  for (int i = 0; i < n; ++i) {
    auto &p = output.points[i];
    const double *state = x.data()+LiomProblem::Index(i,0);
    p.x = state[0]+seed.points.front().x;
    p.y = state[1]+seed.points.front().y;
    p.theta = WrapAngle(state[2]);
    p.v = state[3]; p.a = state[5];
    p.kappa = direction*std::tan(state[4])/wheel_base;
    p.relative_time = x[0]*i/(n-1);
    p.s = i == 0 ? 0.0 : output.points[i-1].s+
      std::hypot(p.x-output.points[i-1].x,p.y-output.points[i-1].y);
  }
  for (int i = 0; i < n; ++i) {
    const auto &a = output.points[std::max(0,i-1)], &b = output.points[std::min(n-1,i+1)];
    output.points[i].dkappa = b.s-a.s > 1e-6 ? (b.kappa-a.kappa)/(b.s-a.s) : 0.0;
  }
  return output;
}
}  // namespace

static bool OptimizeSegment(const planning_msgs::TrajectoryPointArray &seed,
    const OpenSpace_config &vehicle,planning_msgs::TrajectoryPointArray &output,
    LiomReport &report,const LiomOptions &options,const LiomGrid *grid,Clock::time_point deadline) {
  report = LiomReport{};
  const auto started = Clock::now();
  const auto finish = [&](bool success,const std::string &message) {
    report.message = message;
    report.elapsed_ms = MillisecondsSince(started);
    return success;
  };
  const auto timeout = [&]() {
    report.timed_out = true;
    return finish(false,"LIOM wall-time budget exceeded; output unchanged");
  };
  const double speed = seed.is_forward_shift ? vehicle.forward_speed : vehicle.reverse_speed;
  if (seed.points.size() < 3 || !(vehicle.wheel_base > 0) || !(speed > 0) ||
      !(vehicle.max_acceleration > 0) || !(vehicle.max_deceleration > 0) ||
      !(vehicle.max_front_tire_steering_rate > 0) || !(vehicle.max_lateral_acceleration > 0) ||
      !(vehicle.steering_angle > 0 && vehicle.steering_angle < 90) ||
      options.max_iterations < 1 || options.max_penalty_iterations < 1 ||
      !std::isfinite(options.initial_penalty) || options.initial_penalty <= 0 ||
      !std::isfinite(options.max_runtime_ms) || options.max_runtime_ms < 0)
    return finish(false,"invalid segment, vehicle limits or solver options");
  for (size_t i = 0; i < seed.points.size(); ++i) {
    const auto &p = seed.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta) ||
        !std::isfinite(p.kappa) || !std::isfinite(p.v) || !std::isfinite(p.a) ||
        !std::isfinite(p.relative_time) ||
        (i > 0 && p.relative_time <= seed.points[i-1].relative_time))
      return finish(false,"seed must be finite with strictly increasing time");
  }
  if (std::abs(seed.points.front().v) > 1e-6 || std::abs(seed.points.back().v) > 1e-6)
    return finish(false,"this stage only supports stop-to-stop segments");

  if (Clock::now() >= deadline) return timeout();
  std::unique_lock<std::timed_mutex> lock(solver_mutex,std::defer_lock);
  if (deadline == Clock::time_point::max()) lock.lock();
  else if (!lock.try_lock_until(deadline)) return timeout();
  auto guess = MakeGuess(seed,vehicle.wheel_base);
  std::vector<LiomCorridor> corridors;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetIntegerValue("print_level",0);
  app->Options()->SetStringValue("sb","yes");
  app->Options()->SetStringValue("linear_solver","mumps");
  app->Options()->SetIntegerValue("max_iter",options.max_iterations);
  app->Options()->SetNumericValue("bound_relax_factor",0.0);
  app->Options()->SetNumericValue("tol",1e-6);
  if (app->Initialize() != Ipopt::Solve_Succeeded)
    return finish(false,"IPOPT initialization failed");
  double penalty = options.initial_penalty;
  for (int outer = 0; outer < options.max_penalty_iterations; ++outer) {
    if (Clock::now() >= deadline) return timeout();
    if (grid) {
      const auto corridor_started = Clock::now();
      // 首轮围绕等时间初值建框，后续围绕上一轮的新位姿和航向重建；地图前缀和复用。
      // 初值点间余量不足仍允许优化修正，但建框失败时不沿用旧框继续求解。
      const auto sampled = ToMessage(guess,seed,vehicle.wheel_base);
      const bool built = grid->BuildCorridors(sampled,corridors);
      ++report.corridor_builds;
      report.corridor_ms += MillisecondsSince(corridor_started);
      if (Clock::now() >= deadline) return timeout();
      if (!built)
        return finish(false,outer == 0 ? "cannot build conservative safe corridors for resampled seed" :
                                       "cannot rebuild safe corridors around previous solution; output unchanged");
    }
    const auto solver_started = Clock::now();
    // 新框可能有新的局部坐标系，构造问题时重新计算角点辅助量，不能直接复用旧坐标。
    Ipopt::SmartPtr<LiomProblem> problem = new LiomProblem(guess,vehicle,seed.is_forward_shift,penalty,
                                                        seed.points.size(),corridors,deadline);
    const auto status = app->OptimizeTNLP(problem);
    report.solver_ms += MillisecondsSince(solver_started);
    report.solver_status = static_cast<int>(status);
    report.penalty_iterations = outer+1;
    if (Clock::now() >= deadline) return timeout();
    const bool feasible = problem->Feasible(report);
    // 迭代耗尽不能当作成功；优化收敛且模型残差达标后，才交付结果。
    if (status != Ipopt::Solve_Succeeded && status != Ipopt::Solved_To_Acceptable_Level)
      return finish(false,"IPOPT did not converge; output unchanged");
    if (feasible) {
      auto candidate = ToMessage(problem->result,seed,vehicle.wheel_base);
      if (grid) {
        const auto collision_started = Clock::now();
        // 仅优化结果复核改为 0.2 m；走廊及搜索回退仍使用搜索余量，并保留点间扫掠。
        const bool safe = grid->TrajectoryFree(candidate,kOptimizedCollisionMargin);
        report.collision_ms += MillisecondsSince(collision_started);
        if (Clock::now() >= deadline) return timeout();
        if (!safe) return finish(false,"optimized footprint/sweep collides; output unchanged");
      }
      if (Clock::now() >= deadline) return timeout();
      output = std::move(candidate);
      return finish(true,grid ? "grid-constrained segment accepted" :
                               "free-space segment accepted; collision checking not implemented");
    }
    guess = problem->result;  // 下一轮继承状态和段时间，不再重新采样原搜索轨迹。
    penalty *= 10.0;  // LIOM 外层惩罚递增，不是增加 IPOPT 内层迭代次数。
  }
  return finish(false,"kinematic/footprint residual exceeds tolerance; output unchanged");
}

bool OptimizeLiomFreeSpaceSegment(const planning_msgs::TrajectoryPointArray &seed,
    const OpenSpace_config &vehicle,planning_msgs::TrajectoryPointArray &output,
    LiomReport &report,const LiomOptions &options) {
  return OptimizeSegment(seed,vehicle,output,report,options,nullptr,Deadline(Clock::now(),options));
}

bool OptimizeLiomParkingSegments(const std::vector<planning_msgs::TrajectoryPointArray> &seeds,
    const nav_msgs::OccupancyGrid &map,const geometry_msgs::Pose2D &grid_pose,
    const OpenSpace_config &vehicle,std::vector<planning_msgs::TrajectoryPointArray> &output,
    LiomReport &report,const LiomOptions &options) {
  report = LiomReport{};
  const auto started = Clock::now();
  const auto deadline = Deadline(started,options);
  const auto finish = [&](bool ok,const std::string &message) {
    report.message = message;
    report.elapsed_ms = MillisecondsSince(started);
    return ok;
  };
  const auto timeout = [&]() {
    report.timed_out = true;
    return finish(false,"LIOM wall-time budget exceeded; output unchanged");
  };
  if (!std::isfinite(options.max_runtime_ms) || options.max_runtime_ms < 0)
    return finish(false,"invalid runtime budget");
  if (Clock::now() >= deadline) return timeout();
  LiomGrid grid(map,grid_pose,vehicle);
  if (Clock::now() >= deadline) return timeout();
  if (!grid.valid() || seeds.empty()) return finish(false,"invalid grid snapshot or empty segments");
  // 先检查所有换挡连接。固定这些位姿，各段优化不跨段约束曲率相等。
  for (size_t i = 0; i < seeds.size(); ++i) {
    report.failed_segment = i;
    if (seeds[i].points.size() < 3) return finish(false,"segment has fewer than three points");
    if (i == 0) continue;
    const auto &a = seeds[i-1].points.back(), &b = seeds[i].points.front();
    if (seeds[i].header.frame_id != seeds[0].header.frame_id ||
        seeds[i].is_forward_shift == seeds[i-1].is_forward_shift ||
        !std::isfinite(a.x+a.y+a.theta+b.x+b.y+b.theta) ||
        std::hypot(a.x-b.x,a.y-b.y) > 1e-6 || std::abs(WrapAngle(a.theta-b.theta)) > 1e-6)
      return finish(false,"segments must share cusp pose and alternate direction");
  }
  std::vector<planning_msgs::TrajectoryPointArray> candidate;
  for (size_t i = 0; i < seeds.size(); ++i) {
    planning_msgs::TrajectoryPointArray optimized;
    LiomReport segment_report;
    const bool solved = OptimizeSegment(grid.ToGrid(seeds[i]),vehicle,optimized,segment_report,options,&grid,deadline);
    // 成功和失败都累计已发生的工作，避免后段失败抹掉前段耗时。
    report.solver_status = segment_report.solver_status;
    report.timed_out = segment_report.timed_out;
    report.max_residual = std::max(report.max_residual,segment_report.max_residual);
    report.residual_squared += segment_report.residual_squared;
    report.penalty_iterations += segment_report.penalty_iterations;
    report.corridor_builds += segment_report.corridor_builds;
    report.corridor_ms += segment_report.corridor_ms;
    report.solver_ms += segment_report.solver_ms;
    report.collision_ms += segment_report.collision_ms;
    report.failed_segment = i;
    if (!solved) return finish(false,segment_report.message);
    candidate.push_back(grid.ToWorld(optimized));
  }
  // 全部成功才交付，后段失败时不会留下前半条可被误用的输出。
  if (Clock::now() >= deadline) return timeout();
  output = std::move(candidate);
  report.failed_segment = -1;
  return finish(true,"all segments accepted against one grid snapshot");
}
}  // namespace open_space
