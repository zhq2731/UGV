#include "hybrid_a_star/liom_grid.h"
#include <algorithm>
#include <cmath>

namespace open_space {
namespace {
double Wrap(double angle) { return std::atan2(std::sin(angle),std::cos(angle)); }
}

LiomGrid::LiomGrid(const nav_msgs::OccupancyGrid &grid,const geometry_msgs::Pose2D &pose,
                   const OpenSpace_config &vehicle) : grid_(grid),vehicle_(vehicle) {
  resolution_ = grid.info.resolution;
  const auto &origin = grid.info.origin;
  const auto &q = origin.orientation;
  if (!std::isfinite(resolution_) || resolution_ <= 0 || grid.info.width == 0 || grid.info.height == 0 ||
      grid.data.size() != size_t(grid.info.width)*grid.info.height ||
      !std::isfinite(pose.x+pose.y+pose.theta+origin.position.x+origin.position.y) ||
      !std::isfinite(q.z+q.w) || std::abs(q.x) > 1e-8 || std::abs(q.y) > 1e-8 ||
      std::abs(q.z*q.z+q.w*q.w-1.0) > 1e-5 ||
      !std::isfinite(vehicle.car_length+vehicle.car_width+vehicle.rear_overhang+vehicle.search_collision_margin) ||
      vehicle.car_length <= 0 || vehicle.car_width <= 0 || vehicle.rear_overhang < 0 ||
      vehicle.rear_overhang >= vehicle.car_length || vehicle.search_collision_margin < 0) return;
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  world_x_ = pose.x+c*origin.position.x-s*origin.position.y;
  world_y_ = pose.y+s*origin.position.x+c*origin.position.y;
  yaw_ = pose.theta+2*std::atan2(q.z,q.w);
  const int stride = grid.info.width+1;
  occupied_prefix_.assign(size_t(stride)*(grid.info.height+1),0);
  for (unsigned y = 0; y < grid.info.height; ++y) {
    int row = 0;
    for (unsigned x = 0; x < grid.info.width; ++x) {
      row += grid.data[y*grid.info.width+x] != 25;
      occupied_prefix_[(y+1)*stride+x+1] = occupied_prefix_[y*stride+x+1]+row;
    }
  }
  valid_ = true;
}

planning_msgs::TrajectoryPointArray LiomGrid::ToGrid(const planning_msgs::TrajectoryPointArray &path) const {
  auto out = path;
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  for (auto &p : out.points) {
    const double dx = p.x-world_x_, dy = p.y-world_y_;
    p.x = c*dx+s*dy; p.y = -s*dx+c*dy;
    p.theta = Wrap(p.theta-yaw_);
  }
  return out;
}

planning_msgs::TrajectoryPointArray LiomGrid::ToWorld(const planning_msgs::TrajectoryPointArray &path) const {
  auto out = path;
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  for (auto &p : out.points) {
    const double x = p.x, y = p.y;
    p.x = world_x_+c*x-s*y; p.y = world_y_+s*x+c*y;
    p.theta = Wrap(p.theta+yaw_);
  }
  return out;
}

bool LiomGrid::BoxFree(const LiomBox &box) const {
  if (box.xmin <= 0 || box.ymin <= 0 || box.xmax >= grid_.info.width*resolution_ ||
      box.ymax >= grid_.info.height*resolution_) return false;
  const int x0 = std::floor((box.xmin-1e-9)/resolution_), y0 = std::floor((box.ymin-1e-9)/resolution_);
  const int x1 = std::floor((box.xmax+1e-9)/resolution_), y1 = std::floor((box.ymax+1e-9)/resolution_);
  if (x0 < 0 || y0 < 0 || x1 >= int(grid_.info.width) || y1 >= int(grid_.info.height)) return false;
  const int stride = grid_.info.width+1;
  return occupied_prefix_[(y1+1)*stride+x1+1]-occupied_prefix_[y0*stride+x1+1]
       - occupied_prefix_[(y1+1)*stride+x0]+occupied_prefix_[y0*stride+x0] == 0;
}

bool LiomGrid::BuildCorridors(const planning_msgs::TrajectoryPointArray &path,
                              std::vector<LiomCorridor> &corridors) const {
  if (!valid_ || path.points.empty()) return false;
  corridors.clear();
  // 数值缓冲小于一格；0.4 m 等配置余量保持不变，点间安全另做扫掠复核。
  const double padding = 0.25*resolution_;
  const double margin = vehicle_.search_collision_margin+padding;
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto &p = path.points[i];
    LiomBox box{-vehicle_.rear_overhang-margin,-vehicle_.car_width/2-margin,
                vehicle_.car_length-vehicle_.rear_overhang+margin,vehicle_.car_width/2+margin};
    if (!OrientedBoxFree(box,p.x,p.y,p.theta)) {
      // 贴边初值允许在相邻一格找安全框，交给优化器修正；固定端点不移动。
      if (i == 0 || i+1 == path.points.size()) return false;
      bool found = false;
      for (const auto &shift : {std::array<int,2>{1,0},{-1,0},{0,1},{0,-1},
                                {1,1},{1,-1},{-1,1},{-1,-1}}) {
        const double dx = shift[0]*resolution_, dy = shift[1]*resolution_;
        const LiomBox moved{box.xmin+dx,box.ymin+dy,box.xmax+dx,box.ymax+dy};
        if (!OrientedBoxFree(moved,p.x,p.y,p.theta)) continue;
        box = moved; found = true; break;
      }
      if (!found) return false;
    }
    // 矩形沿初值航向扩展，整块区域必须无障碍，不只检查四个角点。
    // 四边轮流各扩一格，避免先扩满一侧把其他方向的空间挤掉；每侧仍最多 1 m。
    std::array<bool,4> blocked{};
    for (int step = 0; step < int(std::ceil(1.0/resolution_)); ++step) {
      for (int axis = 0; axis < 4; ++axis) {
        if (blocked[axis]) continue;
        auto expanded = box;
        if (axis == 0) expanded.xmin -= resolution_;
        if (axis == 1) expanded.ymin -= resolution_;
        if (axis == 2) expanded.xmax += resolution_;
        if (axis == 3) expanded.ymax += resolution_;
        if (!OrientedBoxFree(expanded,p.x,p.y,p.theta)) blocked[axis] = true;
        else box = expanded;
      }
    }
    corridors.push_back({box,p.x-path.points.front().x,p.y-path.points.front().y,p.theta,padding});
  }
  return true;
}

bool LiomGrid::OrientedBoxFree(const LiomBox &box,double x,double y,double theta) const {
  const double hx = (box.xmax-box.xmin)/2, hy = (box.ymax-box.ymin)/2;
  const double mx = (box.xmax+box.xmin)/2, my = (box.ymax+box.ymin)/2;
  const double c = std::cos(theta), s = std::sin(theta);
  const double cx = x+mx*c-my*s, cy = y+mx*s+my*c;
  const double ex = hx*std::abs(c)+hy*std::abs(s), ey = hx*std::abs(s)+hy*std::abs(c);
  const LiomBox bounds{cx-ex,cy-ey,cx+ex,cy+ey};
  if (bounds.xmin <= 0 || bounds.ymin <= 0 || bounds.xmax >= grid_.info.width*resolution_ ||
      bounds.ymax >= grid_.info.height*resolution_) return false;
  if (BoxFree(bounds)) return true;
  const double half_cell = resolution_/2;
  // 分离轴判定矩形车身与整块栅格相交，包含车内小障碍及仅部分重叠的栅格。
  for (int gy = std::max(0,int(std::floor((bounds.ymin-1e-9)/resolution_))); gy <= int(std::floor(bounds.ymax/resolution_)); ++gy) {
    for (int gx = std::max(0,int(std::floor((bounds.xmin-1e-9)/resolution_))); gx <= int(std::floor(bounds.xmax/resolution_)); ++gx) {
      if (grid_.data[gy*grid_.info.width+gx] == 25) continue;
      const double dx = (gx+0.5)*resolution_-cx, dy = (gy+0.5)*resolution_-cy;
      if (std::abs(dx) <= ex+half_cell && std::abs(dy) <= ey+half_cell &&
          std::abs(c*dx+s*dy) <= hx+half_cell*(std::abs(c)+std::abs(s)) &&
          std::abs(-s*dx+c*dy) <= hy+half_cell*(std::abs(c)+std::abs(s))) return false;
    }
  }
  return true;
}

bool LiomGrid::PoseFree(double x,double y,double theta,double sweep_padding) const {
  const double margin = vehicle_.search_collision_margin+sweep_padding;
  return OrientedBoxFree({-vehicle_.rear_overhang-margin,-vehicle_.car_width/2-margin,
    vehicle_.car_length-vehicle_.rear_overhang+margin,vehicle_.car_width/2+margin},x,y,theta);
}

bool LiomGrid::TrajectoryFree(const planning_msgs::TrajectoryPointArray &path) const {
  if (!valid_ || path.points.empty()) return false;
  for (const auto &p : path.points)
    if (!std::isfinite(p.x+p.y+p.theta)) return false;
  const double margin = vehicle_.search_collision_margin;
  const double radius = std::hypot(std::max(vehicle_.rear_overhang,vehicle_.car_length-vehicle_.rear_overhang)+margin,
                                   vehicle_.car_width/2+margin);
  if (!PoseFree(path.points.front().x,path.points.front().y,path.points.front().theta,0)) return false;
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto &a = path.points[i-1], &b = path.points[i];
    if (!PoseFree(b.x,b.y,b.theta,0)) return false;
    const double angle = Wrap(b.theta-a.theta);
    const double displacement = std::hypot(b.x-a.x,b.y-a.y)+radius*std::abs(angle);
    const int steps = std::max(1,int(std::ceil(displacement/(0.25*resolution_))));
    // 补偿两个检查点间的最大车身运动，保守覆盖线性位姿插值的扫掠，而非只查节点。
    const double padding = displacement/(2*steps);
    for (int k = 0; k <= steps; ++k) {
      const double t = double(k)/steps;
      if (!PoseFree(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.theta+t*angle,padding)) return false;
    }
  }
  return true;
}
}  // namespace open_space
