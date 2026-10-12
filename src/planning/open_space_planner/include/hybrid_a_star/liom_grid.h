#pragma once

#include "hybrid_a_star/hybrid_a_star.h"
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <array>

namespace open_space {
constexpr int kLiomCorners = 4;
struct LiomBox { double xmin, ymin, xmax, ymax; };
// 四个外扩车身角点共用一个无障碍凸矩形；坐标轴沿初值车身方向。
struct LiomCorridor { LiomBox box; double x, y, theta, padding; };

// 地图快照辅助类。内部坐标沿栅格坐标轴，以栅格原点为零点，单位 m。
// grid_pose 是地图局部坐标系在轨迹世界坐标系中的位姿，不是每段起点。
class LiomGrid {
public:
  LiomGrid(const nav_msgs::OccupancyGrid &grid,const geometry_msgs::Pose2D &grid_pose,
           const OpenSpace_config &vehicle);
  bool valid() const { return valid_; }
  planning_msgs::TrajectoryPointArray ToGrid(const planning_msgs::TrajectoryPointArray &path) const;
  planning_msgs::TrajectoryPointArray ToWorld(const planning_msgs::TrajectoryPointArray &path) const;
  // 默认沿用搜索余量；优化结果交付时可单独指定复核余量，扫掠补偿仍保留。
  bool TrajectoryFree(const planning_msgs::TrajectoryPointArray &grid_path) const;
  bool TrajectoryFree(const planning_msgs::TrajectoryPointArray &grid_path,double collision_margin) const;
  bool BuildCorridors(const planning_msgs::TrajectoryPointArray &grid_path,
                      std::vector<LiomCorridor> &corridors) const;

private:
  bool BoxFree(const LiomBox &box) const;
  bool OrientedBoxFree(const LiomBox &box,double x,double y,double theta) const;
  bool PoseFree(double x,double y,double theta,double margin) const;
  const nav_msgs::OccupancyGrid &grid_;
  const OpenSpace_config &vehicle_;
  double world_x_ = 0, world_y_ = 0, yaw_ = 0;
  double resolution_ = 0;
  bool valid_ = false;
  std::vector<int> occupied_prefix_;
};
}  // namespace open_space
