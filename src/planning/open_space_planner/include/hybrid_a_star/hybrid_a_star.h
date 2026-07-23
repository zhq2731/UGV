/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
//#define GLOG_USE_GLOG_EXPORT
#ifndef HYBRID_A_STAR_HYBRID_A_STAR_H
#define HYBRID_A_STAR_HYBRID_A_STAR_H

#include "rs_path.h"
#include "state_node.h"

//#include <glog/logging.h>
#include "planning_msgs/TrajectoryPointArray.h"
#include <map>
#include <memory>
//#include "amathutils_lib/geometry.hpp"
#include "amathutils_lib/geometry_point.hpp"

#include "hybrid_a_star/type.h"

using namespace  geometry_point;

struct OpenSpace_config
{
    //车辆信息 车长、车宽、视界系数(0到0.1之间，越大考虑越宽范围的障碍物)
    double car_length;
    double car_width;
    //速度信息 
    double reverse_speed;
    double reverse_brake;
    //安全距离
    double safe_reverse_dis;
    //混合A*参数
    double steering_angle ;
    int steering_angle_discrete_num ;
    double wheel_base;
    double segment_length;
    int segment_length_discrete_num ;
    double steering_penalty ;
    double steering_change_penalty ;
    // 倒车代价
    double reversing_penalty ;
    double shot_distance;

    // 已提交段与换挡点状态机的判定阈值。
    double segment_end_position_tolerance;
    double segment_end_heading_tolerance;
    double stop_speed_tolerance;
    double goal_position_tolerance;
    double goal_heading_tolerance;
    
};
/*
struct Waypoint2D {
	Point2D position;
	double vel;
	double kappa;
	double theta;
    double acc;
    double jerk;

	Waypoint2D() {
	this->position = Point2D(0, 0);
	this->vel = 0;
    this->acc = 0;
    this->jerk = 0;
	}

	Waypoint2D(Point2D position, double vel) {
	this->position = position;
	this->vel = vel;
	}

	Waypoint2D(double x, double y, double vel) {
	this->position.x = x;
	this->position.y = y;
	this->vel = vel;
	}

	Waypoint2D(double x, double y,double theta,double kappa,double vel) {
	  this->position.x = x;
	  this->position.y = y;
	  this->vel = vel;
	  this->theta = theta;
	  this->kappa = kappa;
	}

  Waypoint2D(double x, double y,double theta,double kappa,double vel,double acc,double jerk) {
	  this->position.x = x;
	  this->position.y = y;
	  this->vel = vel;
	  this->theta = theta;
	  this->kappa = kappa;
    this->acc = acc;
    this->jerk = jerk;
	}

};

*/
class HybridAStar {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HybridAStar() = delete;

    HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                int segment_length_discrete_num, double wheel_base, double steering_penalty,
                double reversing_penalty, double steering_change_penalty, double shot_distance,
                int grid_size_phi = 72);

    ~HybridAStar();

    void Init(double x_lower, double x_upper, double y_lower, double y_upper,
              double state_grid_resolution, double map_grid_resolution,double car_length_param,
                       double car_width_param,double wheel_base_param);

    /**
     * @brief 在当前障碍物栅格上执行 Hybrid A* 搜索
     * @param start_state 起点 x、y、航向角
     * @param goal_state 终点 x、y、航向角
     * @param required_start_direction 可选的首段档位约束；NO 表示不约束
     * @return 成功找到无碰路径时返回 true
     *
     * 换挡点重规划时使用 required_start_direction，防止新路径又以上一档位的微小引导段起步。
     */
    bool Search(
        const HybridAStarType::Vec3d &start_state,
        const HybridAStarType::Vec3d &goal_state,
        StateNode::DIRECTION required_start_direction = StateNode::NO);

    HybridAStarType::VectorVec4d GetSearchedTree();

    HybridAStarType::VectorVec3d GetPath() const;

    /**
     * @brief 获取最近一次成功搜索的 Reeds-Shepp 解析连接段
     * @return 搜索树到目标之间的无碰位姿序列，仅供调试可视化使用
     */
    const HybridAStarType::VectorVec3d &GetRsConnectPath() const { return rs_connect_path_; }

    __attribute__((unused)) int GetVisitedNodesNumber() const { return visited_node_number_; }

    __attribute__((unused)) double GetPathLength() const;

    __attribute__((unused)) HybridAStarType::Vec2d CoordinateRounding(const HybridAStarType::Vec2d &pt) const;

    HybridAStarType::Vec2i Coordinate2MapGridIndex(const HybridAStarType::Vec2d &pt) const;

    void SetObstacle(double pt_x, double pt_y);

    void SetObstacle(unsigned int x, unsigned int y);

    /*!
     * Set vehicle shape
     * Consider the shape of the vehicle as a rectangle.
     * @param length vehicle length (a to c)
     * @param width vehicle width (a to d)
     * @param rear_axle_dist Length from rear axle to rear (a to b)
     *
     *         b
     *  a  ---------------- c
     *    |    |          |    Front
     *    |    |          |
     *  d  ----------------
     */
    void SetVehicleShape(double length, double width, double rear_axle_dist);

    /**
     * @brief 使用当前地图与车身外形检查单个车辆位姿是否可行
     * @return 位姿在地图边界内且车身不与障碍格碰撞时返回 true
     *
     * 开放空间状态机用它在新的局部感知栅格到达后复核已提交轨迹。
     */
    bool IsStateCollisionFree(const double &x, const double &y, const double &theta);

    void Reset();

private:
    inline bool HasObstacle(int grid_index_x, int grid_index_y) const;

    inline bool HasObstacle(const HybridAStarType::Vec2i &grid_index) const;

    bool CheckCollision(const double &x, const double &y, const double &theta);

    inline bool LineCheck(double x0, double y0, double x1, double y1);

    /**
     * @brief 尝试用 Reeds-Shepp 曲线将当前节点直接连接到目标
     * @param required_start_direction 若指定，则解析曲线的首个有效运动段必须与之一致
     */
    bool AnalyticExpansions(
        const StateNode::Ptr &current_node_ptr,
        const StateNode::Ptr &goal_node_ptr,
        double &length,
        StateNode::DIRECTION required_start_direction = StateNode::NO);

    inline double ComputeG(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &neighbor_node_ptr) const;

    inline double ComputeH(const StateNode::Ptr &current_node_ptr, const StateNode::Ptr &terminal_node_ptr);

    inline HybridAStarType::Vec3i State2Index(const HybridAStarType::Vec3d &state) const;

    inline HybridAStarType::Vec2d MapGridIndex2Coordinate(const HybridAStarType::Vec2i &grid_index) const;

    void GetNeighborNodes(const StateNode::Ptr &curr_node_ptr, std::vector<StateNode::Ptr> &neighbor_nodes);



    /*!
     * Simplified car model. Center of the rear axle
     * refer to: http://planning.cs.uiuc.edu/node658.html
     * @param step_size Length of discrete steps
     * @param phi Car steering angle
     * @param x Car position (world frame)
     * @param y Car position (world frame)
     * @param theta Car yaw (world frame)
     */
    inline void DynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta) const;

    static inline double Mod2Pi(const double &x);

    bool BeyondBoundary(const HybridAStarType::Vec2d &pt) const;

    void ReleaseMemory();

private:
    uint8_t *map_data_ = nullptr;
    double STATE_GRID_RESOLUTION_{}, MAP_GRID_RESOLUTION_{};
    double ANGULAR_RESOLUTION_{};
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};

    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};

    StateNode::Ptr terminal_node_ptr_ = nullptr;
    StateNode::Ptr ***state_node_map_ = nullptr;

    HybridAStarType::VectorVec3d rs_connect_path_;

    std::multimap<double, StateNode::Ptr> openset_;

    double wheel_base_; //The distance between the front and rear axles
    double segment_length_;
    double move_step_size_;
    double steering_radian_step_size_;
    double steering_radian_; //radian
    double tie_breaker_;

    double shot_distance_;
    int segment_length_discrete_num_;
    int steering_discrete_num_;
    double steering_penalty_;
    double reversing_penalty_;
    double steering_change_penalty_;

    double path_length_ = 0.0;
    double car_length;
    double car_width;

    std::shared_ptr<RSPath> rs_path_ptr_;

    HybridAStarType::VecXd vehicle_shape_;
    HybridAStarType::MatXd vehicle_shape_discrete_;

    // debug
    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;
};

#endif //HYBRID_A_STAR_HYBRID_A_STAR_H
