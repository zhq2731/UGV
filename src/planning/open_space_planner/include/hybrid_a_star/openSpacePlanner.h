
#pragma once
#include "common/inputData.h"
#include "common/basePlanner.h"
#include <deque>
#include <memory>
#include <nav_msgs/OccupancyGrid.h>
#include "hybrid_a_star/hybrid_a_star.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <planning_msgs/OpenSpaceExecutionStatus.h>

class  OpenSpacePlanner : public BasePlanner
{
public:
	OpenSpacePlanner(displayCallback callBack_);
	void setInputData(InputData const & input_data) override;
	bool  implement(double cur_time,const DiscretizedTrajectory prev_trajectory) override;

	void getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_) override;
	/** @brief 清除上一泊车任务的轨迹、反馈和状态机状态，等待新的规划输入。 */
	void resetPlanningState();
private:
	// 控制器一次只接收一个同档位轨迹段；正常到达换挡点后提交缓存的下一段。
	enum class ParkingState {
		WAITING_FOR_INPUT,
		PLANNING,
		TRACKING_COMMITTED_SEGMENT,
		STOP_AT_CUSP,
		EMERGENCY_BRAKING,
		REPLAN_AFTER_SEGMENT_END_MISS,
		GOAL_REACHED,
	};

    nav_msgs::OccupancyGridPtr current_costmap_ptr_;
	std::shared_ptr<VehicleState> current_vehicle_state_ptr_;
	std::shared_ptr<VehicleState> goal_vehicle_state_ptr_;

	OpenSpace_config openspace_config;
	double map_resolution_{0.0};
	planning_msgs::TrajectoryPointArray final_trajectory_;
	planning_msgs::TrajectoryPointArray committed_segment_;
	std::deque<planning_msgs::TrajectoryPointArray> cached_segments_;
	VehicleState committed_goal_state_;
	ParkingState parking_state_{ParkingState::WAITING_FOR_INPUT};
	bool has_committed_goal_{false};
	bool has_committed_segment_{false};

	std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;
	ros::NodeHandle debug_nh_;
	ros::Publisher debug_marker_pub_;
	ros::Publisher goal_reached_pub_;
	ros::Subscriber execution_status_sub_;
	planning_msgs::OpenSpaceExecutionStatus latest_execution_status_;
	bool has_execution_status_{false};

	/** @brief 将自车局部坐标系下的搜索路径转换为 map 坐标系轨迹。 */
	planning_msgs::TrajectoryPointArray GetTraject(const HybridAStarType::VectorVec4d &path, const HybridAStarType::Vec3d &start_state);
	/** @brief 为当前单档位路径生成低速泊车速度、加速度与时间序列。 */
	planning_msgs::TrajectoryPointArray Velocity_Profile_output_os(
		const OpenSpace_config &vel_config,
		planning_msgs::TrajectoryPointArray trajectory);
	/**
	 * @brief 发布 Hybrid A* 扩展边和 Reeds-Shepp 直连段的 RViz 调试标记
	 * @param tree 搜索过程中扩展的父子节点连线
	 * @param rs_path 最终解析直连位姿序列
	 * @param vehicle_pose 局部搜索坐标系在 map 中的起点位姿
	 */
	void publishSearchDebugMarkers(const HybridAStarType::VectorVec4d &tree,
	                               const HybridAStarType::VectorVec3d &rs_path,
	                               const HybridAStarType::Vec3d &vehicle_pose) const;
	/** @brief 判断新目标与当前已提交目标的位置或航向差是否超过阈值。 */
	bool isGoalChanged(const VehicleState &goal) const;
	/** @brief 同时检查目标位置、航向与停稳条件是否全部满足。 */
	bool isGoalReached() const;
	/** @brief 判断车辆是否在已提交段末端误差内停稳，作为换挡点完成条件。 */
	bool isCommittedSegmentFinished() const;
	/**
	 * @brief 用最新 OccupancyGrid 重建 Hybrid A* 碰撞地图
	 * @return 地图尺寸和分辨率有效时返回 true
	 *
	 * 只有数值 25 表示已确认可通行，其余占用格与未知格均按障碍处理。
	 */
	bool initializeCurrentMap();
	/** @brief 从车辆当前进度起，在最新局部栅格上复核已提交段的剩余车身扫掠位姿。 */
	bool isCommittedSegmentCollisionFree() const;
	/**
	 * @brief 从指定轨迹点开始检查离散轨迹及其点间插值是否与最新栅格冲突
	 * @param trajectory 待检查的 map 坐标轨迹
	 * @param start_index 开始检查的轨迹点下标
	 */
	bool isTrajectoryCollisionFree(
		const planning_msgs::TrajectoryPointArray &trajectory,
		size_t start_index,
		double max_check_distance) const;
	/** @brief 从缓存取出下一同档位段，更新时间戳并提交给控制器。 */
	bool commitNextCachedSegment(double cur_time);
	/** @brief 以当前位姿和速度构造原地停车轨迹，用于轨迹失效时覆盖旧轨迹。 */
	planning_msgs::TrajectoryPointArray makeEmergencyStopTrajectory(double cur_time) const;
	/** @brief 清除失效已提交段、切换紧急制动状态并发布停车轨迹。 */
	bool beginEmergencyBraking(double cur_time, const char *reason);
	/** @brief 仅在状态改变时更新泊车状态并输出一条迁移日志。 */
	void setParkingState(ParkingState state);
	/** @brief 返回泊车状态的可读名称，供迁移日志使用。 */
	const char *parkingStateName(ParkingState state) const;
	/**
	 * @brief 接收控制器对当前单段轨迹的执行反馈
	 *
	 * 反馈只在 TRACKING_COMMITTED_SEGMENT 状态中消费；消费后立即清除。
	 * 只有 SEGMENT_END_HOLD 会触发“未满足换挡点但轨迹已耗尽”的
	 * 无档位约束重规划。
	 */
	void executionStatusCallback(
		const planning_msgs::OpenSpaceExecutionStatus::ConstPtr &msg);
};
