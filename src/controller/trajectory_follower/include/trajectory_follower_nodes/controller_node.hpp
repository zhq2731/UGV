// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <ros/ros.h>
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower/lateral_controller_base.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "autoware_msgs/TrajectoryPointArray.h"
#include "autoware_msgs/TrajectoryPoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include  "std_msgs/Float32.h"
#include  "std_msgs/Float64.h"
#include  "std_msgs/Int32.h"
#include  "std_msgs/Empty.h"

#include "planning_msgs/TrajectoryPointArray.h"
#include "planning_msgs/OpenSpaceExecutionStatus.h"
#include "ins_msgs/Ins.h"
#include  "localization_msgs/Localization.h"

#include "driver_msgs/DriveCmd.h"
#include "driver_msgs/ChassisReport.h"
#include "driver_msgs/SteeringWheelCmd.h"
#include "driver_msgs/GearCmd.h"
#include "driver_msgs/MotionStartCmd.h"

#include "vehicle_info_util/vehicle_info_util.hpp"
#include "amathutils_lib/amathutils.hpp"
#include "amathutils_lib/geometry.hpp"

#include "control/controller/lon_controller.h"
#include "display/display.h"
#include "visualization_msgs/MarkerArray.h"
#include <heartbeat_msgs/Heartbeat.h>
#include "platoon_msgs/PlatoonLog.h"

#include <pwd.h>
#include <fstream> 
#include "common/common.h"
#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"
#include "platoon_msgs/PlatoonLog.h"
#include "platoon_common/platoon_common.h"

namespace autoware
{
namespace motion
{
namespace control
{
using trajectory_follower::LateralOutput;
namespace trajectory_follower_nodes
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// \classController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)

struct PlatformParam
{
	std::string vehicle_type;// "zhitong"  # 车辆类型  geometry_c  sanzhou tank500 zhito zw zhitong
	std::string ins_type;// "cgi610" # cgi610 cgi1010 ht33  570d shengda
	std::string id; // "vehicle_1"  # vehicle_1 vehicle_2...
	int num; // # 1 2 3 4...
};


struct ControlLog
{ 
	geometry_msgs::Pose pose;
    double yaw;
	geometry_msgs::Pose nearestPose;
	double current_v;
	double deired_v;
	double v_error;
	double lat_error;
	double yaw_error;
	double driving_s;
	double throttle;
	double brake_pedal;
	double deired_acc;
	geometry_msgs::Pose lastLogPose;
};


class TRAJECTORY_FOLLOWER_PUBLIC Controller
{
public:
  explicit Controller(ros::NodeHandle &nh);
  virtual ~Controller() {logfile.close();}
  void run();
  void sendHeart(unsigned char flag);

private:
	nav_msgs::Odometry current_pose_;
	::common::PlatformParam platform_param;
	double steer_compensation;

	planning_msgs::TrajectoryPointArray sim_trajectory;
	autoware_msgs::TrajectoryPointArray current_waypoints_;
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh;
	ros::Timer timer;
    int driving_mode;
	std::ofstream   logfile;
	ros::Publisher  pub2_;
	ros::Subscriber sub_pose_; 
	ros::Subscriber sub_platoon_log; 

	ros::Subscriber chassis_sub_, motion_start_sub_, open_space_task_reset_sub_;
	
	ros::Subscriber steer_compensation_sub;
	ros::Publisher pub_diplay;
	ros::Publisher pub_heart;
	
	ros::Subscriber steering_sub_;

	trajectory_follower::InputData input_data_;
	double timeout_thr_sec_;
	double steer_compensation_degree;
	boost::optional<LateralOutput> lateral_output_{boost::none};

	std::shared_ptr<trajectory_follower::LateralControllerBase> lateral_controller_;

	/**************longititude control**************/
	std::shared_ptr<car::control::LonController> longitudinal_controller_;
	std_msgs::Header mHeader;
	ros::Publisher gearcmd_pub_; /**档位发布者*/
	ros::Publisher parking_brake_pub_; /**档位发布者*/

    
    ros::Subscriber platoonMission_sub_;
	ros::Subscriber platoonConfig_self_sub_;
	ros::Subscriber sub_ref_path_;
	ros::Subscriber sub_chassis_;
	ros::Subscriber sub_odometry_;
	ros::Subscriber sub_steering_;
	ros::Subscriber sub_accel_;
	ros::Publisher control_cmd_pub_;

	ros::Publisher latcmd_pub_;
	ros::Publisher loncmd_pub_;
	ros::Publisher latcontrol_debug;
	ros::Publisher open_space_mpc_yaw_error_pub_;
	ros::Publisher open_space_mpc_desired_tire_angle_pub_;
	ros::Publisher open_space_mpc_actual_tire_angle_pub_;
	ros::Publisher open_space_mpc_signed_speed_pub_;
	ros::Publisher open_space_mpc_reference_pose_pub_;
	ros::Publisher open_space_execution_status_pub_;
	bool is_forward_shift;
	int  driving_mode_feedback = 0;//0：人工 1：自动
	bool debug;
	bool open_lat_controller;
	bool open_lon_controller;
	bool open_simulate;
	// 开放空间泊车一次只执行一个已提交的低速轨迹段，
	// 与原有参考线的时间索引仿真逻辑保持隔离。
	bool open_space_execution_mode;
	double open_space_lookahead_distance;
	double open_space_speed_kp;
	double open_space_max_acceleration;
	double open_space_max_deceleration;
	double open_space_steering_prepare_tolerance;
	double open_space_stop_speed_tolerance;
	double open_space_hold_deceleration;
	double open_space_hold_brake_pedal;
	double open_space_segment_end_remaining_distance;
	double open_space_segment_end_stable_duration;
	int open_space_steering_prepare_stable_cycles;
	enum class OpenSpaceExecutionState {
		IDLE,
		HOLD_STOP,
		PREPARE_STEERING,
		EXECUTING,
		SEGMENT_END_HOLD,
	};
	OpenSpaceExecutionState open_space_execution_state_{OpenSpaceExecutionState::IDLE};
	planning_msgs::TrajectoryPointArray pending_open_space_trajectory_;
	bool has_pending_open_space_trajectory_{false};
	int open_space_steering_ready_cycles_{0};
	double open_space_steering_prepare_target_{0.0};
	size_t open_space_progress_index_{0};
	ros::Time open_space_segment_end_candidate_start_;
	double heading_compensation_degree;
	bool lat_use_current_velocity_only;
	bool enable_log;
	bool is_pose_set_;
	driver_msgs::SteeringWheelCmd lastLatCmd;
	driver_msgs::DriveCmd lastLonCmd;
	vehicle_info_util::VehicleInfoUtil  *vehcileInfo;
    car::control::InputData lon_input;
	platoon_msgs::PlatoonLog platoonLog;
	unsigned char  trajectoryType = PlatoonType::NONE;
	unsigned char  inputTrajectoryType = PlatoonType::NONE;
	ControlLog controlLog;
	bool platoonBuild = false;
	enum class LateralControllerMode {
	INVALID = 0,
	MPC = 1,
	PURE_PURSUIT = 2,
	};
	enum class LongitudinalControllerMode {
	INVALID = 0,
	PID = 1,
	};

	/**
	* @brief compute control command, and publish periodically
	*/
	void onTrajectory(const planning_msgs::TrajectoryPointArray::Ptr);
	//void onChassis(const driver_msgs::msg::Chassis::SharedPtr msg);
	//void onOdometry(const gps_info_msgs::msg::GpsInfoData::SharedPtr msg);
	//void onSteering(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
	//void onAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
	bool isTimeOut();
	LateralControllerMode getLateralControllerMode(const std::string & algorithm_name) const;
	LongitudinalControllerMode getLongitudinalControllerMode(
	const std::string & algorithm_name) const;

	void chassisCallback(const driver_msgs::ChassisReport::ConstPtr &msg);
	void platoonLogCallback(const platoon_msgs::PlatoonLog::ConstPtr &msg);

    
	void steerCompensationCallback(const std_msgs::Float32::ConstPtr &msg);
	void motionStartCallback(const driver_msgs::MotionStartCmd::ConstPtr &msg);
	/** @brief 新起点或新终点到来时，清除上一泊车轨迹和控制状态。 */
	void openSpaceTaskResetCallback(const std_msgs::Empty::ConstPtr &msg);
	void callbackPose(const localization_msgs::Localization::ConstPtr &msg);
	double distance2D(geometry_msgs::Point &p1 ,geometry_msgs::Point &p2) ;
	double	closestPointVel() const ;
	double normalized_angle(double angle) ;
	void lonControl();
	/**
	 * @brief 在泊车轨迹待接管或前轮准备阶段持续发布制动保持命令
	 *
	 * 该命令覆盖上一段纵向输出：车辆未停稳时按速度反向减速，停稳后继续保持制动踏板。
	 */
	void publishOpenSpaceHoldCommand();
	void latControl();
	void diplayDesireVelocity();
	size_t QueryLowerBoundPoint(const double relative_time,
                                                   planning_msgs::TrajectoryPointArray &trajectory,const double epsilon = 1.0e-5) const ;
	void computeLonSim(driver_msgs::DriveCmd &lonCmd);
	/**
	 * @brief 根据已激活的单段泊车轨迹计算低速纵向命令
	 * @param lonCmd 输出的目标速度与受限加速度命令
	 *
	 * 函数用轨迹投影更新进度，按弧长选取预瞄速度，并在段末应用可停车速度包络。
	 */
	void computeOpenSpaceLonSim(driver_msgs::DriveCmd &lonCmd);
	/** @brief 将轨迹写入横向与纵向跟踪缓冲，作为当前激活轨迹。 */
	void activateTrajectory(const planning_msgs::TrajectoryPointArray &trajectory);
	/**
	 * @brief 接收新泊车段并转入“停车—转角准备—执行”交接流程
	 * @param trajectory 规划器下发的单一档位轨迹段
	 */
	void stageOpenSpaceTrajectory(const planning_msgs::TrajectoryPointArray &trajectory);
	/** @brief 判断轨迹是否为紧急制动或保持停车段。 */
	bool isOpenSpaceStopTrajectory(const planning_msgs::TrajectoryPointArray &trajectory) const;
	/**
	 * @brief 用轨迹起点曲率和行驶方向计算起步前需准备的前轮角
	 * @return 按车辆转角上限限幅后的目标前轮角，单位 rad
	 */
	double calculateOpenSpaceSteeringPrepareTarget(
		const planning_msgs::TrajectoryPointArray &trajectory) const;
	/** @brief 仅在状态发生变化时更新执行状态并记录迁移日志。 */
	void setOpenSpaceExecutionState(OpenSpaceExecutionState state);
	/** @brief 返回执行状态的可读名称，供状态迁移日志使用。 */
	const char *openSpaceExecutionStateName(OpenSpaceExecutionState state) const;
	/**
	 * @brief 检查轨迹末端停车是否已连续稳定足够时间，并向规划器上报段结束保持状态
	 * @param remaining_distance 当前车辆投影点到真实轨迹末端的剩余弧长
	 *
	 * 只有末端剩余弧长、目标速度、实际速度和稳定时间全部满足条件时才触发；
	 * 状态触发后控制器进入制动保持，等待规划器下发新的单段轨迹。
	 */
	void updateOpenSpaceSegmentEndHold(double remaining_distance);
	void callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg) ;

};
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER_NODES__CONTROLLER_NODE_HPP_
