#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <fstream>

#include <sstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#include "planning_msgs/TrajectoryPointArray.h"
#include "ins_msgs/Ins.h"
#include "localization_msgs/Localization.h"
#include "amathutils_lib/amathutils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include <ros/package.h>

#include "driver_msgs/DriveCmd.h"
#include "driver_msgs/GearCmd.h"
#include "driver_msgs/ChassisReport.h"
#include "driver_msgs/SteeringWheelCmd.h"
#include <chrono>
#include "sensor_driver_msgs/GpswithHeading.h"
#include "utm/UTM.h"
#include "ray_msgs/Report.h"

#include "platoon_common/platoon_common.h"

#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"
#include "driver_msgs/MotionStartCmd.h"
#include "std_msgs/Empty.h"
#include "common/common.h"
#include "route_msgs/MultiPoint.h"
#include "route_msgs/InitPoint.h"

struct SimulateParam
{
	bool is_forward;
	double v;
	bool reverse_path;
	bool open_simulate_platoon;
};

struct TrajectoryPointData
{
	double x;
	double y;
	double z;
	double v;
	double theta;
	double relative_time;
};

/** @brief 仿真泊车场中用于生成激光遮挡的静态矩形障碍物。 */
struct ParkingObstacle
{
	double x;
	double y;
	double yaw;
};

class Simulate
{
public:
	explicit Simulate(ros::NodeHandle &nh);

	virtual ~Simulate() {}
	ros::Subscriber loncmd_sub_;
	ros::Subscriber latcmd_sub_;
	ros::Subscriber gearcmd_sub_;
	ros::Subscriber clickPoint_sub_;
	ros::Subscriber initialPose_sub_;
	ros::Subscriber goal_sub_;

	ros::Publisher bit_report_pub_;
	ros::Publisher gpsPub_;
	ros::Publisher pub_chassis_;
	ros::Publisher pub_odometry_;
	ros::Publisher pub_steering_;
	ros::Publisher pub_velcoty_;
	ros::Publisher free_space_map_pub_;
	ros::Publisher parking_visualization_pub_;

	ros::Subscriber platoonMember_sub_;
	ros::Subscriber platoonMission_sub_;
	ros::Subscriber platoonConfig_sub_;

	ros::Subscriber platoonMember_self_sub_;
	ros::Subscriber platoonMission_self_sub_;
	ros::Subscriber motion_start_sub_;
	ros::Subscriber open_space_task_reset_sub_;
	ros::Subscriber platoonConfig_self_sub_;
	ros::Subscriber init_point_sub_;
	ros::Subscriber trajectory_sub_;
	ros::Subscriber multi_point_sub_;

	void publishInitPose();
	bool readGpsTxts(std::string &fileName, std::vector<geometry_msgs::Point> &points);
	bool readCordTxts(std::string &fileName, std::vector<geometry_msgs::Point> &points);

	void onLonControlCommand(const driver_msgs::DriveCmd::ConstPtr msg);
	void onLatControlCommand(const driver_msgs::SteeringWheelCmd::ConstPtr msg);
	/** @brief 接收泊车档位请求，并启动带延迟的虚拟换挡过程。 */
	void onGearControlCommand(const driver_msgs::GearCmd::ConstPtr msg);
	void publishChassis();
	void publishGpsdata();
	double normalizeRadian(const double _angle);
	void callbackTimer(const ros::TimerEvent &event);
	void callbackTimer2(const ros::TimerEvent &event);
	/**
	 * @brief 周期生成以自车为原点的激光射线式局部栅格并发布
	 *
	 * 每条射线上障碍物之前的已观测格置为25，命中格置为100，遮挡区保持未知，
	 * 用于验证“只有已确认可达区域才可规划”的开放空间逻辑。
	 */
	void parkingGridTimer(const ros::TimerEvent &event);
	/** @brief 发布仿真障碍物、车身与前轮实际转角的 RViz Marker。 */
	void publishParkingVisualization(const ros::Time &stamp);

	void initPoseCallBack(
		const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);

	void trajectoryCallBack(const planning_msgs::TrajectoryPointArray::ConstPtr &msg);
	double interpolateTheta(double theta1, double theta2, double alpha);
	void publishLocalizationMsg(double x, double y, double z, double theta);

	void callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg);
	void callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg);
	void callBackinitialPose(const geometry_msgs::PoseStamped::ConstPtr msg);
	void callBackgoal(const geometry_msgs::PointStamped::ConstPtr msg);
	void motionStartCallback(const driver_msgs::MotionStartCmd::ConstPtr &msg);
	/** @brief 新泊车任务到来时停止车辆并清除上一轮运动命令状态。 */
	void openSpaceTaskResetCallback(const std_msgs::Empty::ConstPtr &msg);
	void callBackMultiPointPlanning(const route_msgs::MultiPoint::ConstPtr msg);
	void callBackInitPoint(const route_msgs::InitPoint::ConstPtr msg);
	void callbackCloudmap(const std_msgs::UInt8::ConstPtr &msg);


	common::PlatformParam platformParam;

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	bool lon_timer_inited;
	bool lat_timer_inited;
	// 以下执行器模型仅在开放空间泊车仿真中启用，不改变原道路仿真行为。
	bool open_space_execution_mode_{false};
	// 实际前轮角每秒允许变化的最大弧度，用于模拟转向执行器动态。
	double open_space_front_tire_steering_rate_limit_radps_{0.25};
	// 开启后，档位命令不会立即生效，而是在停稳并等待配置延迟后更新底盘反馈。
	bool open_space_virtual_gear_shift_{false};
	double open_space_virtual_gear_shift_delay_{0.3};

	ros::Timer timer;
	ros::Timer timer2;
	ros::Timer parking_grid_timer_;
	tf::TransformBroadcaster tf_broadcaster_;
	projection::UtmProjector projector_;

private:
	geometry_msgs::Point pos;
	double theta;
	double v;
	double acc;
	vehicle_info_util::VehicleInfoUtil *vehcileInfo;
	bool inited;
	double steering{0.0};
	int plan_point_count = 0;

	std::chrono::system_clock::time_point lon_last_time;
	std::chrono::system_clock::time_point lat_last_time;
	SimulateParam param;
	std::vector<unsigned char> vehicle_num_list;
	bool built = false;
	unsigned char motion_start = 0;
	// 虚拟档位执行器状态：实际反馈档位、待切换档位、执行标志和开始时间。
	uint8_t gear_location_{0};
	uint8_t pending_gear_location_{0};
	bool gear_shift_pending_{false};
	ros::Time gear_shift_start_time_;
	std::map<double, TrajectoryPointData> trajectory_map;
	ros::Time header_time;
	bool last_is_forward_shift;
	int map_length_;
	int map_width_;
	int map_point_num_;
	double map_resolution_;
	double parking_obstacle_length_;
	double parking_obstacle_width_;
	std::vector<ParkingObstacle> parking_obstacles_;

	ros::Subscriber cloud_map_sub_;
	geometry_msgs::Point map_origin_;

	/** @brief 清除虚拟换挡过程并恢复空挡。 */
	void resetVirtualGearState();
};
