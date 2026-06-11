#pragma once

#include <ctime>
#include <cmath>
#include <deque>
#include <ros/ros.h>
#include <mutex>
#include <thread>

#include "planning_msgs/TrajectoryPointArray.h"
#include "planning_msgs/ConflictConstraint.h"
#include "conflict_prediction_resolution/conflict_constraint_processor.hpp"
#include "ins_msgs/Ins.h"
#include "localization_msgs/Localization.h"
#include "driver_msgs/ChassisReport.h"
#include "nav_msgs/Odometry.h"
#include "perception_msgs/PredictionObstacles.h"
#include "perception_msgs/TrajectoryPoint.h"
#include "perception_msgs/PerceptionObstacle.h"
#include <visualization_msgs/Marker.h>


#include "amathutils_lib/amathutils.hpp"
#include "amathutils_lib/geometry.hpp"
#include "display/display.h"


#include "math/vec2d.h"
#include "common/planning_config.h"
#include "common/frame.h"
#include "common/pnc_point.h"
#include "common/obstacle.h"
#include "trajectory/reference_point.h"
#include "trajectory/discretized_path.h"
#include "reference_line_info.h"
#include "common/planning_config.h"
#include "decider/path_bounds_decider.h"
#include "decider/piecewise_jerk_path_optimizer.h"
#include "trajectory/trajectory_stitcher.h"

#include "vehicle_info_util/vehicle_info_util.hpp"
#include "velocity_planner/velocity_planner_flow.h"
#include <heartbeat_msgs/Heartbeat.h>
#include "route_msgs/Replan.h"

//beili
#include "plan2control_msgs/Trajectory.h"
#include <lanelet_map_msgs/Way.h>


#include  "utm/UTM.h"
#include  "pid/pid.h"

#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"
#include "platoon_msgs/PlatoonLog.h"
#include "common/common.h"

#include "platoon_common/platoon_common.h"
#include <pwd.h>

#include "completeRefLinePlanner.h"
#include "common/inputData.h"
#include "refLinePlanner.h"
#include "reversePlanner.h"
#include "leavingVelocityPlanner.h"
#include <std_msgs/Int32.h>
#include "route_msgs/Replan.h"
#include "hybrid_a_star/openSpacePlanner.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <route_msgs/MultiPoint.h>



using namespace ugv::planning;
using namespace ugv::common::math;
using namespace ugv::common;

struct VitualInfo
{
    double s;
	double l;
	int   director;
	ReferenceLine refLine;;
};


enum PLANNER_TYPE {
  COMPLETE_REF_LINE = 0,
  REF_LINE,
  REVERSE,
  OPEN_SPACE,
  NONE_PLANNER
};


class PlanningNode
{

public:
	typedef void (PlanningNode::*optFunc)(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	explicit PlanningNode(ros::NodeHandle &nh);
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh;
    ros::Subscriber referenceLineSub;
    ros::Subscriber poseSub;
    ros::Subscriber chassisSub;
    ros::Subscriber obstaclesSub;
    ros::Publisher	trajectoryPub;
    ros::Publisher	trajectoryCandidatePub;
    ros::Publisher	pub_trajectory;
    ros::Publisher	pub_velocity_curve;
    ros::Publisher	pub_actual_velocity_curve;
    ros::Publisher	pub_obs;
    ros::Publisher	pub_to_beili;
    ros::Publisher	pub_heart;
    ros::Publisher	pub_platoon_log;
    ros::Publisher	pub_replan;
    ros::Publisher	pub_Replan;
	ros::Publisher 	pubFreeSpaceMap;
	ros::Publisher  pub_pc;
	
	ros::Subscriber clickPoint_sub_;
	ros::Subscriber initialPose_sub_;
	ros::Subscriber replan_sub_;
	
	ros::Subscriber global_path_wgs84_sub;
	projection::UtmProjector projector;
	ros::Timer timer;
	
	ros::Subscriber multi_point_sub_;

    ros::Subscriber platoonMember_sub_;
    ros::Subscriber platoonMission_sub_;
    ros::Subscriber platoonConfig_sub_;

    ros::Subscriber platoonMember_self_sub_;
    ros::Subscriber platoonMission_self_sub_;
    ros::Subscriber platoonConfig_self_sub_;
	/***新增***/
	ros::Subscriber trajectory_point_sub_;
	ros::Subscriber speedSub;
	ros::Subscriber conflictConstraintSub;
  
	void callBackMultiPointPlanning(const route_msgs::MultiPoint::ConstPtr msg);
	void velocityPlanning(planning_msgs::TrajectoryPointArray &trajectory);
	void pubReplan(const Obstacle *obstacle);
	Obstacle*   memberToObs(int num);
    void planning(PLANNER_TYPE planner_type);
    void displayLoop();
	void baseRefPlanner(planning_msgs::TrajectoryPointArray &finaleTrajectory,double cur_time,bool &useLast,bool &have_negative_obs);
	void reversePlanner(planning_msgs::TrajectoryPointArray &finaleTrajectory);
    void movingVitualObs();
    void collectDiplayObsInfo(const std::vector<const Obstacle *> obs_list)  ;
    void noPlanner(planning_msgs::TrajectoryPointArray &finaleTrajectory);
    void Predict(const double predicted_time_horizon,const VehicleState& cur_vehicle_state,VehicleState& prediced_state);
    void callbackPlanningTimer(const ros::TimerEvent &event);
	void callbackReplan(const route_msgs::Replan::ConstPtr &msg);
	void callbackDesireSpeed(const std_msgs::Float64::ConstPtr &msg);
    void sendHeart(unsigned char flag);
    void caculateAccumulated_s(planning_msgs::TrajectoryPointArray &trajectory);
    void addExtraPath(planning_msgs::TrajectoryPointArray &inPath);
    void publishVelocityCurveMarker(const planning_msgs::TrajectoryPointArray &candidate_trajectory,
                                    const planning_msgs::TrajectoryPointArray &final_trajectory);
    void recordActualVelocity(const ros::Time &stamp, double velocity);
    void publishActualVelocityCurveMarker();
    void callbackGlobalPath84InPlanning(const lanelet_map_msgs::Way::ConstPtr &msg);
  	static void collectDisplayInfo(const   planning_msgs::TrajectoryPointArray *planned_trajectory,\
    const TrajectoryPoint *planning_start_point = nullptr ,const PathBoundary *lane_boundry = nullptr,const PathBoundary *planning_boundry = nullptr,const ReferenceLine *reference_line = nullptr) ;  
private:
	std::mutex mtx;
	bool  newReplan = true;
	driver_msgs::ChassisReport current_chassis;
	double current_velocity;
	std::vector< const Obstacle*> obsList;
	std::vector< VitualInfo> virtualInfo;
	nav_msgs::Odometry current_pose_;
	double yaw;
	bool newTrajectory = false;
	bool pose_inited_ = false;
	bool is_obs_update = false;
	bool blockedRoad = false;
	vehicle_info_util::VehicleInfoUtil *vehicle_util;
    double heading_compensation_degree;
     
	static bool newBoundry;
	static PathBoundary lane_boundry_;
	static PathBoundary planning_boundry_;
	static ReferenceLine reference_line_;
	static planning_msgs::TrajectoryPointArray planned_trajectory_;
	static TrajectoryPoint planning_start_point_;

	std::vector<Obstacle> obs_vec;
	perception_msgs::PredictionObstacles obs;
	VehicleState cur_vehicle_state;
    DiscretizedTrajectory last_trajectory;	
	std::shared_ptr<VelocityPlannerFlow> velocityPlanner;
	std::thread display_thread_;

	unsigned char    navUncertainty;
	bool specialSituation = false;
	conflict_prediction_resolution::ConflictConstraintProcessor conflict_constraint_processor_;
	bool enable_velocity_curve_marker_ = true;
	double velocity_curve_time_horizon_ = 8.0;
	double velocity_curve_time_scale_ = 1.0;
	double velocity_curve_speed_scale_ = 1.5;
	double velocity_curve_front_offset_ = 4.0;
	double velocity_curve_left_offset_ = 6.0;
	bool enable_actual_velocity_curve_marker_ = true;
	double actual_velocity_curve_history_duration_ = 10.0;
	double actual_velocity_curve_time_scale_ = 1.0;
	double actual_velocity_curve_speed_scale_ = 1.5;
	double actual_velocity_curve_front_offset_ = 4.0;
	double actual_velocity_curve_side_offset_ = -6.0;
	std::deque<std::pair<double, double>> actual_velocity_history_;
    void callbackReferenceLine(const planning_msgs::TrajectoryPointArray::Ptr);
    void callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg);
    void callbackPose(const localization_msgs::Localization::ConstPtr &msg);	
	void callbackObstacles(const perception_msgs::PredictionObstacles::ConstPtr &msg);
	void callbackConflictConstraint(const planning_msgs::ConflictConstraint::ConstPtr &msg);
	void callBackinitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
	void callBackGoal(const geometry_msgs::PoseStamped::ConstPtr msg);
	void loadPlanningParam(ros::NodeHandle &private_nh_);
	PLANNER_TYPE plannerTypeDecision();
	void generateBound(ReferenceLine &refLine,PathBoundary &boundry,
    std::vector<geometry_msgs::Point> &left,std::vector<geometry_msgs::Point> &right);
    std::map<PLANNER_TYPE,std::shared_ptr<BasePlanner>> planners;
	PLANNER_TYPE last_planner_type ;
	InputData  inputData;

	unsigned char shape;
	std::vector<optFunc> optsFunc;
	PlatoonType platoonType;
	std::map<int,platoon_msgs::PlatoonMember> platoonMembers;
	std::vector<unsigned char> vehicle_num_list;
	VehicleState selfState;
	platoon_msgs::PlatoonConfig platoon_config;
	common::PlatformParam platformParam;
	int   leavingNum = 0;
	bool  platoonBuild = false;
    bool  joinSelf = false;
    bool  leaveSelf = false;
    bool  leavePubFlag = false;
	unsigned char  trajectoryType = PlatoonType::NONE;
	PIDController pid_controller;
	void  none(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  build(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  join(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  leave(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  disolve(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  column(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  diamond(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  triangle(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  cancel_reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg);	
	void  mass(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  distrubute(const platoon_msgs::PlatoonMission::ConstPtr &msg);

	void  running(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	
	void  vehicleStateInfo(VehicleState &state,int num) ;
	bool  isOrdered(int from,int to);
	void  callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg) ;
	void  callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void  callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg) ;
	void  platoonVelocityPlanner(planning_msgs::TrajectoryPointArray &trajectory);
	bool  platoonMassPoint();
	bool  openSpaceGoalSet = false;
	void obsToGrid(const std::vector< const Obstacle*> &obsList);
	void computeBoundingBox(const vector<pcl::PointXYZI> obs_pcl,double& min_x, double& max_x, double& min_y, double& max_y);
	void computeFreeSpacePoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudIn, float* free_space, int free_space_n = 360);
	void freeGridMapFilter(float* freeSpacePoints, Eigen::MatrixXi &dst);
	void publishFreeSpaceGridMap(Eigen::MatrixXi &freeSpaceGridMap, nav_msgs::OccupancyGrid& rosMap);
};

bool PlanningNode::newBoundry = false;
PathBoundary PlanningNode::lane_boundry_;
PathBoundary PlanningNode::planning_boundry_;
ReferenceLine PlanningNode::reference_line_;
planning_msgs::TrajectoryPointArray PlanningNode::planned_trajectory_;
TrajectoryPoint PlanningNode::planning_start_point_;
