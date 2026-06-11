#include "planning_node.h"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <std_msgs/ColorRGBA.h>

using namespace ugv::planning;
using namespace ugv::common::math;
using namespace ugv::common;

namespace {

visualization_msgs::Marker makeDeleteAllMarker()
{
	visualization_msgs::Marker marker;
	marker.action = visualization_msgs::Marker::DELETEALL;
	return marker;
}

geometry_msgs::Point makeVelocityCurvePoint(const geometry_msgs::Point &origin,
                                            const double yaw,
                                            const double local_x,
                                            const double local_y,
                                            const double z)
{
	geometry_msgs::Point local_point;
	local_point.x = local_x;
	local_point.y = local_y;

	geometry_msgs::Point global_point = amathutils::localToGlobal(origin, yaw, local_point);
	global_point.z = z;
	return global_point;
}

visualization_msgs::Marker makeVelocityCurveLine(const planning_msgs::TrajectoryPointArray &trajectory,
                                                 const geometry_msgs::Point &origin,
                                                 const double yaw,
                                                 const double front_offset,
                                                 const double left_offset,
                                                 const double time_horizon,
                                                 const double time_scale,
                                                 const double speed_scale,
                                                 const std::string &ns,
                                                 const int id,
                                                 const std_msgs::ColorRGBA &color)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.08;
	marker.color = color;

	for (const auto &point : trajectory.points) {
		const double relative_time = std::max(0.0, point.relative_time);
		if (relative_time > time_horizon) {
			break;
		}

		// 横轴使用 relative_time，纵轴使用速度 v；放在车辆左前方，避免和真实轨迹混在一起。
		marker.points.push_back(makeVelocityCurvePoint(
			origin, yaw, front_offset + relative_time * time_scale,
			left_offset + std::max(0.0, point.v) * speed_scale, 1.2));
	}

	return marker;
}

visualization_msgs::Marker makeVelocityCurveText(const geometry_msgs::Point &position,
                                                 const std::string &text,
                                                 const std::string &ns,
                                                 const int id,
                                                 const std_msgs::ColorRGBA &color,
                                                 const double scale)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position = position;
	marker.pose.orientation.w = 1.0;
	marker.scale.z = scale;
	marker.color = color;
	marker.text = text;
	return marker;
}

std_msgs::ColorRGBA makeColor(const float r, const float g, const float b, const float a)
{
	std_msgs::ColorRGBA color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	return color;
}

}



PlanningNode::PlanningNode(ros::NodeHandle &nh): nh_(nh),private_nh("~")
{
	 vehicle_util = vehicle_info_util::VehicleInfoUtil::get_instance();
	 vehicle_util->loadVehicleingParam(private_nh);
	 loadPlanningParam(private_nh); 
	 private_nh.param<double>("heading_compensation_degree",  heading_compensation_degree, 0.0);
	 // 速度曲线只用于 RViz 调试：横轴为轨迹点 relative_time，纵轴为轨迹点速度 v。
	 private_nh.param<bool>("enable_velocity_curve_marker", enable_velocity_curve_marker_, true);
	 private_nh.param<double>("velocity_curve_time_horizon", velocity_curve_time_horizon_, 8.0);
	 private_nh.param<double>("velocity_curve_time_scale", velocity_curve_time_scale_, 1.0);
	 private_nh.param<double>("velocity_curve_speed_scale", velocity_curve_speed_scale_, 1.5);
	 private_nh.param<double>("velocity_curve_front_offset", velocity_curve_front_offset_, 4.0);
	 private_nh.param<double>("velocity_curve_left_offset", velocity_curve_left_offset_, 6.0);
	 // 实际速度曲线用于观察底盘反馈速度的历史变化，和规划速度曲线分开显示。
	 private_nh.param<bool>("enable_actual_velocity_curve_marker", enable_actual_velocity_curve_marker_, true);
	 private_nh.param<double>("actual_velocity_curve_history_duration", actual_velocity_curve_history_duration_, 10.0);
	 private_nh.param<double>("actual_velocity_curve_time_scale", actual_velocity_curve_time_scale_, 1.0);
	 private_nh.param<double>("actual_velocity_curve_speed_scale", actual_velocity_curve_speed_scale_, 1.5);
	 private_nh.param<double>("actual_velocity_curve_front_offset", actual_velocity_curve_front_offset_, 4.0);
	 private_nh.param<double>("actual_velocity_curve_side_offset", actual_velocity_curve_side_offset_, -6.0);

	 std::string vehicle_platform_file;
	 private_nh.param<std::string>("vehicle_platform_file", vehicle_platform_file, "vehicle_platform.yaml");

	 bool open_simulate;
	 private_nh.param<bool>("open_simulate",  open_simulate, false);

	 common::getPlatformParam(vehicle_platform_file,platformParam);
	 velocityPlanner = std::make_shared<VelocityPlannerFlow>(private_nh);

	 referenceLineSub = nh_.subscribe("referenceLine",1, &PlanningNode::callbackReferenceLine, this);

	 poseSub = nh_.subscribe("odomData", 1, &PlanningNode::callbackPose, this);

	 chassisSub = nh_.subscribe("chassis", 1, &PlanningNode::callbackChassis, this); 

	 trajectoryPub  = nh_.advertise<planning_msgs::TrajectoryPointArray>("trajectory", 1);

	 // 冲突消解决策的具体解释和速度修正由独立处理器负责，PlanningNode 只保留流程连接。
	 conflict_constraint_processor_.loadParam(private_nh);
	 if (conflict_constraint_processor_.enabled()){
	     // 只有开启冲突消解时才发布候选轨迹和订阅冲突约束；关闭时保持原规划链路运行。
	     trajectoryCandidatePub  = nh_.advertise<planning_msgs::TrajectoryPointArray>("trajectory_candidate", 1);
	     conflictConstraintSub = nh_.subscribe("conflict_constraint", 1, &PlanningNode::callbackConflictConstraint, this);
	 }

	 //replan_sub_ = nh_.subscribe("/replan", 1, &PlanningNode::callbackReplan, this); 

	 obstaclesSub = nh_.subscribe("/MultiObjectTracker", 1, &PlanningNode::callbackObstacles, this); 

	 pub_trajectory = nh_.advertise<visualization_msgs::MarkerArray>(
		"planning_marker", 1);

	 pub_velocity_curve = nh_.advertise<visualization_msgs::MarkerArray>(
		"velocity_curve_marker", 1);

	 pub_actual_velocity_curve = nh_.advertise<visualization_msgs::MarkerArray>(
		"actual_velocity_curve_marker", 1);

	 pub_obs = nh_.advertise<visualization_msgs::MarkerArray>(
		"perception_obs", 1);

	 pubFreeSpaceMap = nh_.advertise<nav_msgs::OccupancyGrid>(
		   "free_space_map", 1);
	 
	 speedSub = nh_.subscribe("desireSpeed", 1, &PlanningNode::callbackDesireSpeed, this); 

	 if (open_simulate){
	     clickPoint_sub_  = nh_.subscribe("/move_base_simple/goal", 1, &PlanningNode::callBackGoal, this);
	     initialPose_sub_ = nh_.subscribe("/initialpose", 1, &PlanningNode::callBackinitialPose, this);
	 }
	 display_thread_ = std::thread (&PlanningNode::displayLoop,this);
	 display_thread_.detach();

	 pub_to_beili = nh_.advertise<plan2control_msgs::Trajectory>(
	   "global_path/traj_plan", 1);

	 timer = private_nh.createTimer(ros::Duration(0.1),&PlanningNode::callbackPlanningTimer,this);


	 multi_point_sub_ = nh_.subscribe("/multi_point_planning", 1, &PlanningNode::callBackMultiPointPlanning, this);

	 
	 pub_heart = nh_.advertise<heartbeat_msgs::Heartbeat>("heartbeat", 1);
	 
	 pub_platoon_log = nh_.advertise<platoon_msgs::PlatoonLog>("platoon_log", 1);
	 
	 
	 pub_replan = nh_.advertise<route_msgs::Replan>("/replan", 1);
     pub_pc = nh_.advertise<sensor_msgs::PointCloud2>("/known_points_cloud", 1);
	 
	 global_path_wgs84_sub = nh_.subscribe("topology_global_path_wgs84", 10, &PlanningNode::callbackGlobalPath84InPlanning, this); 


	 planners[COMPLETE_REF_LINE] = std::make_shared<CompleteRefLinePlanner>(this->collectDisplayInfo);
	 planners[REF_LINE] = std::make_shared<RefLinePlanner>(this->collectDisplayInfo);
	 planners[REVERSE] = std::make_shared<ReversePlanner>(this->collectDisplayInfo);
	// planners[OPEN_SPACE] = std::make_shared<OpenSpacePlanner>(this->collectDisplayInfo);
     shape = PlatoonType::COLUMN;

	 platoonMember_sub_ = nh.subscribe("/PlatoonMember", 10, &PlanningNode::callbackPlatoonMember, this); 
	 platoonMission_sub_ = nh.subscribe("/PlatoonMission", 10, &PlanningNode::callbackPlatoonMission, this); 
	 platoonConfig_sub_ = nh.subscribe("/PlatoonConfig", 10, &PlanningNode::callbackPlatoonConfig, this); 
	 
	 
	 platoonMember_self_sub_= nh.subscribe("PlatoonMember_self", 10, &PlanningNode::callbackPlatoonMember, this);
	 platoonConfig_self_sub_= nh.subscribe("PlatoonMission_self", 10, &PlanningNode::callbackPlatoonMission, this);
	 platoonMission_self_sub_= nh.subscribe("PlatoonConfig_self", 10, &PlanningNode::callbackPlatoonConfig, this);
	 
	 optsFunc.push_back(&PlanningNode::none);
	 optsFunc.push_back(&PlanningNode::build);
	 optsFunc.push_back(&PlanningNode::join);
	 optsFunc.push_back(&PlanningNode::leave);
	 optsFunc.push_back(&PlanningNode::disolve);
	 optsFunc.push_back(&PlanningNode::column);
	 optsFunc.push_back(&PlanningNode::diamond);
	 optsFunc.push_back(&PlanningNode::triangle);
	 optsFunc.push_back(&PlanningNode::reverse);
	 optsFunc.push_back(&PlanningNode::cancel_reverse);	 
	 optsFunc.push_back(&PlanningNode::mass);
	 optsFunc.push_back(&PlanningNode::distrubute);
	 optsFunc.push_back(&PlanningNode::running);
	 

}



void  PlanningNode::none(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    return;
}


void  PlanningNode::build(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    vehicle_num_list = msg->vehicle_list;
	platoon_config   = msg->config;
    platoonBuild     =  true;

	std::cout <<"printConfig --start--"<<std::endl;
	std::cout <<"policy: "<<platoon_config.policy<<std::endl;
	std::cout <<"time: "<<platoon_config.time<<std::endl;
	std::cout <<"distance: "<<platoon_config.distance<<std::endl;
	std::cout <<"lateral_offset: "<<platoon_config.lateral_offset<<std::endl;
	std::cout <<"use_path_planning: "<<(int)platoon_config.use_path_planning<<std::endl;
	
	std::cout <<"printConfig --end--"<<std::endl;
	return;
}


void  PlanningNode::join(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    std::cout <<"PlanningNode join num :   "<< (int)msg->num<<std::endl;
    std::cout <<"PlanningNode platform num :   "<< (int)platformParam.num<<std::endl;
	
    if(!platoonCheckNum(vehicle_num_list,msg->num)){
	    vehicle_num_list.push_back(msg->num);
		if (msg->num == platformParam.num){
			leaveSelf = false;
			joinSelf = true;
			leavePubFlag = false;
			return ;
		}
    }
}


void  PlanningNode::leave(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    std::cout <<"PlanningNode leave  num :   "<< (int)msg->num<<std::endl;
    std::cout <<"PlanningNode platform num :   "<< (int)platformParam.num<<std::endl;
    int indexLeave = platoonGetNumIndex(vehicle_num_list,msg->num);
    int indexSelf  = platoonGetNumIndex(vehicle_num_list,platformParam.num);
    assert(indexLeave > 0);
	platoonEraseNum(vehicle_num_list,msg->num);
	
	if (indexLeave > indexSelf)
		return;
	
	if (msg->num  ==  platformParam.num && (!leaveSelf)){
		leaveSelf = true;		
    	joinSelf = false;
		leavePubFlag = false;
		return ;
	}

	leavingNum = msg->num;  
	
	return ;
}


void  PlanningNode::disolve(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::vector<unsigned char>().swap(vehicle_num_list);
	leavingNum = 0;
	joinSelf = false;
	leaveSelf = false;
    platoonBuild = false;
	leavePubFlag = false;
    return ;
}


void  PlanningNode::column(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    return ;

}

void  PlanningNode::diamond(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    return ;
}

void  PlanningNode::triangle(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    return ;
}

void  PlanningNode::reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    return ;
}

void  PlanningNode::mass(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    return ;
}

void  PlanningNode::distrubute(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    
	auto selfIndex = platoonGetNumIndex(vehicle_num_list,platformParam.num);
	inputData.goalState.x = msg->distributePoints[selfIndex].position.x;
	inputData.goalState.y = msg->distributePoints[selfIndex].position.y;
	inputData.goalState.heading = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->distributePoints[selfIndex]));
	openSpaceGoalSet = true;
    return ;
}


void  PlanningNode::cancel_reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    return ;
}

void  PlanningNode::running(const platoon_msgs::PlatoonMission::ConstPtr &msg){
}


void PlanningNode::callbackDesireSpeed(const std_msgs::Float64::ConstPtr &msg)
{
    std::cout <<"callbackDesireSpeed:: "<<msg->data<<std::endl;
    velocityPlanner->setSpeed( msg->data);
}

void PlanningNode::callbackConflictConstraint(const planning_msgs::ConflictConstraint::ConstPtr &msg)
{
    conflict_constraint_processor_.updateConstraint(*msg);
}



void PlanningNode::callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg) {
	platoonMembers[msg->num] = *msg;
	return;
}


void PlanningNode::callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg) {
	assert(msg->command_type>=PlatoonType::BUILD);
	assert(msg->command_type<=PlatoonType::RUNNING);
    
	unsigned char  optType = msg->command_type;
    if ((optType != PlatoonType::BUILD) && (!platoonBuild))
		return;
    
    if ((optType == PlatoonType::BUILD) && (platoonBuild))
		return;	

	trajectoryType = optType;
	
	(this->*optsFunc[msg->command_type])(msg);
	return;
}


void PlanningNode::callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg) {

    platoon_config = *msg;
	return;
}


Obstacle*  PlanningNode::memberToObs(int num)
{
	//std::vector< const Obstacle*>().swap(obsList);
    platoon_msgs::PlatoonMember *mem = &platoonMembers[num];
	ugv::perception::PerceptionObstacle perceptionObstacle;
	geometry_msgs::Vector3 vec_to_center;
	
	double heading  = mem->heading;
	vec_to_center.x = (mem->front_overhang_m + mem->wheel_base_m - mem->rear_overhang_m) / 2.0;
	vec_to_center.y = 0.0;
	
	geometry_msgs::Vector3 g_vec_to_center;
    g_vec_to_center = amathutils::rotate(vec_to_center,heading);
	geometry_msgs::Point center ;
	center.x = mem->position.x + g_vec_to_center.x;
	center.y = mem->position.y + g_vec_to_center.y;

	double xrad = mem->vehicle_length_m/2.0;
	double yrad = mem->vehicle_width_m/2.0;
    Eigen::MatrixXd cpos(8, 2), rotyaw(2, 2), cpos_shift(8, 2);
    rotyaw << cos(heading), sin(heading), -sin(heading), cos(heading);
	
    cpos << -xrad, -yrad, -xrad, 0, -xrad, yrad, 0, yrad, xrad, yrad, xrad, 0,
        xrad, -yrad, 0, -yrad;
    double centerGlobal_x = center.x;
	double centerGlobal_y = center.y;
    cpos_shift << centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y
    	,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y;
    cpos = cpos * rotyaw + cpos_shift; 
  
    ugv::perception::PerceptionObstacle perception_obstacle;
    for (int i = 0; i < cpos.rows(); i++) {   
        perception_obstacle.polygon_point.push_back({cpos(i, 0),cpos(i, 1),0.0});
    }
	int id = 999;
    perception_obstacle.id = id;
    perception_obstacle.position.x = centerGlobal_x;
    perception_obstacle.position.y = centerGlobal_y;
    perception_obstacle.theta = heading;
    perception_obstacle.length = mem->vehicle_length_m;
    perception_obstacle.width  = mem->vehicle_width_m;

	auto obstaclePtr = new Obstacle(to_string(id),perception_obstacle,true);
	//obsList.push_back(obstaclePtr);
	//inputData.obsList = obsList;
	return  obstaclePtr;
}


void PlanningNode::loadPlanningParam(ros::NodeHandle &private_nh_)
{
    PlanningConfig *planning_config = PlanningConfig::get_instance();
	
	private_nh_.param<double>("kPathBoundsDeciderHorizon", planning_config->kPathBoundsDeciderHorizon, 100.0);
	private_nh_.param<double>("kPathBoundsDeciderResolution", planning_config->kPathBoundsDeciderResolution, 0.5);
	private_nh_.param<double>("kDefaultRoadWidth", planning_config->kDefaultRoadWidth, 20.0);
	private_nh_.param<double>("static_obstacle_nudge_l_buffer", planning_config->static_obstacle_nudge_l_buffer, 0.3);
	private_nh_.param<int>("kNumExtraTailBoundPoint", planning_config->kNumExtraTailBoundPoint, 20);
	private_nh_.param<bool>("enable_osqp_debug", planning_config->enable_osqp_debug, false);
	
	private_nh_.param<double>("trajectory_space_resolution", planning_config->trajectory_space_resolution, 1.0);
	private_nh_.param<double>("numerical_epsilon", planning_config->numerical_epsilon, 1e-6);
	private_nh_.param<bool>("enable_osqp_debug", planning_config->enable_osqp_debug, false);
	private_nh_.param<double>("static_obstacle_speed_threshold", planning_config->static_obstacle_speed_threshold, 0.5);

	private_nh_.param<double>("obstacle_lon_end_buffer", planning_config->obstacle_lon_end_buffer, 2.0);
	private_nh_.param<bool>("is_lane_borrowing", planning_config->boundryConfig.is_lane_borrowing, false);
	private_nh_.param<bool>("is_pull_over", planning_config->boundryConfig.is_pull_over, false);
	private_nh_.param<double>("pull_over_destination_to_adc_buffer", planning_config->boundryConfig.pull_over_destination_to_adc_buffer, 25.0);
	private_nh_.param<double>("pull_over_destination_to_pathend_buffer", planning_config->boundryConfig.pull_over_destination_to_pathend_buffer, 10.0);
	private_nh_.param<double>("pull_over_road_edge_buffer", planning_config->boundryConfig.pull_over_road_edge_buffer, 0.15);
	private_nh_.param<double>("pull_over_approach_lon_distance_adjust_factor", planning_config->boundryConfig.pull_over_approach_lon_distance_adjust_factor, 1.5);
	private_nh_.param<double>("adc_buffer_coeff", planning_config->boundryConfig.adc_buffer_coeff, 1.0);
	private_nh_.param<double>("lateral_derivative_bound_default", planning_config->lateral_derivative_bound_default, 2.0);
	private_nh_.param<double>("virtual_obs_length", planning_config->virtual_obs_length, 1.5);
	private_nh_.param<double>("virtual_obs_width", planning_config->virtual_obs_width, 1.0);
	private_nh_.param<bool>("virtual_moveing_obs", planning_config->virtual_moveing_obs, false);
	private_nh_.param<bool>("open_velocity_planner", planning_config->open_velocity_planner, true);
	private_nh_.param<bool>("open_path_planner", planning_config->open_path_planner, true);
	private_nh_.param<bool>("stop_obs_strategy", planning_config->stop_obs_strategy, false);

	private_nh_.param<bool>("enable_trajectory_stitcher", planning_config->enable_trajectory_stitcher, true);
	private_nh_.param<double>("replan_lateral_distance_threshold", planning_config->replan_lateral_distance_threshold, 0.5);
	private_nh_.param<double>("replan_longitudinal_distance_threshold", planning_config->replan_longitudinal_distance_threshold, 2.5);
	
	private_nh_.param<int>("prepoint_num", planning_config->prepoint_num, 10);
	private_nh_.param<double>("zudaunludaoche", planning_config->zudaunludaoche, 1.5);
	private_nh_.param<bool>("open_perception_obs", planning_config->open_perception_obs, false);
	private_nh_.param<int>("kNumExtraEndPoint", planning_config->kNumExtraEndPoint, 10);
	private_nh_.param<int>("noplan_points_num", planning_config->noplan_points_num, 8);
	

	private_nh_.param<double>("stopping_obs_distance", planning_config->stopping_obs_distance, 10.0);


	private_nh_.param<double>("kp", planning_config->pid_config_.kp_, 0.09);
	private_nh_.param<double>("ki", planning_config->pid_config_.ki_, 0.09);
	private_nh_.param<double>("kd", planning_config->pid_config_.kd_, 0.09);

	
	std::vector<std::string> area_str_v;
	area_str_v = common::getStrArrayParam(private_nh_,std::string("areas"),10);
	for(auto &area:area_str_v)
	{ 
	    PathingConfig area_config;
	    area_config.weight = common::getArrayParam(private_nh_,area+std::string("/weight"),4); 
	    area_config.expansion = common::getArrayParam(private_nh_,area+std::string("/expansion"),3);
	    private_nh_.param<double>(area+std::string("/lane_width"), area_config.kDefaultLaneWidth, 10.0);
		planning_config->pathingConfigs.insert(std::make_pair(area, area_config));
	}

	private_nh_.param<int>("mapParams_pointNum", planning_config->mapParams_pointNum, 360);
	private_nh_.param<double>("mapParams_resolution", planning_config->mapParams_resolution, 0.2);
	private_nh_.param<int>("mapParams_length", planning_config->mapParams_length, 40);
	private_nh_.param<int>("mapParams_width", planning_config->mapParams_width, 40);

	
}


void PlanningNode::callbackPlanningTimer(const ros::TimerEvent &event)
{  
    if (!pose_inited_)
		return ;
    PlanningConfig *planning_config = PlanningConfig::get_instance();
    PLANNER_TYPE planner_type = plannerTypeDecision();

    if (planner_type != OPEN_SPACE){
        if (!newTrajectory)
		    return ;
		newTrajectory = false;
    }
    planning_config->open_velocity_planner = true;
	if (planner_type == OPEN_SPACE ){
		if (!openSpaceGoalSet)
			return;
 		obsToGrid(obsList);
		planning_config->open_velocity_planner = false;
	}
	//sendHeart(1);
	//movingVitualObs();
	std::vector< const Obstacle*>().swap(inputData.obsList);
	inputData.obsList = obsList;
	if (leavingNum)
	    inputData.obsList.push_back(memberToObs(leavingNum));
		
	planning(planner_type);
	
}


void PlanningNode::obsToGrid(const std::vector< const Obstacle*> &obsList)
{	
	Point2D cur_veh_center(inputData.vehicleState.x,inputData.vehicleState.y);
	double veh_theta = inputData.vehicleState.heading;
	PlanningConfig *planning_config = PlanningConfig::get_instance();
    nav_msgs::OccupancyGrid occGrid;
	//障碍物转换为栅格
	//将障碍物增密并转换为点云格式，以便后续处理
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	vector<pcl::PointXYZI> obs_pcl;
    for (auto obs : obsList) {
        for (auto polygon_point : obs->Perception().polygon_point) {
            pcl::PointXYZI point;
			Point2D  local_polygon_point;
		    local_polygon_point = geometry_point::globalToLocal(cur_veh_center,veh_theta,Point2D(polygon_point.x,polygon_point.y)); 
            point.x = local_polygon_point.x;
            point.y = local_polygon_point.y;
            point.z = 2.5;
            point.intensity = 1;
			
            obs_pcl.push_back(point);
        }
        double min_x, max_x, min_y, max_y;
        computeBoundingBox(obs_pcl, min_x, max_x, min_y, max_y);
        float resolution = 0.1f;
        int num_points = static_cast<int>((max_x - min_x) / resolution) * static_cast<int>((max_y - min_y) / resolution);
        cloud->points.reserve(cloud->points.size() + num_points);
        for (float x = min_x; x <= max_x; x += resolution) {
            for (float y = min_y; y <= max_y; y += resolution) {
				pcl::PointXYZI point;
				point.x = x;
				point.y = y;
				point.z = 2.5;
				point.intensity = 1;
                cloud->points.push_back(point);
            }
        }
        obs_pcl.clear();
    }
	// for(auto obs : obsList){
	// 	for(auto polygon_point : obs->Perception().polygon_point)
	// 	{
	// 		vector<pcl::PointXYZI> obs_pcl;
	// 	    pcl::PointXYZI point;
	// 		point.x = polygon_point.x;
	// 		point.y = polygon_point.y;
	// 		point.z = 0;
	// 		point.intensity = 1;
	// 		obs_pcl.push_back(point);
	// 		double min_x, max_x, min_y, max_y;
	// 		computeBoundingBox(obs_pcl, min_x, max_x, min_y, max_y);
	// 		float resolution = 0.1f; // 点云密度：1米一个点
	// 		for (float x = min_x; x <= max_x; x += resolution) {
    //     		for (float y = min_y; y <= max_y; y += resolution) {
	// 				// 假设所有点的z坐标相同（或者你可以根据某种规则设置z坐标）
	// 				float z = 2.5; // 平均值，或者其他值
	// 				float intensity = 1.0; // 平均值，或者其他值
	// 				cloud->points.push_back(pcl::PointXYZI(x, y, z, intensity));
    //     		}
    // 		}		
	// 	}
	// }
	// 设置点云的宽度和高度（对于无序点云，宽度是点的数量，高度是1）
	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = true;
	// 转换为ROS点云消息
	 sensor_msgs::PointCloud2 output;
	 pcl::toROSMsg(*cloud, output);

	 // 设置输出点云的header（例如，设置frame_id和timestamp）
    output.header.frame_id = "map"; // 根据你的实际情况设置
    output.header.stamp = ros::Time::now(); // 或者使用特定的时间戳

    // 发布点云话题
    pub_pc.publish(output);
	 
	//将点云信息转换为栅格地图
	float *freeSpacePoints = (float*) calloc(planning_config->mapParams_pointNum,sizeof(float));

	if(freeSpacePoints != nullptr)
	{
		
		float max_dis = std::pow(planning_config->mapParams_length,2);

		for(int i = 0; i < planning_config->mapParams_pointNum; i++)
		{
			freeSpacePoints[i] = max_dis;
		}
	}
	else{
		std::cerr<<"WARNING : memory allocation error!"<<std::endl;
		return;
	}

	Eigen::MatrixXi freeGridMap;

	computeFreeSpacePoints(cloud,freeSpacePoints,planning_config->mapParams_pointNum);
	
	freeGridMapFilter(freeSpacePoints,freeGridMap);//可同行区域栅格地图待修改
	
	publishFreeSpaceGridMap(freeGridMap,occGrid);

	free(freeSpacePoints);

    inputData.occGrid = occGrid ;
	
}

void PlanningNode::publishFreeSpaceGridMap(Eigen::MatrixXi &freeSpaceGridMap, nav_msgs::OccupancyGrid& rosMap)
{
	// nav_msgs::OccupancyGrid rosMap;
	PlanningConfig *planning_config = PlanningConfig::get_instance();

	rosMap.info.resolution = planning_config->mapParams_resolution;
	rosMap.info.origin.position.x = - planning_config->mapParams_length;
	rosMap.info.origin.position.y = - planning_config->mapParams_length;
	rosMap.info.origin.position.z = 0.0;
	//注意占据栅格地图的坐标系与ros右手坐标系相差180度的旋转
	tf::Quaternion q;
	q.setRPY(0, 0, 0);// Y X Z
	rosMap.info.origin.orientation.x = q.x();
	rosMap.info.origin.orientation.y = q.y();
	rosMap.info.origin.orientation.z = q.z();//-1.0;
	rosMap.info.origin.orientation.w = q.w();
	// rosMap.info.origin.orientation = tf::createQuaternionMsgFromYaw(-3.14/2);

	rosMap.info.width = planning_config->mapParams_length * 2 / planning_config->mapParams_resolution;
	rosMap.info.height = planning_config->mapParams_length * 2 / planning_config->mapParams_resolution;
	int data_size = rosMap.info.width * rosMap.info.height;
	rosMap.data.resize(data_size);

	size_t k = 0;
	for(size_t c = 0; c < freeSpaceGridMap.cols(); c++)//列
	{
		for(size_t r = 0; r < freeSpaceGridMap.rows(); r++)//行
		{
			// rosMap.data[k] = k % 256;
			if (freeSpaceGridMap(r,c) == 0)
			{
				rosMap.data[k] = 0;
			}
			else if (freeSpaceGridMap(r,c) == 1)
			{
				rosMap.data[k] = 1;
			}
			else if (freeSpaceGridMap(r,c) == 2)
			{
				rosMap.data[k] = 25;
			}
			else if (freeSpaceGridMap(r,c) == 3)
			{
				rosMap.data[k] = 100;
			}
			else if (freeSpaceGridMap(r,c) == 5)
			{
				rosMap.data[k] = 150;
			}
			else if (freeSpaceGridMap(r,c) == 10)
			{
				rosMap.data[k] = 200;
			}
			else if (freeSpaceGridMap(r,c) == 11)
			{
				rosMap.data[k] = 200;
			}
			else if (freeSpaceGridMap(r,c) == 12)
			{
				rosMap.data[k] = 200;
			}
			else if (freeSpaceGridMap(r,c) == 13)
			{
				rosMap.data[k] = 200;
			}
			k++;
		}
	}

	// rosMap.header.stamp = cloudHeader.stamp;
	// rosMap.header.frame_id = pointCloud_frame_id;

	rosMap.header.stamp = ros::Time::now(); // 使用当前时间
    rosMap.header.frame_id = "vehcile"; // 使用默认帧ID

	pubFreeSpaceMap.publish(rosMap);
	// ROS_INFO("pubFreeSpaceMap!");s
}



void PlanningNode::freeGridMapFilter(float* freeSpacePoints, Eigen::MatrixXi &dst)
{
	PlanningConfig *planning_config = PlanningConfig::get_instance();
	float pixel_size = planning_config->mapParams_resolution;
	float delta_r = pixel_size * 0.75;
	float delta_d_in_r = pixel_size * 0.65;
	int maxlength = planning_config->mapParams_length * 2 / pixel_size, maxwidth = planning_config->mapParams_width/pixel_size;
	
	Eigen::MatrixXi src = Eigen::MatrixXi::Zero(maxlength, maxlength);
	dst = Eigen::MatrixXi::Zero(maxlength, maxlength);
    // std::cout <<"b  ------------"<<std::endl;

	int pointNum = planning_config->mapParams_pointNum;
	float max_dis = planning_config->mapParams_length;//(mapParams.length > mapParams.width ? mapParams.length : mapParams.width) / 2;
	float alpha = planning_config->mapParams_pointNum / 360.0;
	float theta_border = M_PI / planning_config->mapParams_pointNum * 1.2;
	std::vector<float> delta_t;
	for (float j = 0.0001; j < max_dis; j += delta_r) // Prepare the delta theta of different radius
	{
		delta_t.push_back(delta_d_in_r/j);//不同半径下，弧长分辨率对应的角度分辨率
	}

	
	for (int i = 0; i < pointNum; i++)//遍历每一个需要采集的非地面激光点
	{

		
		float r = min(freeSpacePoints[i], freeSpacePoints[(i + 1) % pointNum]);
		r = min(r, freeSpacePoints[(i - 1 + pointNum) % pointNum]);
		r = sqrt(r);                   
		int k = 0;
		for (float j = 0; j < r - 0.5; j += delta_r)
		{
			float dt = delta_t[k++];
			float theta = (i / alpha - 180)*M_PI/180.0;                   
	
			for (float t = theta - theta_border; t < theta + theta_border; t+=dt)
			{
				float x = j*cos(t);
				float y = j*sin(t);
				int m = int((planning_config->mapParams_length + x) / pixel_size);//int((mapParams.offset_x - x) / pixel_size);               
				int n = int((planning_config->mapParams_length + y) / pixel_size);//int((mapParams.offset_y - y) / pixel_size);
				// if (m >= 0 && m < maxlength && n >= 0 && n < maxwidth) 
					src(m, n) = 1;
#ifndef EDGE_PROCESS
					dst(m, n) = 2;
#endif
			}
		}
	}
// #ifdef EDGE_PROCESS
// 	for(int i = 0; i < mapParams.pointNum; i++)
// 	{
		
// 		float angle = (i / alpha - 180);
// 		if(angle < mapParams.scan_angle_r || angle > mapParams.scan_angle_l)
// 			continue;


// 		for(float j = 0; j < max_dis -1; j += delta_r)
// 		{
// 			float x = j * cos((i / alpha - 180) * M_PI /180.0);
// 			float y = j * sin((i / alpha - 180) * M_PI /180.0);
// 			int m = int((mapParams.length + x) / pixel_size);//int((mapParams.offset_x - x) / pixel_size);
// 			int n = int((mapParams.length + y) / pixel_size);//int((mapParams.offset_y - y) / pixel_size);
// 			int theta = int(atan2f(y, x) * 180.0 / M_PI + 180.0 + 0.5);
// 			theta = theta % pointNum;
// 			float r = std::min(freeSpacePoints[theta],freeSpacePoints[(theta+1) % pointNum]);
// 			r = std::min(r,freeSpacePoints[(theta-1+pointNum) % pointNum]);

// 			if(r > j*j +1)   
// 			{
// 				int result = 0;
// 				for(int k = 0; k < 16; k++)
// 				{
// 					// if ((m  + filter_x[k]) >= 0 && (m  + filter_x[k]) < maxlength && 
// 					//     (n + filter_y[k]) >= 0 && (n + filter_y[k]) < maxwidth) 
// 					result += src(m + smaller_filter_x[k], n + smaller_filter_y[k]);
// 				}
// 				if(result < 16)
// 					break;
// 				for (int k = 0; k < 37; k++)               
// 				{
// 					// if ((m + all_x[k]) >= 0 && (m + all_x[k]) < maxlength &&
// 					//     (n + all_y[k]) >= 0 && (n + all_y[k]) < maxwidth)
// 					dst(m+smaller_all_x[k], n+smaller_all_y[k]) = max(1, dst(m+smaller_all_x[k], n+smaller_all_y[k]));
// 				}
// 				dst(m,n) = 2;
// 			}
// 		}
// 	} 
// #endif
}

void PlanningNode::computeFreeSpacePoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloudIn, float* free_space, int free_space_n)
{
	int thetaId;
	float distance_cur;
	size_t pointsNum = pointCloudIn->points.size();

	float alpha = free_space_n / 360;
	for(size_t pid = 0; pid < pointsNum; pid++) //遍历每一个非地面激光点
	{

		if(pointCloudIn->points[pid].z < 2.6)
		{
			distance_cur = std::pow(pointCloudIn->points[pid].x,2) + std::pow(pointCloudIn->points[pid].y,2);

		
			//atan2(y,x) x前方为0度，逆时针0～180，顺时针0～-180
			thetaId = int((atan2f(pointCloudIn->points[pid].y,pointCloudIn->points[pid].x) + M_PI) * 180.0 * alpha /M_PI  + 0.5); //当前激光点对应方向角度 × alpha = 当前激光点对应需要采集的非地面激光点id 
			thetaId = thetaId % free_space_n;
			if(free_space[thetaId] > distance_cur && distance_cur > 1)
			{
				free_space[thetaId] = distance_cur; //得到最小距离
			}
		}
	}

}	

void PlanningNode::computeBoundingBox(const vector<pcl::PointXYZI> obs_pcl,double& min_x, double& max_x, double& min_y, double& max_y)
{
	if (obs_pcl.empty()) {
        // 如果vector为空，则设置默认值为未定义状态（这里简单地设置为0，但实际应用中可能需要更明确的处理方式）
        min_x = max_x = min_y = max_y = 0;
        std::cerr << "Warning: The input vector is empty!" << std::endl;
        return;
    }

    max_x = min_x = obs_pcl[0].x;
    max_y = min_y = obs_pcl[0].y;

    for (const auto& pcl_point : obs_pcl) {
        if (pcl_point.x > max_x) max_x = pcl_point.x;
        if (pcl_point.x < min_x) min_x = pcl_point.x;
        if (pcl_point.y > max_y) max_y = pcl_point.y;
        if (pcl_point.y < min_y) min_y = pcl_point.y;
    }
}



void PlanningNode::callBackGoal(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    
    std::cout <<"get callBackGoal"<<std::endl;
	double yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->pose));
	inputData.goalState.x = msg->pose.position.x;
	inputData.goalState.y = msg->pose.position.y;
	inputData.goalState.heading = yaw;
	openSpaceGoalSet = true; 
	//是否要添加到达终点以后，设置为false	
}



void PlanningNode::callbackReplan(const route_msgs::Replan::ConstPtr &msg)
{
    std::cout << "[planning_node] ====== 进入重规划回调函数 ======" << std::endl;

    if (msg->points.empty()) {
        std::cout<< "[planning_node] 重规划消息中无障碍物点，跳过处理" << std::endl;
        return;
    }

    int obstacle_count = msg->points.size();
    for (int i = 0; i < obstacle_count; ++i) {
		const geometry_msgs::Point& target_point = msg->points[i];
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.pose.pose.position = target_point;
		pose_msg.pose.pose.orientation.x = 0.0;
        pose_msg.pose.pose.orientation.y = 0.0;
        pose_msg.pose.pose.orientation.z = 0.0;
        pose_msg.pose.pose.orientation.w = 1.0;
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose_msg_ptr = 
            boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(pose_msg);
        callBackinitialPose(pose_msg_ptr);
    }
}


void PlanningNode::callBackMultiPointPlanning(const route_msgs::MultiPoint::ConstPtr msg)
{
	 std::cout << "[PlanningNode] ====== 清除障碍物=====" << std::endl;
	    
	 std::vector< const Obstacle*>().swap(obsList);
	 std::vector< const Obstacle*>().swap(inputData.obsList);
}


void PlanningNode::callBackinitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {

	PlanningConfig *planning_config = PlanningConfig::get_instance();
	
	vehicle_info_util::VehicleInfoUtil *vehicle_util = vehicle_info_util::VehicleInfoUtil::get_instance();
    static int id = 1;
    std::cout <<"get callBackinitialPose"<<std::endl;
	
    geometry_msgs::Pose pose = msg->pose.pose;
    //障碍物的八个点转换到全局坐标系下
    Eigen::MatrixXd cpos(8, 2), rotyaw(2, 2), cpos_shift(8, 2);
    double xrad = planning_config->virtual_obs_length/2.0; //障碍物的坐标系为前左上
    double yrad = planning_config->virtual_obs_width /2.0;
    double length = planning_config->virtual_obs_length;
    double width = planning_config->virtual_obs_width;
	
    double yaw  = amathutils::getPoseYawAngle(pose);
    double centerGlobal_x = pose.position.x;
    double centerGlobal_y = pose.position.y;

	if (planning_config->virtual_moveing_obs)
	{
	    std::vector<ReferencePoint> refPoints;
	    trajMsg2RefPoints(inputData.refArray,refPoints);
		ReferenceLine refenceLine(refPoints);
		double lane_left_width,lane_right_width;
		refenceLine.GetLaneWidth(0,&lane_left_width,&lane_right_width);
	    PathPoint machPoint= PathMatcher::MatchToPath(refenceLine.ToDiscretizedReferenceLine(refPoints),pose.position.x,pose.position.y);

        SLPoint slPoint;
		slPoint.set_s(machPoint.s());
		SLPoint slPoint_pose;
        refenceLine.XYToSL(Vec2d(pose.position.x,pose.position.y),&slPoint_pose);
		if (slPoint_pose.l() < 0)
		    slPoint.set_l(-lane_left_width/2.0);
		else
		    slPoint.set_l(lane_left_width/3.0);

		Vec2d xy;
		VitualInfo	obsInfo;
		refenceLine.SLToXY(slPoint, &xy);
		centerGlobal_x = xy.x();
		centerGlobal_y = xy.y();
 		if (fabs(yaw - machPoint.theta()) > M_PI/2.0){
			yaw = amathutils::normalizeRadian(machPoint.theta() + M_PI);	
			obsInfo.director = -1;
 		}
		 else{
			yaw = machPoint.theta();
		    obsInfo.director = 1;
		 }

		 xrad = vehicle_util->vehicle_length_m/2.0;
	     yrad = vehicle_util->vehicle_width_m/2.0;
		 length = vehicle_util->vehicle_length_m;
	     width = vehicle_util->vehicle_width_m;

		 obsInfo.l = slPoint.l();
	     obsInfo.s = slPoint.s();
		 obsInfo.refLine = refenceLine;
		 virtualInfo.push_back(obsInfo);
	}
    
    
    rotyaw << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);
	
    cpos << -xrad, -yrad, -xrad, 0, -xrad, yrad, 0, yrad, xrad, yrad, xrad, 0,
        xrad, -yrad, 0, -yrad;
  

	
    cpos_shift << centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y
    	,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y;
    cpos = cpos * rotyaw + cpos_shift; 

    struct ugv::perception::PerceptionObstacle perception_obstacle;
    for (int i = 0; i < cpos.rows(); i++) {
	    perception_obstacle.polygon_point.push_back({cpos(i, 0),cpos(i, 1),0.0});
    }
    perception_obstacle.id = id;
    perception_obstacle.position.x = centerGlobal_x;
    perception_obstacle.position.y = centerGlobal_y;
    perception_obstacle.theta = yaw;
    perception_obstacle.length = length;
    perception_obstacle.width = width;

	auto obstaclePtr = new Obstacle(to_string(id),perception_obstacle,true);
	obsList.push_back(obstaclePtr);
    id++;
	collectDiplayObsInfo(obsList);
    is_obs_update = true;
}



void PlanningNode::callbackReferenceLine(const planning_msgs::TrajectoryPointArray::Ptr msg)
{
    if (!msg->points.size())
		return ;

	//std::cout <<"------------callbackReferenceLine---------------"<<std::endl;
	newTrajectory = true;
	inputData.refArray = *msg;
	
}


void PlanningNode::callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg)
{

	vehicle_info_util::VehicleInfoUtil *vehicle_util = vehicle_info_util::VehicleInfoUtil::get_instance();
	
	current_chassis = *msg;
	
	current_velocity = (7 == msg->gear_location)?(-msg->current_velocity):msg->current_velocity;
	current_chassis.current_velocity  = current_velocity;
	recordActualVelocity(msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp, current_velocity);

	cur_vehicle_state.steering  = (double)(msg->steering_wheel_angle )/vehicle_util->w2s_primary_coeff/ 180.0 * M_PI;
    cur_vehicle_state.v = current_velocity;
	cur_vehicle_state.kappa = std::tan(cur_vehicle_state.steering) / vehicle_util->wheel_base_m;
    
}


void PlanningNode::callbackPose(const localization_msgs::Localization::ConstPtr &msg)
{
    /*
    boost::shared_ptr<nav_msgs::Odometry> odom_ptr(new nav_msgs::Odometry);
	odom_ptr->pose.pose.position.x = msg->utm_x;
	odom_ptr->pose.pose.position.y = msg->utm_y;
	double norm_yaw = normalized_angle(msg->yaw);
	norm_yaw = norm_yaw * 3.141592653589793 / 180.0;

	odom_ptr->pose.pose.orientation.w = std::cos(norm_yaw * 0.5);
	odom_ptr->pose.pose.orientation.z = std::sin(norm_yaw * 0.5);
	odom_ptr->pose.pose.orientation.y = 0;
	odom_ptr->pose.pose.orientation.x = 0;
	*/
	 double deg2rad = 3.1415926 /  180.0;
	 yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2 + heading_compensation_degree * deg2rad );
	 
	//yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2);	
	current_pose_ = amathutils::getOdometryFromPosAndYaw(msg->location.pose.pose.position,yaw);	

	cur_vehicle_state.x = msg->location.pose.pose.position.x;
	cur_vehicle_state.y = msg->location.pose.pose.position.y;
	cur_vehicle_state.heading = yaw;

	inputData.vehicleState = cur_vehicle_state;
	navUncertainty = msg->original_ins.nav_uncertainty;
	pose_inited_ = true;
}



void PlanningNode::callbackObstacles(const perception_msgs::PredictionObstacles::ConstPtr &msg)
{
    //std::cout <<"callback obstacles"<<std::endl;   
	PlanningConfig *planning_config = PlanningConfig::get_instance();
	//if (!planning_config->open_perception_obs)
		//return;
	
    obs = *msg;
	std::vector< const Obstacle*>().swap(obsList);
	for (auto &prediction_obstacle :msg->prediction_obstacles){
		
		perception_msgs::PerceptionObstacle perception_obstacle = prediction_obstacle.perception_obstacle;
        bool is_static = prediction_obstacle.is_static;
	    ugv::perception::PerceptionObstacle perceptionObstacle;

		perceptionObstacle.id = perception_obstacle.id;
		perceptionObstacle.position.x = perception_obstacle.position.x;
	    perceptionObstacle.position.y = perception_obstacle.position.y;
	    perceptionObstacle.position.z = perception_obstacle.position.z;

	    perceptionObstacle.theta = perception_obstacle.angle;
		
	    perceptionObstacle.velocity.x  = perception_obstacle.velocity.x;
    	perceptionObstacle.velocity.y  = perception_obstacle.velocity.y;
	    perceptionObstacle.velocity.z  = perception_obstacle.velocity.z;
		perceptionObstacle.type = perception_obstacle.type;

		if (perception_obstacle.length < 1e-6)
			continue;
		if (perception_obstacle.width < 1e-6)
			continue;
		if (perception_obstacle.polygon.points.size() <= 2)
			continue;

		//if (5 == perception_obstacle.type && last_area != std::string("road_obstacle_area"))
			///continue;
		
		if (5 == perception_obstacle.type) continue;

		perceptionObstacle.length  = perception_obstacle.length;
	    perceptionObstacle.width   = perception_obstacle.width;
	    perceptionObstacle.height  = perception_obstacle.height;

        /*
		int polygon_point_num = 0;
		
		for (const auto& point : perception_obstacle.polygon.points){

			polygon_point_num++;
			if ((perception_obstacle.type != 99) && polygon_point_num <= 8)
				continue;
			
		    ugv::perception::Point3D point3d; 
		    point3d.x = point.x;point3d.y = point.y;
		    perceptionObstacle.polygon_point.push_back(point3d);
		}
		*/

		int polygon_point_num = 0;
		for (const auto& point : perception_obstacle.polygon.points){

			polygon_point_num++;
			if ((perception_obstacle.polygon.points.size() > 8) && polygon_point_num <= 8)
				continue;
			
		    ugv::perception::Point3D point3d; 
		    point3d.x = point.x;point3d.y = point.y;
		    perceptionObstacle.polygon_point.push_back(point3d);
		}


        Trajectory trajectory;
		for (const auto& predictPoint : prediction_obstacle.trajectory){
			TrajectoryPoint p;
			p.path_point_.set_x(predictPoint.x);
			p.path_point_.set_y(predictPoint.y);
			p.path_point_.set_z(predictPoint.z);
			p.path_point_.set_s(predictPoint.s);
			p.path_point_.set_theta(predictPoint.theta);
			p.path_point_.set_kappa(predictPoint.kappa);
			p.path_point_.set_dkappa(predictPoint.dkappa);
			p.set_v(predictPoint.v) ;
			p.set_a(predictPoint.a) ;
			p.set_relative_time(predictPoint.relative_time);
			trajectory.trajectory_point.push_back(p);
		}
		
		Obstacle *obsPtr;
		if (!is_static)
	        obsPtr = new Obstacle(to_string(perception_obstacle.id),perceptionObstacle,trajectory,is_static);
		else
		    obsPtr = new Obstacle(to_string(perception_obstacle.id),perceptionObstacle,is_static);
		obsList.push_back(obsPtr);
	}
    std::vector< const Obstacle*>().swap(inputData.obsList);
	inputData.obsList = obsList;
	collectDiplayObsInfo(obsList);
    is_obs_update = true;
	
}

void PlanningNode::callbackGlobalPath84InPlanning(const lanelet_map_msgs::Way::ConstPtr &msg)
{
    static int sameCount = 0;

	static lanelet_map_msgs::Way lastWay;

	if (!lastWay.points.size()){
		lastWay = *msg;
		return ;
	}
	
	if (lastWay == *msg)
		sameCount++;
	else{
	 	sameCount = 0;
		specialSituation = false;
	}
	
	lastWay = *msg;
	
	if (sameCount > 300){
	    specialSituation = true;
		std::cout <<"special situation accured "<<std::endl;
	}
}




void PlanningNode::collectDisplayInfo(const       planning_msgs::TrajectoryPointArray *planned_trajectory,
const TrajectoryPoint *planning_start_point  ,const PathBoundary *lane_boundry ,const PathBoundary *planning_boundry ,const ReferenceLine *reference_line )  
{  
    lane_boundry_.clear();
    planning_boundry_.clear();
	
    if (lane_boundry)
        lane_boundry_ = *lane_boundry;
	if (planning_boundry)
	    planning_boundry_ = *planning_boundry;
	if (reference_line)
	    reference_line_ = *reference_line;
	
	planned_trajectory_ = *planned_trajectory;
	if (planning_start_point)
	    planning_start_point_ = *planning_start_point;
	newBoundry = true;
}


void PlanningNode::displayLoop()
{
	 while (ros::ok()) {
	 	if ((!newBoundry) && (!is_obs_update)){
			usleep(1000);
			continue;
	 	}
		
		if (newBoundry){
			visualization_msgs::MarkerArray markerArray;
			newBoundry = false;
			std::vector<geometry_msgs::Point> left_lane_bound,right_lane_bound;
			std::vector<geometry_msgs::Point> left_planning_bound,right_planning_bound;
			generateBound(reference_line_,lane_boundry_,left_lane_bound,right_lane_bound);
			generateBound(reference_line_,planning_boundry_,left_planning_bound,right_planning_bound);

            DisplayConfig cfg;
			
			if (!left_lane_bound.empty()){
		    	cfg.id = 1;
				cfg.r = cfg.b = cfg.g = 1.0;
			    cfg.scale_x = 0.02;
				cfg.ns = std::string("/planning/bound");
		        markerArray.markers.push_back(DisPlay::lineMarker(left_lane_bound,cfg));
			}
			if (!right_lane_bound.empty()){
			    cfg.id = 2;
				cfg.r = cfg.b = cfg.g = 1.0;
			    cfg.scale_x = 0.02;
				cfg.ns = std::string("/planning/bound");
			    markerArray.markers.push_back(DisPlay::lineMarker(right_lane_bound,cfg));
			}
			if (!right_planning_bound.empty()){
			    cfg.id = 3;
				cfg.r = 1.0;
				cfg.b = cfg.g = 0.0;
			    cfg.scale_x = 0.02;
				cfg.ns = std::string("/planning/bound");
			    markerArray.markers.push_back(DisPlay::lineMarker(right_planning_bound,cfg));
			}
			if (!left_planning_bound.empty()){
				cfg.id = 4;
				cfg.r = 1.0;
				cfg.b = cfg.g = 0.0;
				cfg.scale_x = 0.02;
				cfg.ns = std::string("/planning/bound");
				markerArray.markers.push_back(DisPlay::lineMarker(left_planning_bound,cfg));
			}
			
            cfg.id = 1;
		    cfg.r = cfg.g = 1.0;cfg.b = 0.0;
			cfg.scale_x = 0.02;
			cfg.ns = std::string("/planning/trajectory");
			if (planned_trajectory_.points.size()> 2)
			    markerArray.markers.push_back(DisPlay::lineMarker(planned_trajectory_.points,cfg));

			geometry_msgs::Point planing_start_point;
			planing_start_point.x = planning_start_point_.path_point().x();
			planing_start_point.y = planning_start_point_.path_point().y();
			std::vector<geometry_msgs::Point> planing_start_point_v;
			planing_start_point_v.push_back(planing_start_point);
			cfg.id = 2;
		    cfg.r = 1.0;
		    cfg.g = cfg.b = 0.0;
			cfg.scale_x = 0.5;
			cfg.scale_y = 0.5;
			cfg.ns = std::string("/planning/trajectory");
			markerArray.markers.push_back(DisPlay::pointsMarker(planing_start_point_v,cfg));
			
		    pub_trajectory.publish(markerArray);
		}
        
        if (is_obs_update){
			is_obs_update = false;
			visualization_msgs::MarkerArray markerArray;
			std::vector<bool> obses_is_static;
			
			mtx.lock();

			DisplayConfig cfg;
			cfg.id = 1;
			cfg.scale_x = 0.05;
			cfg.ns = std::string("/planning/obstacles");
			visualization_msgs::MarkerArray ploygonArray;
			for (const auto &obs:obs_vec){
				geometry_msgs::Polygon  polygon;
				geometry_msgs::Point32 p;
			    for (const auto& point : obs.Perception().polygon_point) {
					p.x = point.x; p.y = point.y;
			        polygon.points.push_back(p);
			    }
	            if (obs.IsStatic()){
					cfg.r = 1.0;
					cfg.g = 0.0;
				    cfg.b = 0.0;
	            }
				else{
					cfg.r = 0.0;
					cfg.g = 0.0;
					cfg.b = 1.0;
				}
				cfg.id++;
				ploygonArray.markers.push_back(DisPlay::plogonMarker(polygon,cfg));
			}
			mtx.unlock();

            if (!ploygonArray.markers.empty()){
			    markerArray.markers.insert(markerArray.markers.end(),ploygonArray.markers.begin(),ploygonArray.markers.end());
	    	    pub_obs.publish(markerArray);
            }
        }
        
	}
}

void PlanningNode::publishVelocityCurveMarker(
	const planning_msgs::TrajectoryPointArray &candidate_trajectory,
	const planning_msgs::TrajectoryPointArray &final_trajectory)
{
	visualization_msgs::MarkerArray marker_array;
	marker_array.markers.push_back(makeDeleteAllMarker());

	if (!enable_velocity_curve_marker_) {
		pub_velocity_curve.publish(marker_array);
		return;
	}

	if (final_trajectory.points.empty()) {
		pub_velocity_curve.publish(marker_array);
		return;
	}

	const double time_horizon = std::max(0.5, velocity_curve_time_horizon_);
	const double time_scale = std::max(0.1, velocity_curve_time_scale_);
	const double speed_scale = std::max(0.1, velocity_curve_speed_scale_);

	// 曲线默认挂在当前车辆左前方；若定位还未初始化，则退化为挂在轨迹首点附近。
	geometry_msgs::Point origin;
	double yaw_for_curve = 0.0;
	if (pose_inited_) {
		origin = current_pose_.pose.pose.position;
		yaw_for_curve = amathutils::getPoseYawAngle(current_pose_.pose.pose);
	} else {
		origin.x = final_trajectory.points.front().x;
		origin.y = final_trajectory.points.front().y;
		origin.z = 0.0;
	}

	double max_speed = 1.0;
	const auto collect_max_speed = [&](const planning_msgs::TrajectoryPointArray &trajectory) {
		for (const auto &point : trajectory.points) {
			if (point.relative_time > time_horizon) {
				break;
			}
			max_speed = std::max(max_speed, std::max(0.0, point.v));
		}
	};
	collect_max_speed(candidate_trajectory);
	collect_max_speed(final_trajectory);

	const double front_offset = velocity_curve_front_offset_;
	const double left_offset = velocity_curve_left_offset_;
	const double axis_z = 1.0;

	const std_msgs::ColorRGBA axis_color = makeColor(1.0f, 1.0f, 1.0f, 0.65f);
	const std_msgs::ColorRGBA candidate_color = makeColor(0.05f, 0.35f, 1.0f, 0.75f);
	const std_msgs::ColorRGBA final_color = makeColor(0.0f, 1.0f, 0.25f, 0.95f);

	// 坐标轴：横轴为未来时间，纵轴为速度。该图只是调试图，不参与规划计算。
	visualization_msgs::Marker time_axis;
	time_axis.header.frame_id = "map";
	time_axis.header.stamp = ros::Time::now();
	time_axis.ns = "velocity_curve/axis";
	time_axis.id = 1;
	time_axis.type = visualization_msgs::Marker::LINE_STRIP;
	time_axis.action = visualization_msgs::Marker::ADD;
	time_axis.pose.orientation.w = 1.0;
	time_axis.scale.x = 0.04;
	time_axis.color = axis_color;
	time_axis.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve, front_offset, left_offset, axis_z));
	time_axis.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve,
		front_offset + time_horizon * time_scale, left_offset, axis_z));
	marker_array.markers.push_back(time_axis);

	visualization_msgs::Marker speed_axis = time_axis;
	speed_axis.id = 2;
	speed_axis.points.clear();
	speed_axis.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve, front_offset, left_offset, axis_z));
	speed_axis.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve,
		front_offset, left_offset + max_speed * speed_scale, axis_z));
	marker_array.markers.push_back(speed_axis);

	if (!candidate_trajectory.points.empty()) {
		// candidate 表示基础速度规划结果，冲突消解开启时可用于对比消解前后的变化。
		marker_array.markers.push_back(makeVelocityCurveLine(
			candidate_trajectory, origin, yaw_for_curve, front_offset, left_offset,
			time_horizon, time_scale, speed_scale, "velocity_curve/candidate", 3, candidate_color));
	}

	// final 表示真正发布给控制器的速度规划结果，冲突修正后的降速/停车都体现在这条线上。
	marker_array.markers.push_back(makeVelocityCurveLine(
		final_trajectory, origin, yaw_for_curve, front_offset, left_offset,
		time_horizon, time_scale, speed_scale, "velocity_curve/final", 4, final_color));

	marker_array.markers.push_back(makeVelocityCurveText(
		makeVelocityCurvePoint(origin, yaw_for_curve, front_offset, left_offset - 0.8, 1.4),
		"v-t curve", "velocity_curve/text", 5, axis_color, 0.45));
	marker_array.markers.push_back(makeVelocityCurveText(
		makeVelocityCurvePoint(origin, yaw_for_curve, front_offset + time_horizon * time_scale + 0.4,
			left_offset, 1.4),
		"t", "velocity_curve/text", 6, axis_color, 0.4));

	std::ostringstream speed_label;
	speed_label << std::fixed << std::setprecision(1) << max_speed << " m/s";
	marker_array.markers.push_back(makeVelocityCurveText(
		makeVelocityCurvePoint(origin, yaw_for_curve, front_offset,
			left_offset + max_speed * speed_scale + 0.4, 1.4),
		speed_label.str(), "velocity_curve/text", 7, axis_color, 0.4));
	marker_array.markers.push_back(makeVelocityCurveText(
		makeVelocityCurvePoint(origin, yaw_for_curve, front_offset + 1.2,
			left_offset + max_speed * speed_scale + 0.9, 1.4),
		"blue:candidate  green:final", "velocity_curve/text", 8, axis_color, 0.35));

	pub_velocity_curve.publish(marker_array);
}

void PlanningNode::recordActualVelocity(const ros::Time &stamp, const double velocity)
{
	const double now = stamp.toSec();
	actual_velocity_history_.emplace_back(now, velocity);

	const double history_duration = std::max(1.0, actual_velocity_curve_history_duration_);
	while (!actual_velocity_history_.empty() &&
	       now - actual_velocity_history_.front().first > history_duration) {
		actual_velocity_history_.pop_front();
	}
}

void PlanningNode::publishActualVelocityCurveMarker()
{
	visualization_msgs::MarkerArray marker_array;
	marker_array.markers.push_back(makeDeleteAllMarker());

	if (!enable_actual_velocity_curve_marker_) {
		pub_actual_velocity_curve.publish(marker_array);
		return;
	}

	if (!pose_inited_ || actual_velocity_history_.empty()) {
		pub_actual_velocity_curve.publish(marker_array);
		return;
	}

	const double history_duration = std::max(1.0, actual_velocity_curve_history_duration_);
	const double time_scale = std::max(0.1, actual_velocity_curve_time_scale_);
	const double speed_scale = std::max(0.1, actual_velocity_curve_speed_scale_);
	const double front_offset = actual_velocity_curve_front_offset_;
	const double side_offset = actual_velocity_curve_side_offset_;
	const double axis_z = 1.0;

	const geometry_msgs::Point origin = current_pose_.pose.pose.position;
	const double yaw_for_curve = amathutils::getPoseYawAngle(current_pose_.pose.pose);
	const double now = ros::Time::now().toSec();

	double max_speed = 1.0;
	for (const auto &sample : actual_velocity_history_) {
		max_speed = std::max(max_speed, std::fabs(sample.second));
	}

	const std_msgs::ColorRGBA axis_color = makeColor(1.0f, 1.0f, 1.0f, 0.65f);
	const std_msgs::ColorRGBA actual_color = makeColor(1.0f, 0.85f, 0.05f, 0.95f);

	// 坐标轴：横轴从过去 history_duration 秒延伸到当前时刻，纵轴为实际车速绝对值。
	visualization_msgs::Marker time_axis;
	time_axis.header.frame_id = "map";
	time_axis.header.stamp = ros::Time::now();
	time_axis.ns = "actual_velocity_curve/axis";
	time_axis.id = 1;
	time_axis.type = visualization_msgs::Marker::LINE_STRIP;
	time_axis.action = visualization_msgs::Marker::ADD;
	time_axis.pose.orientation.w = 1.0;
	time_axis.scale.x = 0.04;
	time_axis.color = axis_color;
	time_axis.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve,
		front_offset - history_duration * time_scale, side_offset, axis_z));
	time_axis.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve,
		front_offset, side_offset, axis_z));
	marker_array.markers.push_back(time_axis);

	visualization_msgs::Marker speed_axis = time_axis;
	speed_axis.id = 2;
	speed_axis.points.clear();
	speed_axis.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve,
		front_offset - history_duration * time_scale, side_offset, axis_z));
	speed_axis.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve,
		front_offset - history_duration * time_scale, side_offset + max_speed * speed_scale, axis_z));
	marker_array.markers.push_back(speed_axis);

	visualization_msgs::Marker actual_curve;
	actual_curve.header.frame_id = "map";
	actual_curve.header.stamp = ros::Time::now();
	actual_curve.ns = "actual_velocity_curve/speed";
	actual_curve.id = 3;
	actual_curve.type = visualization_msgs::Marker::LINE_STRIP;
	actual_curve.action = visualization_msgs::Marker::ADD;
	actual_curve.pose.orientation.w = 1.0;
	actual_curve.scale.x = 0.08;
	actual_curve.color = actual_color;
	for (const auto &sample : actual_velocity_history_) {
		const double age = std::max(0.0, now - sample.first);
		if (age > history_duration) {
			continue;
		}
		// 当前时刻位于横轴右端，越早的实际速度点越靠左。
		actual_curve.points.push_back(makeVelocityCurvePoint(origin, yaw_for_curve,
			front_offset - age * time_scale, side_offset + std::fabs(sample.second) * speed_scale, 1.2));
	}
	marker_array.markers.push_back(actual_curve);

	marker_array.markers.push_back(makeVelocityCurveText(
		makeVelocityCurvePoint(origin, yaw_for_curve,
			front_offset - history_duration * time_scale, side_offset - 0.8, 1.4),
		"actual v history", "actual_velocity_curve/text", 4, axis_color, 0.45));
	marker_array.markers.push_back(makeVelocityCurveText(
		makeVelocityCurvePoint(origin, yaw_for_curve, front_offset + 0.4, side_offset, 1.4),
		"now", "actual_velocity_curve/text", 5, axis_color, 0.4));

	std::ostringstream speed_label;
	speed_label << std::fixed << std::setprecision(1) << max_speed << " m/s";
	marker_array.markers.push_back(makeVelocityCurveText(
		makeVelocityCurvePoint(origin, yaw_for_curve,
			front_offset - history_duration * time_scale,
			side_offset + max_speed * speed_scale + 0.4, 1.4),
		speed_label.str(), "actual_velocity_curve/text", 6, axis_color, 0.4));

	pub_actual_velocity_curve.publish(marker_array);
}

void PlanningNode::collectDiplayObsInfo(const std::vector<const Obstacle *> obs_list)  
{  
    
	mtx.lock();
    std::vector<Obstacle>().swap(obs_vec);
	for (const auto &obs_:obsList){
        Obstacle  obs = *obs_;
		obs_vec.push_back(obs);
	}
	mtx.unlock();
}



void PlanningNode::generateBound(ReferenceLine &refLine,PathBoundary &boundry,
std::vector<geometry_msgs::Point> &left,std::vector<geometry_msgs::Point> &right)
{
	size_t path_boundary_size = boundry.boundary().size();
	for (size_t i = 0; i < path_boundary_size; ++i) {
		double s = static_cast<double>(i) * boundry.delta_s() +
			  boundry.start_s();
		SLPoint sl_point_right,sl_point_left;
		Vec2d right_point ,left_point;
		sl_point_right.set_l(boundry.boundary()[i].first);
		sl_point_right.set_s(s);
		refLine.SLToXY(sl_point_right,&right_point);
		
		sl_point_left.set_l(boundry.boundary()[i].second);
		sl_point_left.set_s(s);
		refLine.SLToXY(sl_point_left,&left_point);
		geometry_msgs::Point ros_left_point,ros_right_point;
		ros_left_point.x = left_point.x();ros_left_point.y = left_point.y();
		ros_right_point.x = right_point.x();ros_right_point.y = right_point.y();
		left.push_back(ros_left_point);
		right.push_back(ros_right_point);
	}
	return;
}


void PlanningNode::addExtraPath(planning_msgs::TrajectoryPointArray &inPath)
{
    PlanningConfig *planning_config = PlanningConfig::get_instance();
    planning_msgs::TrajectoryPoint lastPoint = inPath.points.back();
    planning_msgs::TrajectoryPoint secondLastPoint = inPath.points[inPath.points.size()-2];
	
    Vec2d dir (lastPoint.x -secondLastPoint.x ,lastPoint.y - secondLastPoint.y);
	dir.Normalize();
	
	double resolution = planning_config->kPathBoundsDeciderResolution;
	for (size_t i = 0;i < planning_config->kNumExtraEndPoint;i++)
	{
	    planning_msgs::TrajectoryPoint point = inPath.points.back();
		point.x = point.x + resolution *dir.x();
		point.y = point.y + resolution *dir.y();
		point.s = point.s + resolution;
	    inPath.points.push_back(point);    
	}
}


//从后(from)往前(to)
bool PlanningNode::isOrdered(int from,int to)
{
    platoon_msgs::PlatoonMember fromMem = platoonMembers[from];
    platoon_msgs::PlatoonMember toMem = platoonMembers[to];
	
	double distance = amathutils::distance2D(fromMem.position,toMem.position);
	
	double angle = std::atan2(toMem.position.y - fromMem.position.y, toMem.position.x - fromMem.position.x);

	bool   direction= std::fabs(amathutils::normalizeRadian(angle - toMem.heading)) < (M_PI / 2.0);

	double limieDistance = fromMem.front_overhang_m + fromMem.wheel_base_m + toMem.rear_overhang_m + 3.0;

	if ((distance > limieDistance ) && direction)
		return true;

	return false;
}


//leadPos x y z,leaderHeading arc
bool  PlanningNode::platoonMassPoint()
{
    if (trajectoryType == PlatoonType::DISTRIBUTE)
		return true;
	
    geometry_msgs::Point massPoint;
    int indexSelf  = platoonGetNumIndex(vehicle_num_list,platformParam.num);
	double lengthToLeader = indexSelf * platoon_config.distance;
	double backHeading = amathutils::normalizeRadian(platoonMembers[vehicle_num_list[0]].heading+ M_PI);
    massPoint.x = platoonMembers[vehicle_num_list[0]].position.x +  std::cos( backHeading) *  lengthToLeader;
    massPoint.y = platoonMembers[vehicle_num_list[0]].position.y +  std::sin( backHeading) *  lengthToLeader;
	inputData.goalState.x = massPoint.x;
	inputData.goalState.y = massPoint.y;
	inputData.goalState.heading = platoonMembers[vehicle_num_list[0]].heading;
    openSpaceGoalSet = true;
	return true;
}



PLANNER_TYPE PlanningNode::plannerTypeDecision()
{	
    if (trajectoryType != inputData.refArray.type){
		std::cout <<"condition 1 "<<std::endl;
	    return NONE_PLANNER;
    }
	
    if (((trajectoryType == PlatoonType::DISTRIBUTE)|| (trajectoryType == PlatoonType::MASS))
		&& platoonMembers[platformParam.num].role == FOLLOWER){
		platoonMassPoint();
		return OPEN_SPACE;
	}
		
     //队形发生了改变.
    if (shape != inputData.refArray.shape && 
	    (amathutils::distanceToTrajectory(inputData.refArray.points,inputData.vehicleState)> 0.3 )) {
	    return REF_LINE;
    }
		
	shape = inputData.refArray.shape;
		
    
    if (leavingNum  && !isOrdered(leavingNum, platformParam.num))
	    return REF_LINE;
	
    if (leavingNum  && (amathutils::distanceToTrajectory(inputData.refArray.points,inputData.vehicleState)> 0.3 )){
	    return REF_LINE;
	}
    leavingNum = 0;

	if (joinSelf  && !isOrdered(platformParam.num,vehicle_num_list[vehicle_num_list.size()-2])){
	    std::cout <<"condition 2 "<<std::endl;
		return NONE_PLANNER;
	}
	
	if (joinSelf && (amathutils::distanceToTrajectory(inputData.refArray.points,inputData.vehicleState)> 0.3 ))
	    return REF_LINE;     	
		
	joinSelf = false;
	
	if (leaveSelf && leavePubFlag){
	    std::cout <<"condition 3 "<<std::endl;
		return NONE_PLANNER;
	}
		
    if (!inputData.refArray.is_forward_shift)
		return REVERSE;

	
    PlanningConfig *planning_config = PlanningConfig::get_instance();
	if ((!planning_config->open_path_planner))
	    return COMPLETE_REF_LINE;

	
	bool big_curve = false;
	int  nearIndex	=  amathutils::closestPoint(inputData.refArray.points,inputData.vehicleState.x,inputData.vehicleState.y);
	int  lowerIndex =  std::max(0,nearIndex - 10);
	int  upIndex	=  std::min(nearIndex+10,(int)inputData.refArray.points.size());
	for (size_t index = lowerIndex;index < upIndex;index++){
	    if (fabs(inputData.refArray.points[index].kappa) >= 0.1) {
			big_curve = true;
			break;
	    }
	}
    if (big_curve)
		return COMPLETE_REF_LINE;
	

	return REF_LINE;
	
}

void PlanningNode::vehicleStateInfo(VehicleState &state,int num) {

	state.x = platoonMembers[num].position.x;
	state.y = platoonMembers[num].position.y;
	state.heading = platoonMembers[num].heading;
	state.v = platoonMembers[num].linear_velocity;
	state.gear = platoonMembers[num].gear;
	return;
	
}

void  PlanningNode::platoonVelocityPlanner(planning_msgs::TrajectoryPointArray &trajectory)
{
    
	PlanningConfig *planning_config = PlanningConfig::get_instance();
	std::vector<ReferencePoint> refPoints;
	trajMsg2RefPoints(trajectory,refPoints);
	ReferenceLine   refenceLine(refPoints);
	auto selfIndex = platoonGetNumIndex(vehicle_num_list,platformParam.num);
    assert(selfIndex > 0);
	
    VehicleState leader_vehicle_state;
    VehicleState self_vehicle_state;
    VehicleState fronter_vehicle_state;
    int fronter_index = platoonFronterIndex((PlatoonType)trajectoryType,selfIndex);
	
	vehicleStateInfo(self_vehicle_state,platformParam.num);
	
	vehicleStateInfo(leader_vehicle_state,vehicle_num_list[0]);
    
	vehicleStateInfo(fronter_vehicle_state,vehicle_num_list[fronter_index]);
	
	TrajectoryPoint leaderPoint; // x y theta v
	TrajectoryPoint selfPoint;
	
	leaderPoint.path_point_.x_     = leader_vehicle_state.x;
	leaderPoint.path_point_.y_     = leader_vehicle_state.y;
	leaderPoint.path_point_.theta_ = leader_vehicle_state.heading;
	leaderPoint.v_                 = leader_vehicle_state.v;

	if (last_trajectory.size() <= 2){
		selfPoint.path_point_.x_      = self_vehicle_state.x;
		selfPoint.path_point_.y_      = self_vehicle_state.y;
		selfPoint.path_point_.theta_  = self_vehicle_state.heading;
		selfPoint.v_                  = self_vehicle_state.v;
	}
	else
	{
		size_t position_matched_index = last_trajectory.QueryNearestPointWithBuffer(
			{self_vehicle_state.x, self_vehicle_state.y}, 1.0e-6);
		
		selfPoint = last_trajectory.TrajectoryPointAt(position_matched_index);
	}

	planning_start_point_ = selfPoint;
	
	auto leader_sl_info = refenceLine.ToFrenetFrame(leaderPoint);

	auto self_sl_info = refenceLine.ToFrenetFrame(selfPoint);

	SLPoint fronter_sl_info ;
	
    refenceLine.XYToSL(Vec2d(fronter_vehicle_state.x,fronter_vehicle_state.y),&fronter_sl_info);
	double selfToFronter_s = fronter_sl_info.s() - self_sl_info.first[0];
	if (false == trajectory.is_forward_shift )
		selfToFronter_s = -selfToFronter_s;
	//assert(selfToFronter_s > 0);
	
	PidConf pid_config = planning_config->pid_config_;
	/*
	if (self_vehicle_state.v < 0.5){
		pid_config.kp_ = 1.5;
	    pid_controller.SetPID(pid_config);	
	}
	else
	{
	     pid_controller.SetPID(planning_config->pid_config_);
	}
    */
	pid_controller.SetPID(planning_config->pid_config_);
	double error_s = selfToFronter_s - platoon_config.distance;//位置误差
	double delta_v = pid_controller.Control(error_s, 0.1);
	double desired_v_on_reference = delta_v + leader_sl_info.first[1];
 	pid_controller.getPIDParam(pid_config);

	// log.desiredDistance = platoon_config.distance;
	// log.realDistance = selfToFronter_s;
	// log.errorDistance = error_s;
	// std::cout <<"pidconfig:    "<<pid_config.kp()<<"  "<<pid_config.ki()<<"  "<<pid_config.kd()<<std::endl;
	// std::cout <<"selfToFronter_s   "<<selfToFronter_s<<std::endl;
	// std::cout <<"platoon_config.distance   "<<platoon_config.distance<<std::endl;
	// std::cout <<"error_s  "<<error_s<<std::endl;
	// std::cout <<"leader_v   "<<leader_sl_info.first[1]<<std::endl;
	// std::cout <<"delta_v "<<delta_v<<std::endl;
	if (true == trajectory.is_forward_shift )
	    desired_v_on_reference = amathutils::Clamp(desired_v_on_reference,0.1,10.0);
	else
	    desired_v_on_reference = amathutils::Clamp(desired_v_on_reference,-10.0,0.0);

	
	//std::cout <<"desired_v_on_reference "<<desired_v_on_reference<<std::endl;
    if (fabs(desired_v_on_reference) < 0.6 )
	    desired_v_on_reference = 0.0;
	
	for (auto &point:trajectory.points){
		point.v = desired_v_on_reference;
		point.a = 0.0;
	    point.relative_time = point.s/point.v;
	}
	return ;
}


void PlanningNode::pubReplan(const Obstacle *obstacle){
	geometry_msgs::Point point;
	static geometry_msgs::Point blockingPoint;
    if (nullptr == obstacle){
		newReplan = true;
		return;
    }
	
    if ( fabs(current_velocity) > 1e-1)
		return;
	

    
    //规避重复发送，相当与如果阻断的是同一个位置，那么只发送一次。
	if (false == newReplan && (amathutils::distance2D(blockingPoint,obstacle->Perception().position) < 2.0))
		return;
	
	 blockingPoint.x = obstacle->Perception().position.x;
	 blockingPoint.y = obstacle->Perception().position.y;
     point.x = blockingPoint.x;
     point.y = blockingPoint.y;
	 
     //obsPoint.z = obstacle->Perception().position.z;
	 std::cout <<"pub_replan to globalPlanner"<<std::endl;
     newReplan = false;

	 
	 route_msgs::Replan obsPoint;
	 for (auto obs : obsList) {
		 geometry_msgs::Point point;
	 	 point.x = obs->Perception().position.x ;
	     point.y = obs->Perception().position.y ;
		 
		 obsPoint.points.push_back(point);
	 }
	 
	 pub_replan.publish(obsPoint);
    
	 std::vector< const Obstacle*>().swap(obsList);
	 std::vector< const Obstacle*>().swap(inputData.obsList);
}

void PlanningNode::velocityPlanning(planning_msgs::TrajectoryPointArray &trajectory)
{
	PlanningConfig *planning_config = PlanningConfig::get_instance();
	if (platoonBuild && platoonMembers[platformParam.num].role == FOLLOWER){
		if (leaveSelf){
			LeavingVelocityPlanner leavingVelocityPlanner;
			leavingVelocityPlanner.decelerateProfile(trajectory,inputData.vehicleState.v);
			leavePubFlag = true;
		}
		else
			platoonVelocityPlanner(trajectory);

	}
	else{
		if (planning_config->open_velocity_planner && trajectory.points.size())
			 velocityPlanner->process(current_chassis,obs,trajectory,trajectory,navUncertainty,	false);
	}
	return;

}
 
void PlanningNode::planning(PLANNER_TYPE planner_type)
{   	
	ros::Time time = ros::Time::now();

	if (NONE_PLANNER == planner_type)
		return;
	
	if (planner_type != last_planner_type){
	   std::vector<TrajectoryPoint>().swap(last_trajectory);
	}
	last_planner_type = planner_type;
	std::shared_ptr<BasePlanner> planner_ptr = planners[planner_type];

    planner_ptr->setInputData(inputData);
	
	bool ret = planner_ptr->implement(time.toSec(),last_trajectory);
	if (!ret)
		return;
    
	planning_msgs::TrajectoryPointArray trajectory;
	planner_ptr->getTrajectory(trajectory);
	pubReplan(planner_ptr->blockedObs());
	
	trajectory.is_forward_shift = inputData.refArray.is_forward_shift;
    trajectory.task_area = inputData.refArray.task_area;
    trajectory.type = inputData.refArray.type;	
	trajectory.header.stamp = time;
	if (trajectory.header.frame_id.empty())
		trajectory.header.frame_id = "map";
	velocityPlanning(trajectory);
	planning_msgs::TrajectoryPointArray candidate_trajectory;
	if (conflict_constraint_processor_.enabled()){
		// 先把基础速度规划后的候选轨迹给冲突判定节点，再用最近一次冲突消解决策修正本帧速度。
		candidate_trajectory = trajectory;
		trajectoryCandidatePub.publish(trajectory);
		conflict_constraint_processor_.apply(trajectory);
	}
	publishVelocityCurveMarker(candidate_trajectory, trajectory);
	publishActualVelocityCurveMarker();
	trajectoryPub.publish(trajectory);
    std::vector<TrajectoryPoint>().swap(last_trajectory);
	trajMsg2DiscretTraj(trajectory,last_trajectory);

	return ;
	
}


void PlanningNode::movingVitualObs()
{

    if (!pose_inited_)
		return ;
    if (virtualInfo.empty())
		return;
	PlanningConfig *planning_config = PlanningConfig::get_instance();
	if (!planning_config->virtual_moveing_obs) return;
    double dt  = 0.1;
	
    Eigen::MatrixXd cpos(8, 2), rotyaw(2, 2), cpos_shift(8, 2);
	double xrad = vehicle_util->vehicle_length_m/2.0;
	double yrad = vehicle_util->vehicle_width_m/2.0;
	double length = vehicle_util->vehicle_length_m;
	double width = vehicle_util->vehicle_width_m;
    
	std::vector< const Obstacle*>().swap(obsList);

	for (unsigned int i = 0; i< virtualInfo.size();i++)
    {
		VitualInfo &obs_info = virtualInfo[i];
		SLPoint slPoint;
		slPoint.set_s(obs_info.s + dt * (current_velocity-1.5) * obs_info.director);
	    slPoint.set_l(obs_info.l);

		if (slPoint.s() < 0.1 || slPoint.s() > obs_info.refLine.Length())
			continue;

		obs_info.s = slPoint.s();
		
        double centerGlobal_x,centerGlobal_y;
		Vec2d xy;
		obs_info.refLine.SLToXY(slPoint, &xy);
		centerGlobal_x = xy.x();
		centerGlobal_y = xy.y();

		double yaw  =obs_info.refLine.GetNearestReferencePoint(slPoint.s()).heading() ;
		if (obs_info.director < 0){
			yaw = amathutils::normalizeRadian(yaw + M_PI);	
		}

		
	    rotyaw << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);
		
	    cpos << -xrad, -yrad, -xrad, 0, -xrad, yrad, 0, yrad, xrad, yrad, xrad, 0,
	        xrad, -yrad, 0, -yrad;
	  
	    cpos_shift << centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y
	    	,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y,centerGlobal_x, centerGlobal_y;
	    cpos = cpos * rotyaw + cpos_shift; 
		
		struct ugv::perception::PerceptionObstacle perception_obstacle;
	    for (int j = 0; j < cpos.rows(); j++) {   
	        perception_obstacle.polygon_point.push_back({cpos(j, 0),cpos(j, 1),0.0});
	    }
		
	    perception_obstacle.position.x = centerGlobal_x;
	    perception_obstacle.position.y = centerGlobal_y;
	    perception_obstacle.theta = yaw;
	    perception_obstacle.length = length;
	    perception_obstacle.width = width;
        perception_obstacle.id = i;


	    auto obstaclePtr = new Obstacle(to_string(i),perception_obstacle,true);
	    obsList.push_back(obstaclePtr);	
    }
	
	collectDiplayObsInfo(obsList);
    is_obs_update = true;
}


void PlanningNode::sendHeart(unsigned char flag)
{
    static int heart_count = 0;
	heart_count++;
	
    static unsigned char  send_count = 0;
	if (0 == heart_count%10 )
	{   
	    heart_count = 0;
	    heartbeat_msgs::Heartbeat beat;
		beat.flag = flag;
		beat.heart_beat = send_count;
		pub_heart.publish(beat);
	    send_count++;
	}
}



int main( int argc, char** argv )
{
	ros::init(argc, argv, "planning");
	ros::NodeHandle n;
	PlanningNode planningNode(n);
	ros::spin();
	ROS_INFO(" planning The iteration end.");
	return 0;
}
