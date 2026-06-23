
#include "reference_node.h"
#include "formatting_reference.h"
#include <unistd.h>
#include "logger/logger.h"


ReferenceNode::ReferenceNode(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{	
    std::string fileName;
	private_nh_.param<std::string>("file_name", fileName, "gpsData.txt");
	private_nh_.param<double>("forward_s", param.forward_s, 50.0);
	private_nh_.param<double>("backward_s", param.backward_s, 10.0);
	private_nh_.param<double>("normal_area_forward_s", param.normal_area_forward_s, 50.0);
	private_nh_.param<double>("others_area_forward_s", param.others_area_forward_s, 80.0);
	private_nh_.param<std::string>("task_area", param.task_area, "normal_area");

    private_nh_.param<bool>("direct_to_control", param.direct_to_control, false);
    private_nh_.param<bool>("is_forward_shift", param.is_forward_shift, true);
    private_nh_.param<bool>("reverse_path", param.reverse_path, false);


	geometry_msgs::Point origin;
    private_nh_.param<double>("latitude", origin.x , 0.0);
	private_nh_.param<double>("longitude", origin.y , 0.0);
	private_nh_.param<double>("altitude", origin.z , 0.0);
	
    projector = projection::UtmProjector(origin);

	vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();
	vehcileInfo->loadVehicleingParam(private_nh_);

	// ros subscribers
	sub_current_pose = nh_.subscribe("odomData", 10, &ReferenceNode::currentPoseCallback, this);
    
	chassisSub = nh_.subscribe("chassis", 10, &ReferenceNode::callbackChassis, this); 
	pathSub = nh_.subscribe("trajectory_point_array", 10, &ReferenceNode::callbackRemotePath, this); 

	route_sub  = nh_.subscribe("/route_result", 10, &ReferenceNode::callbackRemotePath, this); 

    sub_shift_forward = nh_.subscribe("shift_forward", 1, &ReferenceNode::callbackReverse, this); 
	
    platoonMember_sub_ = nh.subscribe("/PlatoonMember", 10, &ReferenceNode::callbackPlatoonMember, this); 
	platoonMission_sub_ = nh.subscribe("/PlatoonMission", 10, &ReferenceNode::callbackPlatoonMission, this); 
	platoonConfig_sub_ = nh.subscribe("/PlatoonConfig", 10, &ReferenceNode::callbackPlatoonConfig, this); 

	platoonMember_self_sub_  = nh.subscribe("PlatoonMember_self", 10, &ReferenceNode::callbackPlatoonMember, this);
    platoonConfig_self_sub_  = nh.subscribe("PlatoonMission_self", 10, &ReferenceNode::callbackPlatoonMission, this);
    platoonMission_self_sub_ = nh.subscribe("PlatoonConfig_self", 10, &ReferenceNode::callbackPlatoonConfig, this);

	
    std::string topic_trajectory = param.direct_to_control?std::string("trajectory"):std::string("referenceLine");
	pub_reference  = nh_.advertise<planning_msgs::TrajectoryPointArray>(topic_trajectory, 1);
	pub_diplay = nh_.advertise<visualization_msgs::MarkerArray>(
		"reference_vehcile_route_marker", 1);
	pub_heart = nh_.advertise<heartbeat_msgs::Heartbeat>("heartbeat", 1);
	timer = nh_.createTimer(ros::Duration(0.1),&ReferenceNode::callbackTimerReference,this);
    smoother_init();
    
	sleep(5);//用于仿真
	/**
	std::vector<geometry_msgs::Point> trajectory;
	std::string recode_data_dir  = ros::package::getPath("launch_node");
	param.pathFile = recode_data_dir + std::string("/data/")+fileName;
	readGpsTxts(param.pathFile,trajectory);
	deleteExceptionPoints(trajectory);
	std::cout <<"trajectory size "<<trajectory.size()<<std::endl;
    amathutils::resamplePoints(0.5,trajectory);
	std::cout <<"trajectory size 2 : "<<trajectory.size()<<std::endl;

	
	setTrajectory(trajectory);
	draw_route(trajectory);
	**/
	usleep(1);
	lastClosestIndex = 0;
	std::string vehicle_platform_file;
	private_nh_.param<std::string>("vehicle_platform_file", vehicle_platform_file, "vehicle_platform.yaml");
    common::getPlatformParam(vehicle_platform_file,platformParam);
	shape = PlatoonType::COLUMN;

	optsFunc.push_back(&ReferenceNode::none);
	optsFunc.push_back(&ReferenceNode::build);
	optsFunc.push_back(&ReferenceNode::join);
	optsFunc.push_back(&ReferenceNode::leave);
	optsFunc.push_back(&ReferenceNode::disolve);
	optsFunc.push_back(&ReferenceNode::column);
	optsFunc.push_back(&ReferenceNode::diamond);
	optsFunc.push_back(&ReferenceNode::triangle);
	optsFunc.push_back(&ReferenceNode::reverse);
	optsFunc.push_back(&ReferenceNode::cancel_reverse);
	optsFunc.push_back(&ReferenceNode::mass);
	optsFunc.push_back(&ReferenceNode::distribute);
	optsFunc.push_back(&ReferenceNode::running);

    std::string param_node_dir  = ros::package::getPath("launch_node");
	std::string log_param_file = param_node_dir + std::string("/param/log/")+std::string("log.yaml");

	//自身的pkg名称，对应log.yaml中的名称
	std::string pkg_name = std::string("reference_line_node"); 
	LogCfg logCfg = Logger::logCfg(log_param_file,pkg_name);
    Logger *logger = Logger::get_instance();
	logger->init(logCfg);
}


void  ReferenceNode::none(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    return;
}

void  ReferenceNode::updatePathToLeader(){

	lastClosestIndex = 0;
	std::vector<geometry_msgs::Point>().swap(leader_route);
	leader_route_length = 0.0;
	if (platformParam.num != vehicle_num_list[0]){//非头车
		addExtraPath(leader_route);
	    leader_route_length	= amathutils::trajectoryLength(leader_route);
		std::cout <<"build leader_route size  "<<leader_route.size()<<std::endl;
	}
}


void  ReferenceNode::build(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    std::cout <<"build start"<<std::endl;
	platoon_config = msg->config;
	platoonBuild = true;
	vehicle_num_list = msg->vehicle_list;
	shape = PlatoonType::COLUMN;
    std::cout <<"build end"<<std::endl;
	return;
}


void  ReferenceNode::join(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    if(!platoonCheckNum(vehicle_num_list,msg->num)){
	    vehicle_num_list.push_back(msg->num);
		if (msg->num == platformParam.num)
			lastClosestIndex = 0;
		    shape = PlatoonType::COLUMN;
    }
}


void  ReferenceNode::leave(const platoon_msgs::PlatoonMission::ConstPtr &msg){
    platoonEraseNum(vehicle_num_list,msg->num);
}


void  ReferenceNode::disolve(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	platoonBuild = false;
	lastClosestIndex = 0;
	std::vector<unsigned char>().swap(vehicle_num_list);
	shape = PlatoonType::COLUMN;
}


void  ReferenceNode::column(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	uint8  optType = msg->command_type;
	shape = optType;
	return ;
}


void  ReferenceNode::diamond(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	uint8  optType = msg->command_type;
	shape = optType;
    return ;
}


void  ReferenceNode::triangle(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	uint8  optType = msg->command_type;
	shape = optType;
    return ;
}


void  ReferenceNode::reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	uint8  optType = msg->command_type;
	if (shape != PlatoonType::COLUMN){
		std::cout <<"warning: current shape is not  COLUMN "<<std::endl;
		return;
	}
	
	param.is_forward_shift = false;
    return ;
}


void  ReferenceNode::cancel_reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg){

	uint8  optType = msg->command_type;
	std::vector<geometry_msgs::Point>().swap(leader_route);
	leader_route_length = 0.0;
    
	if (platformParam.num != vehicle_num_list[0]){//非头车
		addExtraPath(leader_route);
	    leader_route_length	= amathutils::trajectoryLength(leader_route);
		std::cout <<" cancel_reverse build leader_route size:   "<<leader_route.size()<<std::endl;
	}
	shape = PlatoonType::COLUMN;
	param.is_forward_shift = true;
    return ;
}


void  ReferenceNode::mass(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	uint8  optType = msg->command_type;
	//shape = optType;
	std::vector<geometry_msgs::Point>().swap(leader_route);
	leader_route_length = 0.0;
	lastClosestIndex = 0;
	
    return ;
}


void  ReferenceNode::distribute(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	uint8  optType = msg->command_type;
	//shape = optType;
	std::vector<geometry_msgs::Point>().swap(leader_route);
	leader_route_length = 0.0;
	lastClosestIndex = 0;
    return ;
}

void  ReferenceNode::running(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	uint8  optType = msg->command_type;
	updatePathToLeader();
    return ;
}

void ReferenceNode::callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg) {

    /*
    bool lead_gear_changed = false;
    if ((msg->role == PlatoonRole::LEADER)&& platoonBuild) {
       if ((7 == msg->gear )  && (7 != platoonMembers[msg->num].gear)) //切换到倒档位
           lead_gear_changed = true;
    }
	*/
	
	platoonMembers[msg->num] = *msg;

	//std::cout <<"member "<<(int)msg->num <<std::endl;
	//std::cout <<"role  "<<(int)msg->role <<std::endl;
    if (!platoonBuild)
		return;
	
    if (msg->role != PlatoonRole::LEADER)
		return;

	if (platformParam.num == vehicle_num_list[0])
		return;
	
	//if (lead_gear_changed)
		
	geometry_msgs::Point current_pos = msg->position;
    double moveDistance = 0.0;
	if (leader_route.empty())
	{    
	    leader_route.push_back(current_pos);
		return;
	}
    
	geometry_msgs::Point lastPoint = leader_route.back();
    moveDistance = amathutils::distance2D(lastPoint, current_pos);

	if (moveDistance > 0.5 && msg->gear != 7){
	    leader_route_length += moveDistance;
	    leader_route.push_back(current_pos);
	}
	
	while (leader_route_length >500.0){
	    leader_route_length -= amathutils::distance2D(leader_route[0], leader_route[1]);
		leader_route.erase(leader_route.begin());
	}
	
	visualization_msgs::MarkerArray markerArray;
	DisplayConfig cfg;
	cfg.id = 1;
	cfg.r = 1.0;
	cfg.scale_x  = 0.02;
	cfg.ns = std::string("/reference/leader_route");
	markerArray.markers.push_back(DisPlay::lineMarker(leader_route,cfg));
	pub_diplay.publish(markerArray);
	
	return;
}



//
void ReferenceNode::callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg) {

	uint8  optType = msg->command_type;
	assert(optType >= PlatoonType::BUILD);
	assert(optType <= PlatoonType::RUNNING);
	
    if ((optType != PlatoonType::BUILD) && (!platoonBuild))
		return;
    
    if ((optType == PlatoonType::BUILD) && (platoonBuild))
		return;	
	
    trajectoryType = optType;
	
	std::vector<planning_msgs::TrajectoryPoint>().swap(lastRefLineRaw.points);
	std::cout <<"cmd type  "<<(int)msg->command_type<<std::endl;
	(this->*optsFunc[msg->command_type])(msg);
	
	return;
}


//
void ReferenceNode::callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg) {

    platoon_config = *msg;
	
	return;
}


void ReferenceNode::addExtraPath( std::vector<geometry_msgs::Point> &inPath)
{
    int leaderNum = vehicle_num_list[0];
	double dir_x = platoonMembers[leaderNum].position.x - platoonMembers[platformParam.num].position.x;
	double dir_y = platoonMembers[leaderNum].position.y - platoonMembers[platformParam.num].position.y;
    double distance = std::hypot(dir_x, dir_y);
	dir_x /= distance;
	dir_y /= distance;
	double resolution = 0.5 ;
	int count  = distance/0.5;

	geometry_msgs::Point point0;
	point0.x = platoonMembers[platformParam.num].position.x;
	point0.y = platoonMembers[platformParam.num].position.y;
	inPath.push_back(point0);
	for (size_t i = 0;i < count;i++)
	{
	    geometry_msgs::Point point = inPath.back();
		point.x = point.x + resolution *dir_x;
		point.y = point.y + resolution *dir_y;
	    inPath.push_back(point);    
	}
	return ;
}


void ReferenceNode::callbackReverse(const std_msgs::Int32::ConstPtr &msg){
     std::cout <<"callbackReverse "<<msg->data <<std::endl;
	 if (msg->data)
	     param.is_forward_shift = false;
	 else	
	     param.is_forward_shift = true;
	 
	 std::vector<planning_msgs::TrajectoryPoint>().swap(lastRefLineRaw.points);
}




void ReferenceNode::clickPointCallBack(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {

    geometry_msgs::Pose pose = msg->pose.pose;
    state.position.x = pose.position.x;
    state.position.y = pose.position.y;
    state.yaw = amathutils::getPoseYawAngle(pose);
	odom.pose = msg->pose;
	pose_inited = true;
	return;
}


void ReferenceNode::currentPoseCallback(const localization_msgs::Localization::ConstPtr &msg) {
    
    state.position.x = msg->location.pose.pose.position.x;
    state.position.y = msg->location.pose.pose.position.y;
	
    state.yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2);

	odom = amathutils::getOdometryFromPosAndYaw(msg->location.pose.pose.position,state.yaw);
    //std::cout <<"yaw "<<state.yaw / 3.1415 * 180 <<std::endl;
    geometry_msgs::Point current_pos;
	current_pos.x = state.position.x;
	current_pos.y = state.position.y;

    double moveDistance = 0.0;
	if (!real_route.empty())
	{
	    geometry_msgs::Point lastPoint = real_route.back();
		moveDistance = amathutils::distance2D(lastPoint, current_pos);
	}
	else
	    real_route.push_back(current_pos);

	visualization_msgs::MarkerArray markerArray;
	
    if (moveDistance > 0.5 ){
	    real_route.push_back(current_pos);
		DisplayConfig cfg;
		cfg.id = 1;
		cfg.r = 0.5;
		cfg.b = 0.5;
		cfg.scale_x  = 0.02;
		cfg.ns = std::string("/reference/real_state");
		markerArray.markers.push_back(DisPlay::lineMarker(real_route,cfg));
    }
     
	static ros::Time last_vehicle_marker_pub;
	const ros::Time now = ros::Time::now();
	const bool publish_vehicle_marker = !pose_inited ||
		(moveDistance > 0.1) ||
		last_vehicle_marker_pub.isZero() ||
		((now - last_vehicle_marker_pub).toSec() > 0.2);

	if (publish_vehicle_marker){
		last_vehicle_marker_pub = now;

		DisplayConfig config;
		config.id = 2;
		config.ns = std::string("/reference/real_state");
		visualization_msgs::MarkerArray v_markArray = DisPlay::vehicleMarkerArray(odom, vehcileInfo, config);

		markerArray.markers.insert(markerArray.markers.end(),v_markArray.markers.begin(),v_markArray.markers.end());

		std::ostringstream oss;
		oss << std::setprecision(4) << state.vel * 3.6;
		std::string vel_str = oss.str() + "km/h";

		geometry_msgs::Point strLocalPos;
		strLocalPos.x = (vehcileInfo->vehicle_length_m - vehcileInfo->rear_overhang_m) * 0.5;
		strLocalPos.y = -vehcileInfo->vehicle_width_m * 0.5 - 3.0 ;strLocalPos.z = 0;
		double yaw = amathutils::getPoseYawAngle(odom.pose.pose);
		geometry_msgs::Point globalPoint = amathutils::localToGlobal(odom.pose.pose.position,yaw,strLocalPos);

		config.id = 6;
		config.r  = 1.0;
		config.g  = 1.0;
		config.scale_z = 1.5;
		config.position = globalPoint;
		markerArray.markers.push_back(DisPlay::stringMarker(vel_str,config));
	}

    pub_diplay.publish(markerArray);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(state.position.x , state.position.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, state.yaw);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "vehcile"));

	pose_inited = true;
	return;
}
	

void ReferenceNode::setTrajectory(std::vector<geometry_msgs::Point> &trajectory_)
{
     global_route  = trajectory_;
}

void ReferenceNode::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}


void ReferenceNode::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}


bool ReferenceNode::tooBigCurlture(double x,double y,planning_msgs::TrajectoryPointArray &refTrajectory){

	int  nearIndex = amathutils::closestPoint(refTrajectory.points,state.position.x,state.position.y);
	int  lowerIndex =  std::max(0,nearIndex - 15);
	int  upIndex	=  std::min(nearIndex+15,(int)refTrajectory.points.size());
	for (size_t index = lowerIndex;index < upIndex;index++){
		if (fabs(refTrajectory.points[index].kappa )>= 0.5) {
			std::cout <<"too big culture"<<std::endl;
			return true;
		}
	}
	return false;
}


bool ReferenceNode::smoothFem(planning_msgs::TrajectoryPointArray &refTrajectory,int lIndex,int rIndex)
{
    if ( refTrajectory.points.size() < 3)
		return true;
    auto fem_smoother = std::make_shared<FemPosDeviationSmoother>(smoother_config);
	std::vector<std::pair<double, double>> raw_point2d;
    std::vector<double> points_x,points_y,bounds;
	bounds.resize(refTrajectory.points.size());

	double smooth_lat_bound = smoother_config.lat_bound;
	if (tooBigCurlture(state.position.x,state.position.y,refTrajectory)){
	    smooth_lat_bound = 1.5 * smoother_config.lat_bound;	
	}
	
	for (int i = 0; i < refTrajectory.points.size(); i++) {

		bounds[i] = 0.0;
		if (i >= lIndex && i <= rIndex )
	        bounds[i] = smooth_lat_bound;
	    raw_point2d.emplace_back(refTrajectory.points[i].x,refTrajectory.points[i].y);
	}
	NormalizePoints(&raw_point2d);
	bounds.front() = 1e-6;
	bounds.back() = 1e-6;
	
	std::vector<double> box_bounds = bounds;
    const double box_ratio = 1.0 / std::sqrt(2.0);
    for (auto& bound : box_bounds) {
        bound *= box_ratio;
    }
    if(!fem_smoother->Solve(raw_point2d,box_bounds,&points_x,&points_y)){
		std::cout <<"smoother not solve"<<std::endl;
        return false;
    }
	//std::cout <<""<<std::endl;
	refTrajectory.points.resize(points_x.size());
	for (unsigned int i = 0 ; i < points_x.size();i++){
	    refTrajectory.points[i].x =points_x[i] +zero_x_;	
	    refTrajectory.points[i].y =points_y[i] +zero_y_;	
	}
	return true;
}


common::ErrorCode ReferenceNode::readGpsTxts(std::string &fileName, std::vector<geometry_msgs::Point> &trajectoryTotal)
{

    std::ifstream filename(fileName);
    if (!filename)
    {
        LOG_ERROR("open trajectory file error {} ",fileName);
        return common::REF_OPEN_TRAJECTORY_ERR;
    }
	
    std::string oneLine;
	getline(filename,oneLine);
	int first = 0;
    while(getline(filename,oneLine))
    {
		geometry_msgs::Point p;
        std::istringstream streamOneLine(oneLine);
        std::string ignore;		
        streamOneLine >>ignore;
        streamOneLine >>ignore;
        streamOneLine >>ignore;
        streamOneLine >>p.x;
        streamOneLine >>p.y;
		p = projector.forward(p);
        trajectoryTotal.push_back(p);
    }
	
	if (param.reverse_path)
        std::reverse(trajectoryTotal.begin(),trajectoryTotal.end());
	
    filename.close();
    return common::NO_ERROR;
}


void ReferenceNode::callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg)
{
	state.vel = (7 == msg->gear_location)?(-msg->current_velocity):msg->current_velocity;
    
}
void  ReferenceNode::trajMsg2GeometryMsg  (const planning_msgs::TrajectoryPointArray &trajectory,std::vector<geometry_msgs::Point> &outPoints)
{
	std::vector<geometry_msgs::Point>().swap(outPoints);
	
	for (const auto &point : trajectory.points){
		geometry_msgs::Point p;
		p.x = point.x;
	    p.y = point.y;
		outPoints.push_back(p);
	}
	return ;
}


void ReferenceNode::callbackRemotePath(const planning_msgs::TrajectoryPointArray::ConstPtr &msg)
{	
    std::cout <<"callbackRemotePath from global planner "<<msg->points.size()<<std::endl;
	if (msg->points.size() < 2){
	    std::cout <<"too less points,points size  "<<(int)msg->points.size()<<std::endl;
		return ;
	}
	
    planning_msgs::TrajectoryPointArray trajectory = *msg; 
	amathutils::printTraj(trajectory.points);
	const auto & p0 = trajectory.points.at(0);
	const auto & p1 = trajectory.points.at(1);
	double angle = std::atan2(p1.y - p0.y,p1.x - p0.x);
    angle = amathutils::normalizeRadian(angle);
	param.is_forward_shift = true;
	if (std::fabs(angle - state.yaw) > 1.570796) {
	    std::cout <<"------------receive backing up path --------"<<std::endl;
		std::reverse(trajectory.points.begin(),trajectory.points.end());
		param.is_forward_shift = false;	
	}
		
	amathutils::resamplePoints(0.5,trajectory.points);

	std::cout << " resample after "<<trajectory.points.size()<<std::endl;
	//deleteExceptionPoints(trajectory.points);
	//std::cout << " deleteExceptionPoints  "<<trajectory.points.size()<<std::endl;

	std::vector<geometry_msgs::Point> remotePath;
	trajMsg2GeometryMsg(trajectory,remotePath);
	setTrajectory(remotePath);
	std::vector<planning_msgs::TrajectoryPoint>().swap(lastRefLineRaw.points);

	lastClosestIndex = 0;
	draw_route(remotePath);
}


void ReferenceNode::sendHeart(unsigned char flag)
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


bool  ReferenceNode::generateReferenceLine(const std::vector<geometry_msgs::Point> &route,planning_msgs::TrajectoryPointArray &referenceLine)
{
    if (route.empty())
		return false;
	
	int closestIndex = amathutils::getClosestIndex(lastClosestIndex,route,state.position);
	if (0 == lastClosestIndex)
		std::cout <<"closestIndex  "<<closestIndex<<std::endl;
	lastClosestIndex =	closestIndex;
	
	if ((closestIndex == route.size()-1) && param.is_forward_shift)
		return false;

	if ((closestIndex == 0 ) && false == param.is_forward_shift)
		return false;
	param.is_forward_shift?forward_traj(closestIndex,route,referenceLine.points,param.forward_s,param.backward_s):
	backward_traj(closestIndex,route,referenceLine.points,param.forward_s,param.backward_s);
	
	referenceLine.is_forward_shift = param.is_forward_shift;
	referenceLine.task_area = param.task_area;

	return true;
	
}


int ReferenceNode::forwardAppendPath(const planning_msgs::TrajectoryPointArray &preTraj,const planning_msgs::TrajectoryPointArray &curTraj,
planning_msgs::TrajectoryPointArray &addpendPath,double refAppendLen)
{
    if (preTraj.points.empty()){
		std::cout <<"preTraj empty -----------"<<std::endl;
		return 0.0;
    }
	
	planning_msgs::TrajectoryPoint preBackPoint;
	preBackPoint = preTraj.points.back();
	int closestIndex = -1;

	for (int i = curTraj.points.size()-1; i >= 0; i--){
		
		if(amathutils::distance2D(preBackPoint,curTraj.points[i]) < 1e-3){
		   closestIndex = i;
		   break;
		}
	}
	if (-1 == closestIndex){
		
     	std::cout <<"pr----last ---------- -----------  "<<lastClosestIndex <<std::endl;
		amathutils::printTraj(preTraj.points);
		std::cout <<"pr-------------------- -----------"<<std::endl;
			amathutils::printTraj(curTraj.points);
	}
	double appendLength = amathutils::trajectoryLength(curTraj.points,closestIndex,curTraj.points.size()-1);
	if (appendLength < refAppendLen)
	    return 0.0;
	
	addpendPath.points.insert(addpendPath.points.end(),curTraj.points.begin()+closestIndex+1,curTraj.points.end());

	return appendLength;
}

double ReferenceNode::backwardAppendPath(const planning_msgs::TrajectoryPointArray &preTraj,const planning_msgs::TrajectoryPointArray &curTraj,
planning_msgs::TrajectoryPointArray &addpendPath,double refAppendLen)
{
    if (preTraj.points.empty())
		return 0.0;

	planning_msgs::TrajectoryPoint preFrontPoint;
	preFrontPoint = preTraj.points.front();
	int closestIndex = -1;
	for (unsigned int i = 0; i < curTraj.points.size(); i++){
		
		if(amathutils::distance2D(preFrontPoint,curTraj.points[i]) < 1e-3){
		   closestIndex = i;
		   break;
		}
	}
	if (-1 == closestIndex){
		return 0.0;
	}
	double appendLength = amathutils::trajectoryLength(curTraj.points,0,closestIndex);
	if (appendLength < refAppendLen)
	    return 0.0;
	
	addpendPath.points.insert(addpendPath.points.end(),curTraj.points.begin(),curTraj.points.begin()+closestIndex);

	return appendLength;
}



void ReferenceNode::callbackTimerReference(const ros::TimerEvent &event)
{   
	sendHeart(0);
	if (!pose_inited)
	   return;
	planning_msgs::TrajectoryPointArray referenceLine;
    std::vector<geometry_msgs::Point> route;

	if (platoonBuild && (platformParam.num != vehicle_num_list[0]))
        route = leader_route; 
	else
		route = global_route;
	
	if (!generateReferenceLine(route,referenceLine))
        return;
    
	planning_msgs::TrajectoryPointArray tmpReferenceLine = referenceLine;
	planning_msgs::TrajectoryPointArray addpendPath;
	planning_msgs::TrajectoryPointArray lastRefernce;
    double refAppendLen = 3.0;
    
	/*
	double lenToEnd = param.is_forward_shift?amathutils::distance2D(route.back(),referenceLine.points.back()):amathutils::distance2D(route.front(),referenceLine.points.front());
	if (lenToEnd < 1e-3){
		//std::cout <<"got to end "<<std::endl;
	    refAppendLen = 1e-3;
	}
	double appendLength = param.is_forward_shift?forwardAppendPath(lastRefLineRaw,referenceLine,addpendPath,refAppendLen):backwardAppendPath(lastRefLineRaw,referenceLine,addpendPath,refAppendLen);
    if (appendLength < 1e-1 && (!lastRefLineRaw.points.empty())){
		referenceLine = lastRefLine;
		referenceLine.shape = shape;
		referenceLine.type = trajectoryType;
		pub_reference.publish(referenceLine);
		return;
    }
	*/
	
    int lIndex = 0,rIndex = referenceLine.points.size()-1;
	if (!addpendPath.points.empty()){
		referenceLine = lastSmoothedLine;
		auto insertPos = param.is_forward_shift?referenceLine.points.end():referenceLine.points.begin();
		referenceLine.points.insert(insertPos,addpendPath.points.begin(),addpendPath.points.end());
		int closestIndex = amathutils::getClosestIndex(0,referenceLine.points,state.position);
		planning_msgs::TrajectoryPointArray referenceLineFinal;

		param.is_forward_shift?forward_traj(closestIndex,referenceLine.points,referenceLineFinal.points,param.forward_s,param.backward_s):
		backward_traj(closestIndex,referenceLine.points,referenceLineFinal.points,param.forward_s,param.backward_s);

		referenceLine = referenceLineFinal;
				
		lIndex = param.is_forward_shift?(amathutils::closestPoint(referenceLine.points,addpendPath.points.front())-10):0;

		rIndex = param.is_forward_shift?referenceLine.points.size():(amathutils::closestPoint(referenceLine.points,addpendPath.points.back())+10);
		
	}
	
	lastRefLineRaw = tmpReferenceLine;

    amathutils::caculateKappa(1,referenceLine.points);
 	if (!smoothFem(referenceLine,lIndex,rIndex) && lastRefLine.points.size()){
	     referenceLine = lastRefLine;
		 std::cout <<"smoothFem failture "<<std::endl;
	}

    lastSmoothedLine = referenceLine;
	if (platoonBuild && (platformParam.num != vehicle_num_list[0]) && 
		(shape == PlatoonType::DIAMOND || shape == PlatoonType::TRIANGLE )){
	    double offset = platoonGetLoffset(vehicle_num_list,platformParam.num,platoon_config.lateral_offset);
		if (fabs(offset )> 1e-3){
			amathutils::caculateHeading(referenceLine.points);
			amathutils::caculateAccumulated_s(referenceLine.points);
			FormattingRefLine formatRef(referenceLine);
			formatRef.implement(offset,referenceLine);
		}
	}

    amathutils::caculateKappa(1,referenceLine.points);
	amathutils::caculateHeading(referenceLine.points);
	amathutils::caculateAccumulated_s(referenceLine.points);
	amathutils::caculateDkappa(referenceLine.points);
	referenceLine.is_forward_shift = param.is_forward_shift;
	referenceLine.task_area = param.task_area;
    referenceLine.shape = shape;
    referenceLine.type = trajectoryType;
	//循迹的时候，只用来做横向控制的时候才用set_vel
	set_vel(referenceLine);
	lastRefLine = referenceLine;

	pub_reference.publish(referenceLine);
    
	visualization_msgs::MarkerArray markerArray;
	if (referenceLine.points.size() > 0){
	    DisplayConfig cfg;
		cfg.id = 1;
		cfg.b = 1.0;
		cfg.scale_x = 0.02;
		cfg.ns = std::string("/reference/referenceLine");
	    markerArray.markers.push_back(DisPlay::lineMarker(referenceLine.points,cfg));
	}
	pub_diplay.publish(markerArray);
}



void ReferenceNode::smoother_init()
{
	private_nh_.param<double>("weight_fem_pos_deviation",smoother_config.weight_fem_pos_deviation ,1.0e10);
	private_nh_.param<double>("weight_ref_deviation",smoother_config.weight_ref_deviation , 1.0);
	private_nh_.param<double>("weight_path_length",smoother_config.weight_path_length , 1.0);
	private_nh_.param<bool>("apply_curvature_constraint",smoother_config.apply_curvature_constraint , false);
	private_nh_.param<double>("weight_curvature_constraint_slack_var",smoother_config.weight_curvature_constraint_slack_var , 1e2);

	private_nh_.param<double>("curvature_constraint",smoother_config.curvature_constraint , 0.2);
	private_nh_.param<bool>("use_sqp",smoother_config.use_sqp , false);
	private_nh_.param<double>("sqp_ftol",smoother_config.sqp_ftol ,  1e-4);
	private_nh_.param<double>("sqp_ctol",smoother_config.sqp_ctol ,  1e-3);
	private_nh_.param<int>("sqp_pen_max_iter",smoother_config.sqp_pen_max_iter , 10);	

	private_nh_.param<int>("sqp_sub_max_iter",smoother_config.sqp_sub_max_iter , 100);
	private_nh_.param<int>("max_iter",smoother_config.max_iter , 500);
	private_nh_.param<double>("time_limit",smoother_config.time_limit , 0.0);
	private_nh_.param<bool>("verbose",smoother_config.verbose , false);
	private_nh_.param<bool>("scaled_termination",smoother_config.scaled_termination , true);
	private_nh_.param<bool>("warm_start",smoother_config.warm_start , true);	

	private_nh_.param<int>("print_level",smoother_config.print_level , 0);
	private_nh_.param<int>("max_num_of_iterations",smoother_config.max_num_of_iterations , 500);
	private_nh_.param<int>("acceptable_num_of_iterations",smoother_config.acceptable_num_of_iterations , 15);
	private_nh_.param<double>("tol",smoother_config.tol , 1e-8);
	private_nh_.param<double>("acceptable_tol",smoother_config.acceptable_tol , 1e-1);
	private_nh_.param<double>("lat_bound",smoother_config.lat_bound , 0.25);   
}


void ReferenceNode::draw_route( std::vector<geometry_msgs::Point>  &globalRoute)
{
	visualization_msgs::MarkerArray markerArray;
	DisplayConfig cfg;
	cfg.id = 1;
	cfg.r = 1.0;
	cfg.g = 1.0;
	cfg.b = 1.0;
	cfg.scale_x = 0.02;
	cfg.ns = std::string("/reference/global_route");
	markerArray.markers.push_back(DisPlay::lineMarker(globalRoute,cfg));
	pub_diplay.publish(markerArray);
}


void ReferenceNode::set_vel( planning_msgs::TrajectoryPointArray &trajectory)
{
	for (size_t i = 0; i < trajectory.points.size(); i++) {
	    trajectory.points[i].v = state.vel;
	}
}


int main(int argc ,char *argv[])
{        
	ros::init(argc, argv, "referenceLine");
	ros::NodeHandle nh;
	ROS_INFO("referenceLine start.");

	ReferenceNode referenceNode(nh);
	ros::spin();
	ROS_INFO(" ReferenceNode The iteration end.");
	
	return 0;
}


