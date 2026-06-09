
#include "cooperation.h"
#include <unistd.h>

#include <pwd.h>



Cooperation::Cooperation(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{	
	vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();
	vehcileInfo->loadVehicleingParam(private_nh_);

	std::string vehicle_platform_file;
	private_nh_.param<std::string>("vehicle_platform_file", vehicle_platform_file, "vehicle_platform.yaml");
    common::getPlatformParam(vehicle_platform_file,platformParam);
	std::cout <<platformParam.vehicle_type<<std::endl;
    std::cout <<platformParam.ins_type<<std::endl;
    std::cout <<platformParam.id<<std::endl;
    std::cout <<platformParam.num<<std::endl;

	
	private_nh_.param<int>("policy", param.policy, 1);
	private_nh_.param<double>("time", param.time, 2.0);
	private_nh_.param<double>("distance", param.distance, 8.0);
	private_nh_.param<double>("stop_distance", param.stop_distance, 6.0);
    private_nh_.param<double>("lateral_offset", param.lateral_offset, 5.0);
    private_nh_.param<bool>("use_path_planning", param.use_path_planning, false);
    private_nh_.param<bool>("use_path_planning", param.use_path_planning, false);
    private_nh_.param<double>("deceleration", param.deceleration, -1.0);

	
	// ros subscribers
	current_pose_sub_ = nh_.subscribe("odomData", 10, &Cooperation::callbackCurrentPose, this);
	chassis_sub_ = nh_.subscribe("chassis", 10, &Cooperation::callbackChassis, this); 
	platoon_opt_sub_ = nh_.subscribe("platoon_opt", 10, &Cooperation::callbackOpt, this); 

    platoonMember_sub_ = nh_.subscribe("/PlatoonMember", 10, &Cooperation::callbackPlatoonMember, this); 
	platoonMission_sub_ = nh_.subscribe("/PlatoonMission", 10, &Cooperation::callbackPlatoonMission, this); 
	platoonConfig_sub_ = nh_.subscribe("/PlatoonConfig", 10, &Cooperation::callbackPlatoonConfig, this); 

    //局域网内，使用topic传递消息，无论是主从机还是单个机器
    //都只能使用命名空间隔离。不同的车使用topic分享信息。
    //所以信息共享这块不能使用命名空间隔离。发布的消息其他车辆
    //包括本车都可以接收到，为避免重复，在多车仿真的时候，不使用self话题。
    bool open_simulate;
	private_nh_.param<bool>("open_simulate",  open_simulate, false);
	std::cout <<" open_simulate *******************  "<<open_simulate<<std::endl;
	if (!open_simulate){
		pub_mission_ = nh_.advertise<platoon_msgs::PlatoonMission>(
			"PlatoonMission_self", 1); 

		pub_config_ = nh_.advertise<platoon_msgs::PlatoonConfig>(
			"PlatoonConfig_self", 1); 
		
		pub_member_ = nh_.advertise<platoon_msgs::PlatoonMember>(
			"PlatoonMember_self", 1); 
	}
	else{
		pub_mission_ = nh_.advertise<platoon_msgs::PlatoonMission>(
			"/PlatoonMission", 1); 

		pub_config_ = nh_.advertise<platoon_msgs::PlatoonConfig>(
			"/PlatoonConfig", 1); 
		
		pub_member_ = nh_.advertise<platoon_msgs::PlatoonMember>(
			"/PlatoonMember", 1); 

	}
	
	pub_start_ = nh_.advertise<driver_msgs::MotionStartCmd>(
		"/chassis_motion_start_cmd",1);

	pub_diplay_ = nh_.advertise<visualization_msgs::MarkerArray>(
			"cooperationMarkers", 1);
	
	mode_cmd_pub_= nh_.advertise<driver_msgs::ModeCmd>("/vehicle_cmd_gate_mode_cmd",1);

	timer = nh_.createTimer(ros::Duration(0.1),&Cooperation::callbackTimer,this);
	goal_sub_  = nh_.subscribe("/move_base_simple/goal", 1, &Cooperation::callBackGoal, this);

    std::cout <<"-----------------Cooperation-----------------"<<std::endl;
	optsFunc.push_back(&Cooperation::none);
	optsFunc.push_back(&Cooperation::build);
	optsFunc.push_back(&Cooperation::join);
	optsFunc.push_back(&Cooperation::leave);
	optsFunc.push_back(&Cooperation::disolve);
	optsFunc.push_back(&Cooperation::column);
	optsFunc.push_back(&Cooperation::diamond);
	optsFunc.push_back(&Cooperation::triangle);
	optsFunc.push_back(&Cooperation::reverse);
	optsFunc.push_back(&Cooperation::cancel_reverse);
	optsFunc.push_back(&Cooperation::mass);
	optsFunc.push_back(&Cooperation::distribute);
	optsFunc.push_back(&Cooperation::running);

	optsFuncSelf.push_back(&Cooperation::noneSelf);
	optsFuncSelf.push_back(&Cooperation::buildSelf);
	optsFuncSelf.push_back(&Cooperation::joinSelf);
	optsFuncSelf.push_back(&Cooperation::leaveSelf);
	optsFuncSelf.push_back(&Cooperation::disolveSelf);
	optsFuncSelf.push_back(&Cooperation::columnSelf);
	optsFuncSelf.push_back(&Cooperation::diamondSelf);
	optsFuncSelf.push_back(&Cooperation::triangleSelf);	
	optsFuncSelf.push_back(&Cooperation::reverseSelf);
	optsFuncSelf.push_back(&Cooperation::cancel_reverseSelf);	
	optsFuncSelf.push_back(&Cooperation::massSelf);
	optsFuncSelf.push_back(&Cooperation::distributeSelf);
	optsFuncSelf.push_back(&Cooperation::runningSelf);

	platoonMembers[platformParam.num].id = platformParam.id;
	platoonMembers[platformParam.num].num = platformParam.num;
	platoonMembers[platformParam.num].role = FOLLOWER;
	platoonMembers[platformParam.num].type = NONE;
	platoonMembers[platformParam.num].driving_mode = MANUAL;
	platoonMembers[platformParam.num].front_overhang_m = vehcileInfo->front_overhang_m;
	platoonMembers[platformParam.num].rear_overhang_m  = vehcileInfo->rear_overhang_m ;
	platoonMembers[platformParam.num].wheel_base_m     = vehcileInfo->wheel_base_m    ;
	platoonMembers[platformParam.num].vehicle_width_m  = vehcileInfo->vehicle_width_m;
	platoonMembers[platformParam.num].vehicle_height_m  = vehcileInfo->vehicle_height_m;
	platoonMembers[platformParam.num].vehicle_length_m  = vehcileInfo->vehicle_length_m;
	
	display_thread_ = std::thread (&Cooperation::displayLoop,this);
	display_thread_.detach();

}


//leadPos x y z,leaderHeading arc
bool  Cooperation::followerReachedMassPoint(int index)
{ 
    assert(index > 0);
	double lengthToLeader = index * platoon_config.distance;
	double backHeading = amathutils::normalizeRadian(platoonMembers[vehicle_num_list[0]].heading+ M_PI);
	geometry_msgs::Point massPoint;
    massPoint.x = platoonMembers[vehicle_num_list[0]].position.x +  std::cos( backHeading) *  lengthToLeader;
    massPoint.y = platoonMembers[vehicle_num_list[0]].position.y +  std::sin( backHeading) *  lengthToLeader;

	double disToMassPoint = amathutils::distance2D(platoonMembers[vehicle_num_list[index]].position,massPoint);
	if (disToMassPoint < 3.0 && fabs(platoonMembers[vehicle_num_list[index]].linear_velocity) < 0.2 ){
	    std::cout <<"fronter vehicle reach mass point index :"<<index <<"num: "<<vehicle_num_list[index]<<std::endl;
		return true;
	}
	return false;
}

//leadPos x y z,leaderHeading arc
bool  Cooperation::fronterReachedMassPoint(int index)
{ 
	if (-1 == index)
		return true;
	
	double disToMassPoint = amathutils::distance2D(platoonMembers[vehicle_num_list[index]].position,distributeGoals[index].position);
	if (disToMassPoint < 3.0 && fabs(platoonMembers[vehicle_num_list[index]].linear_velocity) < 0.2 ){
	    std::cout <<"fronter vehicle reach distribute point index :"<<index <<"num: "<<vehicle_num_list[index]<<std::endl;
		return true;
	}
	
	return false;
}


void Cooperation::callbackTimer(const ros::TimerEvent &event)
{   	
	//集结应该是从前往后
    if (massFlag){
         int indexSelf  = platoonGetNumIndex(vehicle_num_list,platformParam.num);
         if (!followerReachedMassPoint(indexSelf-1))
		 	return;
		 massFlag = false;
		 toAuto();
		 toStartMove();
    }
	
	//分散应该是从后往前
    if (distributeFlag){
		
	     int indexSelf  = platoonGetNumIndex(vehicle_num_list,platformParam.num);
		 
         if (!fronterReachedMassPoint(indexSelf -1 ))
		 	return;
		 distributeFlag = false;
		 toStartMove();
    }

	if ((platoonMembers[platformParam.num].type = PlatoonType::MASS)
		&& (platoonMembers[platformParam.num].role == PlatoonRole::LEADER)){
		if (followerReachedMassPoint(vehicle_num_list.size()-1)){
			platoon_msgs::PlatoonMission mission;
			mission.command_type = PlatoonType::RUNNING;
			pub_mission_.publish(mission);
		}
	}
    

}

void Cooperation::displayLoop()
{
	 while (ros::ok()) {
        visualization_msgs::MarkerArray markArray;
	    int id = 0;
		for (auto iter_member = platoonMembers.begin(); iter_member != platoonMembers.end(); ++iter_member)
		{
		    if (iter_member->first == platformParam.num )
			    continue;
			
			vehicle_info_util::VehicleInfoUtil vehicle_mem_info;
			vehicle_mem_info.wheel_base_m = iter_member->second.wheel_base_m;
			vehicle_mem_info.front_overhang_m = iter_member->second.front_overhang_m;
			vehicle_mem_info.vehicle_width_m = iter_member->second.vehicle_width_m;
			vehicle_mem_info.vehicle_height_m = iter_member->second.vehicle_height_m;
			nav_msgs::Odometry odom_mem = amathutils::getOdometryFromPosAndYaw(iter_member->second.position,iter_member->second.heading); 
  
			DisplayConfig config;
			config.id = id;
			config.ns = std::string("/cooperation/vehicleMember");
			visualization_msgs::MarkerArray v_markArray  = DisPlay::vehicleMarkerArray(odom_mem,&vehicle_mem_info,config);
			markArray.markers.insert(markArray.markers.end(),v_markArray.markers.begin(),v_markArray.markers.end());
			id += 3;
		} 
		
		pub_diplay_.publish(markArray);
	    usleep(1000*50);
	 }

}
void  Cooperation::noneSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"noneSelf ,error opt "<<std::endl;

    return;
}

//本车为引导车，构建车队
void  Cooperation::buildSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
    
	platoon_msgs::PlatoonMember self = platoonMembers[platformParam.num];
	std::map<double,int> distance_map;
    for (auto iter = platoonMembers.begin(); iter != platoonMembers.end(); ++iter){
		double distance = amathutils::distance2D(self.position, iter->second.position);
		distance_map[distance] = iter->first;
    }
	
    for (auto iter = distance_map.begin(); iter != distance_map.end(); ++iter){
		vehicle_num_list.push_back(iter->second);
    }
	
	distributeGoals.resize(vehicle_num_list.size());
    std::cout <<"build,vehicle_num_list: ";
	for (auto &num:vehicle_num_list)
	{
	    std::cout <<  (int)num <<"  ";
	}
	std::cout <<std::endl;

	platoonMembers[platformParam.num].type = PlatoonType::BUILD;
	platoonMembers[platformParam.num].role = LEADER;
	
    platoon_msgs::PlatoonMission mission;
	mission.command_type = PlatoonType::BUILD;
	mission.config.time = param.time;
	mission.config.distance = param.distance;
	mission.config.lateral_offset = param.lateral_offset;
	mission.config.use_path_planning = param.use_path_planning;
	mission.config.deceleration = param.deceleration; 
	mission.vehicle_list = vehicle_num_list;
    platoon_config = mission.config;
	pub_mission_.publish(mission);
	toManual();
	toStopMove();
	platoonBuild = true;
	return;
}


//当本次作为引导车的时候
void  Cooperation::joinSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
    
    if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	std::cout <<"joinSelf "<<std::endl;

    if(platoonCheckNum(vehicle_num_list,msg->num))
		return;
	if (msg->num == platformParam.num)
	    platoonMembers[platformParam.num].type = PlatoonType::JOIN;
	
	vehicle_num_list.push_back(msg->num);
	
	platoon_msgs::PlatoonMission mission;
	mission.command_type = PlatoonType::JOIN;
	mission.num = msg->num;
	pub_mission_.publish(mission);

}

//本次作为引导车
void  Cooperation::leaveSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
    
    if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	std::cout <<"leaveSelf "<<std::endl;

    if(!platoonCheckNum(vehicle_num_list,msg->num))
		return;

	if (msg->num == platformParam.num)
	    platoonMembers[platformParam.num].type = PlatoonType::LEAVE;

	platoonEraseNum(vehicle_num_list,msg->num);
	platoon_msgs::PlatoonMission mission;
	mission.command_type = PlatoonType::LEAVE;
	mission.num = msg->num;
	pub_mission_.publish(mission);

}

//本车为引导车，解散车队
void  Cooperation::disolveSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"disolveSelf "<<std::endl;
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	std::vector<uint8>().swap(vehicle_num_list);
	
	platoonMembers[platformParam.num].type = PlatoonType::DISSOLVE;
    platoon_msgs::PlatoonMission mission;
	mission.command_type = PlatoonType::DISSOLVE;
	pub_mission_.publish(mission);
	platoonBuild = false;
    return ;
}


void  Cooperation::columnSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"columnSelf "<<std::endl;
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	
	platoonMembers[platformParam.num].type = PlatoonType::COLUMN;
	platoon_msgs::PlatoonMission mission;
	mission.config.time = param.time;
	mission.config.distance = param.distance;
	mission.config.lateral_offset = param.lateral_offset;
	mission.config.use_path_planning = param.use_path_planning;

	mission.command_type = PlatoonType::COLUMN;
	pub_mission_.publish(mission);

}

void  Cooperation::diamondSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"diamondSelf "<<std::endl;
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	platoonMembers[platformParam.num].type = PlatoonType::DIAMOND;
	platoon_msgs::PlatoonMission mission;
	mission.config.time = param.time;
	mission.config.distance = param.distance;
	mission.config.lateral_offset = param.lateral_offset;
	mission.config.use_path_planning = param.use_path_planning;

	mission.command_type = PlatoonType::DIAMOND;
	pub_mission_.publish(mission);

}


void  Cooperation::triangleSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"triangleSelf "<<std::endl;
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	platoonMembers[platformParam.num].type = PlatoonType::TRIANGLE;
	platoon_msgs::PlatoonMission mission;
	mission.config.time = param.time;
	mission.config.distance = param.distance;
	mission.config.lateral_offset = param.lateral_offset;
	mission.config.use_path_planning = param.use_path_planning;

	mission.command_type = PlatoonType::TRIANGLE;
	pub_mission_.publish(mission);
}


void  Cooperation::reverseSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"reverseSelf "<<std::endl;
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	
	platoonMembers[platformParam.num].type = PlatoonType::REVESE;
	platoon_msgs::PlatoonMission mission;
	mission.config.time = param.time;
	mission.config.distance = param.distance;
	mission.config.lateral_offset = param.lateral_offset;
	mission.config.use_path_planning = param.use_path_planning;

	mission.command_type = PlatoonType::REVESE;
	pub_mission_.publish(mission);
}

void  Cooperation::cancel_reverseSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"cancel_reverseSelf "<<std::endl;
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	platoonMembers[platformParam.num].type = PlatoonType::CANCEL_REVESE;
	platoon_msgs::PlatoonMission mission;
	mission.config.time = param.time;
	mission.config.distance = param.distance;
	mission.config.lateral_offset = param.lateral_offset;
	mission.config.use_path_planning = param.use_path_planning;

	mission.command_type = PlatoonType::CANCEL_REVESE;
	pub_mission_.publish(mission);
}


void  Cooperation::massSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"massSelf "<<std::endl;
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	platoonMembers[platformParam.num].type = PlatoonType::MASS;
	platoon_msgs::PlatoonMission mission;
	mission.config.time = param.time;
	mission.config.distance = param.distance;
	mission.config.lateral_offset = param.lateral_offset;
	mission.config.use_path_planning = param.use_path_planning;
	mission.command_type = PlatoonType::MASS;
	pub_mission_.publish(mission);
}


void  Cooperation::distributeSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	std::cout <<"distributeSelf "<<std::endl;
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	platoonMembers[platformParam.num].type = PlatoonType::DISTRIBUTE;
	platoon_msgs::PlatoonMission mission;
	mission.config.time = param.time;
	mission.config.distance = param.distance;
	mission.config.lateral_offset = param.lateral_offset;
	mission.config.use_path_planning = param.use_path_planning;
	mission.distributePoints = distributeGoals;
	mission.command_type = PlatoonType::DISTRIBUTE;
	distributeFlag = true;
	toStopMove();
	pub_mission_.publish(mission);
}



//
void  Cooperation::runningSelf(const platoon_msgs::PlatoonOpt::ConstPtr &msg){
	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
		return;
	
	std::cout <<"runningSelf "<<std::endl;
	
	platoon_msgs::PlatoonMission mission;
	mission.command_type = PlatoonType::RUNNING;
	pub_mission_.publish(mission);
	
	toAuto();
	toStartMove();

}


void  Cooperation::none(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"none opot "<<std::endl;

    return;
}


//本车为跟随车，构建车队
void  Cooperation::build(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"build opt "<<std::endl;

    platoon_msgs::PlatoonMission mission;
    vehicle_num_list = msg->vehicle_list;
	platoonMembers[platformParam.num].role = FOLLOWER;
	platoonMembers[platformParam.num].type = BUILD;
	platoonBuild = true;
	toManual();
	toStopMove();
	return;
}


void  Cooperation::join(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"join opt "<<std::endl;

    if(!platoonCheckNum(vehicle_num_list,msg->num)){
	    vehicle_num_list.push_back(msg->num);
		if (msg->num == platformParam.num)
			platoonMembers[platformParam.num].type = JOIN;

    }
}


void  Cooperation::leave(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	 std::cout <<"leave opt "<<std::endl;
	 if(platoonCheckNum(vehicle_num_list,msg->num)){
		 platoonEraseNum(vehicle_num_list,msg->num);
		 if (msg->num == platformParam.num)
			 platoonMembers[platformParam.num].type = LEAVE;
	 
	 }
}

//本车为跟随车，解散车队
void  Cooperation::disolve(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"disolve opt "<<std::endl;
	std::vector<uint8>().swap(vehicle_num_list);
	platoonMembers[platformParam.num].type = DISSOLVE;
	platoonBuild = false;
    return ;
}


void  Cooperation::column(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"column opt "<<std::endl;
	
	platoonMembers[platformParam.num].type = PlatoonType::COLUMN;
}

void  Cooperation::diamond(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"diamond opt "<<std::endl;
	
	platoonMembers[platformParam.num].type = PlatoonType::DIAMOND;

}

void  Cooperation::triangle(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"triangle opt "<<std::endl;
	platoonMembers[platformParam.num].type = PlatoonType::TRIANGLE;
}

void  Cooperation::reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"reverse opt "<<std::endl;
	platoonMembers[platformParam.num].type = PlatoonType::REVESE;
}


void  Cooperation::cancel_reverse(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"cancel_reverse opt "<<std::endl;
	platoonMembers[platformParam.num].type = PlatoonType::CANCEL_REVESE;
}


void  Cooperation::mass(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"mass  opt "<<std::endl;
	platoonMembers[platformParam.num].type = PlatoonType::MASS;

	if (platoonMembers[platformParam.num].role != PlatoonRole::LEADER)
	    massFlag = true;
}


void  Cooperation::distribute(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"distribute opt "<<std::endl;
	platoonMembers[platformParam.num].type = PlatoonType::DISTRIBUTE;
	distributeGoals = msg->distributePoints;
	toStopMove();
	distributeFlag = true;
}

void  Cooperation::running(const platoon_msgs::PlatoonMission::ConstPtr &msg){
	std::cout <<"running opt "<<std::endl;
	platoonMembers[platformParam.num].type = PlatoonType::RUNNING;
	
	toAuto();
	toStartMove();

}


void Cooperation::callbackCurrentPose(const localization_msgs::Localization::ConstPtr &msg) {
    current_pose.x = msg->location.pose.pose.position.x;
    current_pose.y = msg->location.pose.pose.position.y;
    current_pose.heading = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2);
	odom = amathutils::getOdometryFromPosAndYaw(msg->location.pose.pose.position,current_pose.heading);	

	platoonMembers[platformParam.num].position.x = current_pose.x;
	platoonMembers[platformParam.num].position.y = current_pose.y;
	platoonMembers[platformParam.num].heading = current_pose.heading;
	//std::cout <<"current_pose.heading:  "<<platformParam.id<<std::endl;
	pub_member_.publish(platoonMembers[platformParam.num]);
}

void Cooperation::callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg)
{
	double vel = (7 == msg->gear_location)?(-msg->current_velocity):msg->current_velocity;
	platoonMembers[platformParam.num].linear_velocity = vel;
	platoonMembers[platformParam.num].gear =  msg->gear_location;
	platoonMembers[platformParam.num].driving_mode =  msg->driving_mode;
	
}


void Cooperation::callbackOpt(const platoon_msgs::PlatoonOpt::ConstPtr &msg)
{
    assert(msg->cmd >= PlatoonType::BUILD);
	assert(msg->cmd <= PlatoonType::RUNNING);

	unsigned char  optType = msg->cmd;
    if ((optType != PlatoonType::BUILD) && (!platoonBuild))
		return;
    
    if ((optType == PlatoonType::BUILD) && (platoonBuild))
		return;	

	(this->*optsFuncSelf[msg->cmd])(msg);
}


//其他车辆的信息
void Cooperation::callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg) {
    //std::cout <<"callbackPlatoonMember"<<std::endl;
	//std::cout <<msg->id<<std::endl;
/*
	if (platoonMembers.find(msg->id) != platoonMembers.end())
		platoonMembers[msg->id] = *msg;
	else
		platoonMembers.insert(std::pair<std::string, platoon_msgs::PlatoonMember>(msg->id, *msg));
*/
	platoonMembers[msg->num] = *msg;
	return;
}


//来自与其他引导车的指令
void Cooperation::callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg) {
    assert(msg->command_type>=PlatoonType::BUILD);
	assert(msg->command_type<=PlatoonType::RUNNING);

	unsigned char  optType = msg->command_type;
    if ((optType != PlatoonType::BUILD) && (!platoonBuild))
		return;
    
    if ((optType == PlatoonType::BUILD) && (platoonBuild))
		return;	

	(this->*optsFunc[msg->command_type])(msg);
	return;
}


//来自与其他引导车的编队参数
void Cooperation::callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg) {
    std::cout <<"callbackPlatoonConfig"<<std::endl;
    std::cout <<"policy "<<msg->policy<<std::endl;

    platoon_config = *msg;
	return;
}

void Cooperation::callBackGoal(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    if (!platoonBuild)
		return;
	if ( platoonMembers[platformParam.num].role != LEADER )
		return;
	
    static int count = 0;
	
    std::cout <<"Cooperation get callBackGoal"<<std::endl;
	distributeGoals[count] = msg->pose;

    visualization_msgs::MarkerArray markerArray;
    DisplayConfig cfg;
	cfg.scale_x  = 1.0;
	cfg.scale_y  = 2.0;
	cfg.scale_z  = 1.0;
	cfg.quaternion = msg->pose.orientation;
	cfg.position = msg->pose.position;
	cfg.id = count;
	cfg.ns = std::string("/cooperation/distributePoints");
	markerArray.markers.push_back(DisPlay::arrowMarkerMethod2(cfg));
	pub_diplay_.publish(markerArray); 
	
	count = (count+1 )% distributeGoals.size();
}




bool Cooperation::toStartMove()
{
	//前车也是无人的情况下
	driver_msgs::MotionStartCmd motionStartCmd;
	motionStartCmd.motion_start = 1;
	pub_start_.publish(motionStartCmd);
	return true;
}

bool Cooperation::toStopMove()
{
	//前车也是无人的情况下
	driver_msgs::MotionStartCmd motionStartCmd;
	motionStartCmd.motion_start = 0;
	pub_start_.publish(motionStartCmd);
	
	return true;
}


bool Cooperation::toManual()
{
	driver_msgs::ModeCmd msgModeCmd_;
	
    msgModeCmd_.driving_mode = 0;
	mode_cmd_pub_.publish(msgModeCmd_);

	return true;
}


bool Cooperation::toAuto()
{
	driver_msgs::ModeCmd msgModeCmd_;

	msgModeCmd_.driving_mode = 1;
	msgModeCmd_.gear_mode = 1;
	msgModeCmd_.mode_flag = 0;
	mode_cmd_pub_.publish(msgModeCmd_);

	return true;
}


int main(int argc ,char *argv[])
{        
	ros::init(argc, argv, "cooperation");
	ros::NodeHandle nh;
	ROS_INFO("cooperation start.");

	Cooperation cooperation(nh);
	ros::spin();
	ROS_INFO(" cooperation The iteration end.");
	return 0;
}



