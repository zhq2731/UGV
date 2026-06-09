#include "cloud_comm/cloud_comm.h"



CloudComm::~CloudComm()
{
	udp_send_ptr->exit();
	udp_recv_ptr->exit();
}

CloudComm::CloudComm(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{ 

	std::string vehicle_platform_file;

	private_nh_.param<std::string>("vehicle_platform_file", vehicle_platform_file, "vehicle_platform.yaml");

	::common::getPlatformParam("vehicle_platform.yaml",platform_param);
	

 	std::string pkg_dir = ros::package::getPath("launch_node");
	std::string vehicleIpfile = pkg_dir + std::string("/param/global/vehicles_ip.yaml");

	platform_param.id = "zhito_A";
	common::IpParam vehicleIpParam = common::getVehicleIp(vehicleIpfile, platform_param);
	
	std::cout <<"vehicleIpParam  ip "<<vehicleIpParam.local_ip <<std::endl;
	std::cout <<"vehicleIpParam  port "<<vehicleIpParam.local_port <<std::endl;
	std::cout <<"cloudIpParam  ip "<<vehicleIpParam.cloud_single_ip <<std::endl;
	std::cout <<"cloudIpParam  port "<<vehicleIpParam.cloud_single_port <<std::endl;
	std::cout <<"multiIpParam  ip "<<vehicleIpParam.cloud_multi_ip <<std::endl;
	std::cout <<"multiIpParam  port "<<vehicleIpParam.cloud_multi_port <<std::endl;
	netInfo.host_ip=vehicleIpParam.local_ip;
	netInfo.host_port=vehicleIpParam.local_port;
	netInfo.cloud_ip=vehicleIpParam.cloud_single_ip;
	netInfo.cloud_port=vehicleIpParam.cloud_single_port;
	netInfo.multicast_ip=vehicleIpParam.cloud_multi_ip;
	netInfo.multicast_port=vehicleIpParam.cloud_multi_port;
	  
	numInfo.vehicle_num = platform_param.num;
	std::cout <<platform_param.vehicle_type<<std::endl;
    std::cout <<platform_param.ins_type<<std::endl;
    std::cout <<platform_param.id<<std::endl;
    std::cout <<platform_param.num<<std::endl;

	socketInit(netInfo.timeOutmilliSecond);
    // sleep(1);

	error_feedback_ = nh_.subscribe("/error", 1, &CloudComm::callbackError, this);
	chassis_status_feedback_ = nh_.subscribe("/chassis", 1, &CloudComm::callbackChassisStatus, this);
	localization_feedback_ = nh_.subscribe("/odomData",1,&CloudComm::callbackLocalization, this); 
	// obs_feedback_ = nh_.subscribe("/MultiObjectTracker",1,&CloudComm::callbackObstacle, this);
	configuration_feedback_ = nh_.subscribe("/configuration_up",1,&CloudComm::callbackConfig, this);
	path_feedback_ = nh_.subscribe("/cloud_route_display", 1, &CloudComm::callbackPath, this);                //上报路径
	platoon_state_ = nh_.subscribe("/PlatoonMember", 1, &CloudComm::callbackPlatoonState, this);
	platoon_state_self = nh_.subscribe("/PlatoonMember_self", 1, &CloudComm::callbackPlatoonState, this);
	//department two display
	// map_request_ = nh_.subscribe("/request_new_map",1,&CloudComm::callbackRequest, this);
	origin_feedback_ = nh_.subscribe("/init_point",1,&CloudComm::callbackOrigin, this);             //坐标原点经纬度（未用到）
	reference_feedback_ = nh_.subscribe("/referenceLine", 1, &CloudComm::callbackReference, this);
	trajectory_feedback_ = nh_.subscribe("/trajectory", 1, &CloudComm::callbackTrajectory, this);

	power_cmd_pub_=nh_.advertise<driver_msgs::PowerCmd>("/remote_chassis_power_cmd",1);
	motion_cmd_pub_=nh_.advertise<driver_msgs::MotionStartCmd>("/chassis_motion_start_cmd",1);
	mode_cmd_pub_=nh_.advertise<driver_msgs::ModeCmd>("/vehicle_cmd_gate_mode_cmd",1);
	gear_cmd_pub_=nh_.advertise<driver_msgs::GearCmd>("/remote_chassis_gear_cmd",1);
	light_cmd_pub_=nh_.advertise<driver_msgs::LightHornWiperCmd>("/remote_chassis_light_horn_wiper_cmd",1);
	estop_cmd_pub_=nh_.advertise<driver_msgs::EstopCmd>("/remote_chassis_estop_cmd",1);//新话题
	steering_cmd_pub_=nh_.advertise<driver_msgs::SteeringWheelCmd>("/remote_chassis_steeringwheel_cmd",1);
	drive_cmd_pub_=nh_.advertise<driver_msgs::DriveCmd>("/remote_chassis_drive_cmd",1);
	speed_cmd_pub_=nh_.advertise<std_msgs::Float32>("/remote_chassis_speed_cmd",1);//新话题

	cruise_cmd_pub_=nh_.advertise<std_msgs::Float64>("/desireSpeed",1);
	//backingUp=nh_.advertise<geometry_msgs::Point>("/backingup_cmd",1);
	parking_cmd_pub_=nh_.advertise<driver_msgs::ParkingBrakeCmd>("/remote_chassis_parking_brake_cmd",1);
	//estop_cmd_pub_ = nh_.advertise<driver_msgs::EstopCmd>("/remote_chassis_estop",1);
	chassis_cmd_pub_ = nh_.advertise<driver_msgs::ChassisCmd>("/chassis_cmd",1);
	configuration_pub_ = nh_.advertise<configuration_msgs::ConfigurationDown>("/configuration_down",1);
	heart_pub_ = nh_.advertise<heartbeat_msgs::Heartbeat>("/heartbeat",1);
	path_info_pub_=nh_.advertise<planning_msgs::TrajectoryPointArray>("/trajectory_point_array",1);
	points_planned_pub_ = nh_.advertise<planning_msgs::TrajectoryPointArray>("/points_tobe_planned",1);

	platoon_control_pub_ = nh_.advertise<platoon_msgs::PlatoonMission>("/platoon_opt",1);
	
	//department two display
	multiple_point_pub_ = nh_.advertise<route_msgs::MultiPoint>("/multi_point_planning",1);
	block_point_pub_ = nh_.advertise<route_msgs::Replan>("/replan",1);
	map_sign_pub_ = nh_.advertise<std_msgs::UInt8>("/mapSign",1);
	satellite_mode_pub_ = nh_.advertise<std_msgs::UInt8>("/mode",1);
	obs_pub_ = nh_.advertise<perception_msgs::PredictionObstacles>("/GlobalMultiObjectTracker",1);
	initPoint_pub_ = nh_.advertise<route_msgs::InitPoint>("/init_point",1);
	
	msgsFunc[0x3060] = &CloudComm::chassisControl;
	msgsFunc[0x1010] = &CloudComm::heartOpt;
	msgsFunc[0xD050] = &CloudComm::viewChange;
	msgsFunc[0x8080] = &CloudComm::paramConfig;
	msgsFunc[0x80D0] = &CloudComm::pathInfoOpt;
	msgsFunc[0x4050] = &CloudComm::pointTobePlanned;
	// vehicle queue
	msgsFunc[0x5090] = &CloudComm::queueControl;
	//department two display
	msgsFunc[0x9010] = &CloudComm::multipleTaskPoints;
	msgsFunc[0x9020] = &CloudComm::blockPoints;
	msgsFunc[0x9030] = &CloudComm::mapSign;
	msgsFunc[0x9050] = &CloudComm::satState;
	msgsFunc[0x9060] = &CloudComm::obstacleOpt;
	msgsFunc[0x9070] = &CloudComm::initPoint;
	msgsFunc[0x90A0] = &CloudComm::mapRequest;
	
	timer_status_ = nh_.createTimer(ros::Duration(0.05),&CloudComm::callbackTimerStatusFeedback,this);
	timer_heart_= nh_.createTimer(ros::Duration(0.1),&CloudComm::callbackTimerHeartFeedback,this);
	timer_location_= nh_.createTimer(ros::Duration(0.1),&CloudComm::callbackTimerLocationFeedback,this);
	timer_platoonState_ = nh_.createTimer(ros::Duration(0.1),&CloudComm::callbackTimerPlatoonFeedback,this);
	timer_obstacle_ = nh_.createTimer(ros::Duration(0.1),&CloudComm::callbackTimerConfigFeedback,this);
	timer_sensorStatus_ = nh_.createTimer(ros::Duration(0.1),&CloudComm::callbackTimerSensorFeedback,this);
	// callbackTimerConfigFeedback();
	udp_thread= std::thread (&CloudComm::udpThreadFunc,this);
	udp_thread.detach();
	// serial_thread= std::thread (&CloudComm::serialThreadFunc,this);
	// serial_thread.detach();
	
	//logger();
}

void CloudComm::udpThreadFunc()
{
	int count=0;
	
	while(true)
	{	
		cout << "LALALLLALALALAL" << endl;
		std::cout<<"数据："<<count<<std::endl;
		
		int recvLength = udp_recv_ptr->receive(recvBuf, sizeof(recvBuf));
		if ((recvLength <= 0 )){
			std::cout << "123" <<std::endl;
			continue;
		}
			
		cout << "LALALLLALALALAL" << endl;
		std::cout<<"数据："<<count<<std::endl;

		MsgHeader msgHeader_;
		memcpy(&msgHeader_,recvBuf,sizeof(msgHeader_));
		count++;
		

		(this->*msgsFunc[msgHeader_.msg_id])(recvBuf);
		
		// usleep(10000);
	}
}

void CloudComm::serialThreadFunc(){
	
	serial_node.InitJoySerial("/dev/ttyACM0");
	while(true){
		serial_node.getData();
		// std::cout << "急停 " << std::hex << int(serial_node.getData().light[1] & 0x01) << std::endl;
		// std::cout << "左转 " << std::hex << int(serial_node.getData().light[1] & 0x02) << std::endl;
		// std::cout << "右转 " << std::hex << int(serial_node.getData().light[1] & 0x04) << std::endl;
		// std::cout << "档位 " << std::hex << int(serial_node.getData().light[1] & (0x08 | 0x10 | 0x20 | 0x80)) << std::endl;
		// std::cout << "速度 " << std::dec << int(serial_node.getData().updown1) << std::endl;
		// std::cout << "左右转向角度 " << std::dec << int(serial_node.getData().leftright2) << std::endl;
		sleep(0.5);
	}
	
}

bool CloudComm::socketInit(int milliSecond)
{
	 udp_send_ptr = std::make_shared<udpServer>();
	 udp_recv_ptr = std::make_shared<udpServer>();
	 std::cout <<"host_port "<<netInfo.host_port<<std::endl;
	 std::cout <<"host_ip "<<netInfo.host_ip<<std::endl;
	 udp_recv_ptr->setup(netInfo.host_port,netInfo.host_ip);
	 udp_recv_ptr->setTimeOut(milliSecond);

	 udp_send_ptr_multi = std::make_shared<MulticastSender>(netInfo.multicast_ip, netInfo.multicast_port,netInfo.host_ip,1,false);
	// udp_recv_ptr = std::make_shared<MulticastReceiver>(netInfo.info_multi_ip, vehicleIpParam.info_multi_port,vehicleIpParam.local_ip);	 

	 
	 return true;
}

uint8 CloudComm::getCheckSum(uint8 *data,int size)
{
	uint8 checkVal=0;
	for(int i=0;i<size;i++)
		{
			checkVal^=data[i];
		}
	return checkVal;
}

uint32 CloudComm::Acquire24AbsTime()
{
	struct timeval curTime;
	gettimeofday(&curTime, NULL);
	time_t t = time(NULL);
	tm *local;
	local = localtime(&t);			
    uint32 msg_time = curTime.tv_usec/1000 + ((local->tm_hour*60 + local->tm_min)*60 + local->tm_sec)*1000;
	return msg_time;
}


// void CloudComm::callbackTimerHeartFeedback(const ros::TimerEvent &event){
	
// 	int count = 0;
// 	MsgHeader msgHeader;
// 	MsgEnd msgEnd;
// 	UdpHeartFeedback udpHeartFeedback;
// 	unsigned char hard_num;
// 	vector<unsigned short> hard_error;
// 	unsigned char soft_num;
// 	vector<unsigned short> soft_error;

// 	msgHeader.msg_id  = 0x6010;
// 	msgHeader.msg_source_add = inet_addr(netInfo.host_ip.c_str());
// 	msgHeader.msg_time = Acquire24AbsTime();
// 	msgHeader.msg_destination_add = inet_addr(netInfo.multicast_ip.c_str());
// 	msgHeader.msg_vehicle_num = numInfo.vehicle_num;
// 	msgHeader.msg_flag = 0x00FF;
// 	msgHeader.msg_num++;


// 	hard_num = hard_errcode.size();
// 	soft_num = soft_errcode.size();



// 	ros::Time now = ros::Time::now();
//     time_t time_sec = now.sec;
//     struct tm* timeinfo = localtime(&time_sec);
    
// 	udpHeartFeedback.year = static_cast<short>(timeinfo->tm_year + 1900);
//     udpHeartFeedback.month = static_cast<unsigned char>(timeinfo->tm_mon + 1);
//     udpHeartFeedback.day = static_cast<unsigned char>(timeinfo->tm_mday);
//     udpHeartFeedback.hour = static_cast<unsigned char>(timeinfo->tm_hour);
//     udpHeartFeedback.minute = static_cast<unsigned char>(timeinfo->tm_min);
//     udpHeartFeedback.second = static_cast<unsigned char>(timeinfo->tm_sec);
//     udpHeartFeedback.millisecond = static_cast<short>(now.nsec / 1000000);


	
// 	memcpy(sendBuf,&msgHeader,sizeof(msgHeader));
// 	memcpy(sendBuf+sizeof(msgHeader), &udpHeartFeedback, sizeof(udpHeartFeedback));
// 	memcpy(sendBuf+sizeof(msgHeader)+sizeof(udpHeartFeedback), &hard_num, sizeof(hard_num));
// 	count += sizeof(msgHeader) + sizeof(udpHeartFeedback) + sizeof(hard_num);
// 	for(int i = 0; i < hard_num; i++){
// 		memcpy(sendBuf+count+i,&hard_errcode.at(i),sizeof(unsigned short));
// 	}
// 	memcpy(sendBuf+count+hard_num*sizeof(unsigned short),&soft_num,sizeof(soft_num));
// 	count += hard_num*sizeof(unsigned short) + sizeof(soft_num);
// 	for(int j = 0; j < soft_num; j++){
// 		memcpy(sendBuf+count+j,&soft_errcode.at(j),sizeof(unsigned short));
// 	}
// 	count += soft_num*sizeof(unsigned short);
// 	memcpy(sendBuf+count,&msgEnd,sizeof(msgEnd));
// 	count += sizeof(msgEnd);

// 	msgHeader.msg_length = count;
// 	udp_send_ptr->send(sendBuf, count, netInfo.multicast_ip, netInfo.multicast_port);

// }

                                                       //下面这个心跳结构体不包含软硬件设备状态

void CloudComm::callbackTimerHeartFeedback(const ros::TimerEvent &event){

    //if(!heartFeedbackFlag)
		//return ;
	//heartFeedbackFlag = false;
	
	FullUdpHeartFeedback fullUdpHeartFeedback_;
	UdpHeartFeedback udpHeartFeedback;

	fullUdpHeartFeedback_.msgHeader_.msg_id  = 0x6010;
	fullUdpHeartFeedback_.msgHeader_.msg_length = sizeof(fullUdpHeartFeedback_);
	fullUdpHeartFeedback_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
	fullUdpHeartFeedback_.msgHeader_.msg_destination_add = inet_addr(netInfo.multicast_ip.c_str()) ;
	fullUdpHeartFeedback_.msgHeader_.msg_flag = 0x00FF;
	fullUdpHeartFeedback_.msgHeader_.msg_num++;
	fullUdpHeartFeedback_.msgHeader_.msg_vehicle_num = numInfo.vehicle_num;
	fullUdpHeartFeedback_.msgHeader_.msg_time = Acquire24AbsTime();

	ros::Time now = ros::Time::now();
    time_t time_sec = now.sec;
    struct tm* timeinfo = localtime(&time_sec);
	udpHeartFeedback.year = static_cast<short>(timeinfo->tm_year + 1900);
    udpHeartFeedback.month = static_cast<unsigned char>(timeinfo->tm_mon + 1);
    udpHeartFeedback.day = static_cast<unsigned char>(timeinfo->tm_mday);
    udpHeartFeedback.hour = static_cast<unsigned char>(timeinfo->tm_hour);
    udpHeartFeedback.minute = static_cast<unsigned char>(timeinfo->tm_min);
    udpHeartFeedback.second = static_cast<unsigned char>(timeinfo->tm_sec);
    udpHeartFeedback.millisecond = static_cast<short>(now.nsec / 1000000);

	fullUdpHeartFeedback_.udpHeartFeedback_ = udpHeartFeedback;

	memcpy(sendBuf,&fullUdpHeartFeedback_,sizeof(fullUdpHeartFeedback_));
	udp_send_ptr_multi->send(sendBuf, sizeof(FullUdpHeartFeedback));
	//udp_send_ptr->send(sendBuf, sizeof(FullUdpHeartFeedback), netInfo.multicast_ip, netInfo.multicast_port);

}

void CloudComm::callbackTimerStatusFeedback(const ros::TimerEvent &event)
{
	
    if(!statusFeedbackFlag)
		return ;
	statusFeedbackFlag = false;	
	
	FullUdpChassisFeedback fullUdpChassisFeedback_;

	fullUdpChassisFeedback_.msgHeader_.msg_id  = 0x8020;
	fullUdpChassisFeedback_.msgHeader_.msg_length = sizeof(fullUdpChassisFeedback_);
	fullUdpChassisFeedback_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
	fullUdpChassisFeedback_.msgHeader_.msg_destination_add = inet_addr(netInfo.multicast_ip.c_str()) ;
	fullUdpChassisFeedback_.msgHeader_.msg_flag = 0x00FF;
	fullUdpChassisFeedback_.msgHeader_.msg_num++;
	fullUdpChassisFeedback_.msgHeader_.msg_vehicle_num = numInfo.vehicle_num;
	fullUdpChassisFeedback_.msgHeader_.msg_time = Acquire24AbsTime();

	fullUdpChassisFeedback_.udpChassisFeedback_ = udpChassisFeedback;
	fullUdpChassisFeedback_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpChassisFeedback_,sizeof(fullUdpChassisFeedback_)-sizeof(fullUdpChassisFeedback_.msgEnd_));
	// cout << "chesu" << fullUdpChassisFeedback_.udpChassisFeedback_.VehicleFbk << endl;
	// cout << "dianliang" << fullUdpChassisFeedback_.udpChassisFeedback_.oil << endl;
	memcpy(sendBuf,&fullUdpChassisFeedback_,sizeof(fullUdpChassisFeedback_));
	//udp_send_ptr->send(sendBuf, sizeof(FullUdpChassisFeedback), netInfo.multicast_ip, netInfo.multicast_port);
	udp_send_ptr_multi->send(sendBuf, sizeof(FullUdpChassisFeedback));
	
	// std::cout << "是否成功发出： " << udp_send_ptr->send(sendBuf, sizeof(FullUdpChassisFeedback), netInfo.multicast_ip, netInfo.multicast_port) << std::endl;
}

void CloudComm::callbackTimerLocationFeedback(const ros::TimerEvent &event){

    if(!locationFeedbackFlag)
		return ;
	locationFeedbackFlag = false;

	FullUdpLocationFeedback fullUdpLocationFeedback_;

	fullUdpLocationFeedback_.msgHeader_.msg_id  = 0x8030;
	fullUdpLocationFeedback_.msgHeader_.msg_length = sizeof(FullUdpLocationFeedback);
	fullUdpLocationFeedback_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
	fullUdpLocationFeedback_.msgHeader_.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
	fullUdpLocationFeedback_.msgHeader_.msg_flag = 0x00FF;
	fullUdpLocationFeedback_.msgHeader_.msg_num++;
	fullUdpLocationFeedback_.msgHeader_.msg_time = Acquire24AbsTime();
	fullUdpLocationFeedback_.msgHeader_.msg_vehicle_num = numInfo.vehicle_num;

	fullUdpLocationFeedback_.udpLocationFeedback_ = udpLocationFeedback;
	// std::cout << "车辆經度： " << std::setprecision(15) << fullUdpLocationFeedback_.udpLocationFeedback_.longitude << std::endl;
	// std::cout << "车辆纬度： " << std::setprecision(15) << fullUdpLocationFeedback_.udpLocationFeedback_.latitude << std::endl;
	// std::cout << "车辆坐标x: " << fullUdpLocationFeedback_.udpLocationFeedback_.coordinate_.x << std::endl;
	// std::cout << "车辆坐标y: " << fullUdpLocationFeedback_.udpLocationFeedback_.coordinate_.y << std::endl;
	// std::cout << "车辆坐标z: " << fullUdpLocationFeedback_.udpLocationFeedback_.coordinate_.z << std::endl;
	// std::cout << "车辆航向角： " << fullUdpLocationFeedback_.udpLocationFeedback_.posture_.orientation << std::endl;
	// std::cout << "车辆俯仰角： " << fullUdpLocationFeedback_.udpLocationFeedback_.posture_.pitch << std::endl;
	// std::cout << "车辆横滚角： " << fullUdpLocationFeedback_.udpLocationFeedback_.posture_.roll << std::endl;
	fullUdpLocationFeedback_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpLocationFeedback_,sizeof(fullUdpLocationFeedback_)-sizeof(fullUdpLocationFeedback_.msgEnd_));
	// cout << "lalalala" << udpLocationFeedback.latitude << endl;
	memcpy(sendBuf,&fullUdpLocationFeedback_,sizeof(fullUdpLocationFeedback_));
	udp_send_ptr->send(sendBuf, sizeof(FullUdpLocationFeedback), netInfo.cloud_ip, netInfo.cloud_port);
	// udp_send_ptr_multi->send(sendBuf, sizeof(FullUdpLocationFeedback));

}

void CloudComm::callbackTimerSensorFeedback(const ros::TimerEvent &event){
    if(!sensorFeedbackFlag)
		return ;
	sensorFeedbackFlag = false;

	FullUdpSensorFeedback fullUdpSensorFeedback_;
	fullUdpSensorFeedback_.msgHeader_.msg_id  = 0x7080;
	fullUdpSensorFeedback_.msgHeader_.msg_length = sizeof(FullUdpSensorFeedback);
	fullUdpSensorFeedback_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
	fullUdpSensorFeedback_.msgHeader_.msg_destination_add = inet_addr(netInfo.multicast_ip.c_str()) ;
	fullUdpSensorFeedback_.msgHeader_.msg_flag = 0x00FF;
	fullUdpSensorFeedback_.msgHeader_.msg_num++;
	fullUdpSensorFeedback_.msgHeader_.msg_time = Acquire24AbsTime();
	fullUdpSensorFeedback_.msgHeader_.msg_vehicle_num = numInfo.vehicle_num;

	fullUdpSensorFeedback_.udpSensorFeedback_ = udpSensorFeedback;
	fullUdpSensorFeedback_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpSensorFeedback_,sizeof(fullUdpSensorFeedback_)-sizeof(fullUdpSensorFeedback_.msgEnd_));

	memcpy(sendBuf,&fullUdpSensorFeedback_,sizeof(fullUdpSensorFeedback_));
	udp_send_ptr_multi->send(sendBuf, sizeof(FullUdpSensorFeedback));
	//udp_send_ptr->send(sendBuf, sizeof(FullUdpSensorFeedback), netInfo.multicast_ip, netInfo.multicast_port);
}

void CloudComm::callbackTimerPlatoonFeedback(const ros::TimerEvent &event){

    
	for (auto iter = platoonFeedBack.begin(); iter != platoonFeedBack.end(); ++iter)
	{
		if ( false == infos[iter->first])
			continue;
		
		infos[iter->first] = false;
		FullUdpPlatoonStateFeedback fullUdpPlatoonStateFeedback_;
		fullUdpPlatoonStateFeedback_.msgHeader_.msg_id  = 0x5080;
		fullUdpPlatoonStateFeedback_.msgHeader_.msg_length = sizeof(FullUdpPlatoonStateFeedback);
		fullUdpPlatoonStateFeedback_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
		fullUdpPlatoonStateFeedback_.msgHeader_.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
		fullUdpPlatoonStateFeedback_.msgHeader_.msg_flag = 0x00FF;
		fullUdpPlatoonStateFeedback_.msgHeader_.msg_num++;
		fullUdpPlatoonStateFeedback_.msgHeader_.msg_time = Acquire24AbsTime();
		fullUdpPlatoonStateFeedback_.msgHeader_.msg_vehicle_num = iter->first;

		fullUdpPlatoonStateFeedback_.udpPlatoonStateFeedback_ = platoonFeedBack[iter->first];
		fullUdpPlatoonStateFeedback_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpPlatoonStateFeedback_,sizeof(fullUdpPlatoonStateFeedback_)-sizeof(fullUdpPlatoonStateFeedback_.msgEnd_));

		memcpy(sendBuf,&fullUdpPlatoonStateFeedback_,sizeof(fullUdpPlatoonStateFeedback_));
		//udp_send_ptr_multi->send(sendBuf, sizeof(FullUdpPlatoonStateFeedback));
		//std::cout <<"send data num :"<<iter->first<<std::endl;
		udp_send_ptr->send(sendBuf, sizeof(FullUdpPlatoonStateFeedback), netInfo.cloud_ip, netInfo.cloud_port);	
		
	}

}

void CloudComm::callbackTimerConfigFeedback(const ros::TimerEvent &event){
    if(!configFeedbackFlag)
		return ;
	configFeedbackFlag = false;

	FullUdpConfigFeedback fullUdpConfigFeedback_;

	fullUdpConfigFeedback_.msgHeader_.msg_id  = 0x6090;	
	fullUdpConfigFeedback_.msgHeader_.msg_length = sizeof(FullUdpConfigFeedback);	
	fullUdpConfigFeedback_.msgHeader_.msg_num++;
	fullUdpConfigFeedback_.msgHeader_.msg_time = Acquire24AbsTime();
	fullUdpConfigFeedback_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str()) ;
	fullUdpConfigFeedback_.msgHeader_.msg_destination_add = inet_addr(netInfo.multicast_ip.c_str()) ;
	fullUdpConfigFeedback_.msgHeader_.msg_flag = 0x00FF ;
	fullUdpConfigFeedback_.msgHeader_.msg_time = Acquire24AbsTime();
	fullUdpConfigFeedback_.msgHeader_.msg_vehicle_num = numInfo.vehicle_num;

	fullUdpConfigFeedback_.udpConfigFeedback_ = udpConfigFeedback;
	// std::cout << "状态配置 " << fullUdpConfigFeedback_.udpConfigFeedback_.config_status << std::endl;
	
	fullUdpConfigFeedback_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpConfigFeedback_,sizeof(fullUdpConfigFeedback_)-sizeof(fullUdpConfigFeedback_.msgEnd_));

    memcpy(sendBuf,&fullUdpConfigFeedback_,sizeof(fullUdpConfigFeedback_));
	udp_send_ptr_multi->send(sendBuf, sizeof(FullUdpConfigFeedback));
	//udp_send_ptr->send(sendBuf, sizeof(FullUdpConfigFeedback), netInfo.multicast_ip, netInfo.multicast_port);
}





void CloudComm::callbackChassisStatus(const driver_msgs::ChassisReport::ConstPtr &msg){
	udpChassisFeedback.driving_mode = msg->driving_mode;
	// udpChassisFeedback.driving_mode_switch = msg->driving_mode_switch;         //没有无人驾驶开关
	udpChassisFeedback.autonomous_mode = msg->mode_flag;
	// udpChassisFeedback.oil = msg->remaining_electricity;
	udpChassisFeedback.oil = msg->remaining_oil;
	udpChassisFeedback.mile = msg->total_kilometres;
	udpChassisFeedback.EngineSpeedFbk = msg->current_engine_speed;
	udpChassisFeedback.EngineTorque = msg->current_engine_torque;
	udpChassisFeedback.throttle_pedal = msg->throttle_pedal;
	udpChassisFeedback.BrakePedalSTA = msg->brake_pedal;
	// udpChassisFeedback.max_speed = ;      //无最大车速
	udpChassisFeedback.VehicleFbk = msg->current_velocity;
	udpChassisFeedback.SteerWheelFbk = msg->steering_wheel_angle;
	udpChassisFeedback.steer_mode = msg->steer_mode;
	udpChassisFeedback.ParkStaFbk = msg->parking_brake;
	udpChassisFeedback.gear_location = msg->gear_location;
	udpChassisFeedback.position_light = msg->position_light;
	udpChassisFeedback.left_light = msg->left_light;
	udpChassisFeedback.right_light = msg->right_light;
	udpChassisFeedback.rear_light = msg->rear_foggy_light;
	udpChassisFeedback.blackout_light = msg->air_lamp_fbk;
	udpChassisFeedback.high_light = msg->high_light;
	udpChassisFeedback.low_light = msg->low_light;
	udpChassisFeedback.reverse_light = msg->reverse_light;                 
	udpChassisFeedback.emergency_light = msg->emergency_light;
	udpChassisFeedback.foggy_light  = msg->front_foggy_light;
	                                                                            //室内灯没有
	udpChassisFeedback.air_horn = msg->horn;     //气电喇叭没有区分


	// udpChassisFeedback.ECUOnlineSta = msg->ECUOnlineSta;
	// udpChassisFeedback.TCUOnlineSta = msg->TCUOnlineSta;
	// udpChassisFeedback.EBSOnlineSta = msg->EBSOnlineSta;
	// udpChassisFeedback.EPBOnlineSta = msg->EPBOnlineSta;
	// udpChassisFeedback.DashOnlineSta = msg->DashOnlineSta;
	// udpChassisFeedback.CEPSOnlineSta = msg->CEPSOnlineSta;
	// udpChassisFeedback.BMCOnlineSta = msg->BMCOnlineSta;
	// udpChassisFeedback.WaterTemFbk = msg->WaterTemFbk;
	// udpChassisFeedback.OilPreSTA = msg->OilPreSTA;
	// udpChassisFeedback.GasHeatSTA = msg->GasHeatSTA;
	// udpChassisFeedback.FuelWaterSTA = msg->FuelWaterSTA;
	// udpChassisFeedback.AmbTemFbk = msg->AmbTemFbk;
	// udpChassisFeedback.GearShiftFbk = msg->GearShiftFbk;
	// udpChassisFeedback.ATFFbk = msg->ATFFbk;
	// udpChassisFeedback.ROTemFbk = msg->ROTemFbk;
	// udpChassisFeedback.ROStaFbk = msg->ROStaFbk;
	// udpChassisFeedback.GearBoxCurStaFbk = msg->GearBoxCurStaFbk;
	// udpChassisFeedback.TCaseCurStaFbk = msg->TCaseCurStaFbk;
	// udpChassisFeedback.TCaseGearFbk = msg->TCaseGearFbk;
	// udpChassisFeedback.TCaseTemFbk = msg->TCaseTemFbk;
	// udpChassisFeedback.Axle1TemFbk = msg->Axle1TemFbk;
	// udpChassisFeedback.Axle2TemFbk = msg->Axle2TemFbk;
	// udpChassisFeedback.Axle3TemFbk = msg->Axle3TemFbk;
	// udpChassisFeedback.Axle4TemFbk = msg->Axle4TemFbk;
	
	// udpChassisFeedback.FrontAxleStrModelSta = msg->FrontAxleStrModelSta;
	// udpChassisFeedback.RearAxleStrModelSta = msg->RearAxleStrModelSta;
	// udpChassisFeedback.Axle1StrSta = msg->Axle1StrSta;
	// udpChassisFeedback.Axle4StrSta = msg->Axle4StrSta;
	// udpChassisFeedback.Axle1RStrSta = msg->Axle1RStrSta;
	// udpChassisFeedback.SteerTorFbk = msg->SteerTorFbk;
	// udpChassisFeedback.StrFaultCode = msg->StrFaultCode;
	// udpChassisFeedback.CEPSFaultCode1 = msg->CEPSFaultCode1;
	// udpChassisFeedback.CEPSFaultCode2 = msg->CEPSFaultCode2;
	// udpChassisFeedback.RearBraPre = msg->RearBraPre;
	// udpChassisFeedback.FrontBraPre = msg->FrontBraPre;
	// udpChassisFeedback.ParkPre = msg->ParkPre;
	// udpChassisFeedback.BrakeStaFbk = msg->BrakeStaFbk;
	// udpChassisFeedback.EPBWarnLightRFbk = msg->EPBWarnLightRFbk;
	// udpChassisFeedback.EPBWarnLightYFbk = msg->EPBWarnLightYFbk;
	// udpChassisFeedback.ASRLight = msg->ASRLight;
	
	// udpChassisFeedback.ASRFun = msg->ASRFun;
	// udpChassisFeedback.ASRRun = msg->ASRRun;
	// udpChassisFeedback.EBSWarnLightRFbk = msg->EBSWarnLightRFbk;
	// udpChassisFeedback.EBSWarnLightYFbk = msg->EBSWarnLightYFbk;
	// udpChassisFeedback.electric_horn = msg->electric_horn;
	// udpChassisFeedback.brake_light = msg->brake_light;
	// udpChassisFeedback.AirBrakeLampSTA = mcallbackPlatoonStatesg->AirBrakeLampSTA;
	statusFeedbackFlag = true;
}

void CloudComm::callbackError(const errorcode_msgs::Errorcode::ConstPtr &msg){
	soft_errcode = msg -> soft_errcode;
	hard_errcode = msg -> hard_errcode;
}

void CloudComm::callbackLocalization(const localization_msgs::Localization::ConstPtr &msg){
	udpLocationFeedback.latitude = msg->original_ins.latitude;
	udpLocationFeedback.longitude = msg->original_ins.longitude;
	udpLocationFeedback.altitude = msg->original_ins.altitude;
	udpLocationFeedback.east_speed = msg->original_ins.east_speed;
	udpLocationFeedback.north_speed= msg->original_ins.north_speed;
	udpLocationFeedback.sky_speed = msg->original_ins.sky_speed;
	udpLocationFeedback.system_status = msg->original_ins.system_state;
	udpLocationFeedback.satellite_status = msg->original_ins.satellite_status;
	udpLocationFeedback.coordinate_.coordinate_system = 3;
	udpLocationFeedback.coordinate_.UTM = 1;
	udpLocationFeedback.coordinate_.sign = 0;
	udpLocationFeedback.coordinate_.x = msg->location.pose.pose.position.x;
	udpLocationFeedback.coordinate_.y = msg->location.pose.pose.position.y;
	udpLocationFeedback.coordinate_.z = msg->location.pose.pose.position.z;
	udpLocationFeedback.posture_.orientation = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2);
	// udpLocationFeedback.posture_.orientation = amathutils::getPoseYawAngle(msg->location.pose.pose);
	udpLocationFeedback.posture_.pitch = msg->original_ins.pitch;
	udpLocationFeedback.posture_.roll = msg->original_ins.roll;
	// udpLocationFeedback.origin_longitude_ = originInfo.origin_longitude;
	// udpLocationFeedback.origin_latitude_ = originInfo.origin_latitude;
	udpLocationFeedback.ins_state = msg->original_ins.ins_state;
	udpLocationFeedback.gps_state = msg->original_ins.gps_state;
    locationFeedbackFlag = true;
	
}

void CloudComm::callbackOrigin(const geometry_msgs::Point::ConstPtr &msg){
	udpLocationFeedback.origin_latitude_ = msg->x;
	udpLocationFeedback.origin_longitude_ = msg->y;
}

void CloudComm::callbackPlatoonState(const platoon_msgs::PlatoonMember::ConstPtr &msg){
    UdpPlatoonStateFeedback udpPlatoonStateFeedback;
	udpPlatoonStateFeedback.spacing = msg->spacing_distance;
	udpPlatoonStateFeedback.x = msg->latLon.x;
	udpPlatoonStateFeedback.y = msg->latLon.y;
	udpPlatoonStateFeedback.orientation = msg->heading;
	udpPlatoonStateFeedback.velocity = msg->linear_velocity;
	udpPlatoonStateFeedback.acceleration = msg->linear_acceleration;
	udpPlatoonStateFeedback.num = msg->num;
	udpPlatoonStateFeedback.role = msg->role;
	udpPlatoonStateFeedback.index = msg->index;
	udpPlatoonStateFeedback.state = msg->type;
	udpPlatoonStateFeedback.gear = msg->gear;
	udpPlatoonStateFeedback.driving_mode = msg->driving_mode;
	//numPlatoon.vehicle_num = msg->num;
	platoonFeedBack[msg->num] = udpPlatoonStateFeedback;
	infos[msg->num] = true;
}

void CloudComm::callbackRequest(const std_msgs::UInt8::ConstPtr &msg){
	
	if(msg->data == 1){
		std::cout << "hahahaha" << std::endl;
		std::string pkg_launch_dir = ros::package::getPath("launch_node");
		string yaml_location = pkg_launch_dir + std::string("/param/global/global_config.yaml");
		string map_location = pkg_launch_dir + std::string("/data/cloudmap.osm") + std::string(";") + yaml_location;
		MsgHeader msgHeader;
		MsgEnd msgEnd;
		msgHeader.msg_id  = 0x9040;	
		msgHeader.msg_length = map_location.length();	
		msgHeader.msg_num = msg_num_++;
		msgHeader.msg_time = Acquire24AbsTime();
		msgHeader.msg_vehicle_num = numInfo.vehicle_num;
		msgHeader.msg_source_add = inet_addr(netInfo.host_ip.c_str()) ;
		msgHeader.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
		msgHeader.msg_flag = 0x00FF ;
		
		memcpy(sendBuf,&msgHeader,sizeof(msgHeader));
		memcpy(sendBuf+sizeof(msgHeader),map_location.c_str(),map_location.length());
		memcpy(sendBuf+sizeof(msgHeader)+map_location.length(),&msgEnd,sizeof(msgEnd));
		udp_send_ptr->send(sendBuf, sizeof(MsgHeader)+map_location.length()+sizeof(MsgEnd), netInfo.cloud_ip, netInfo.cloud_port);

		std::cout << "地图存放位置： " << map_location << std::endl;
		std::cout << "配置文件存放位置： " << yaml_location << std::endl;


	}
}

// void CloudComm::callbackPath(const planning_msgs::TrajectoryPointArray::ConstPtr &msg){
// 	FullUdpPathInfo fullUdpPathInfo_;
// 	fullUdpPathInfo_.msgHeader_.msg_id  = 0x80B0;
// 	fullUdpPathInfo_.msgHeader_.msg_length = sizeof(FullUdpPathInfo);
// 	fullUdpPathInfo_.msgHeader_.msg_time = Acquire24AbsTime();
// 	fullUdpPathInfo_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
// 	fullUdpPathInfo_.msgHeader_.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
// 	fullUdpPathInfo_.msgHeader_.msg_flag = 0x00FF;
// 	fullUdpPathInfo_.msgHeader_.msg_num++;
// 	fullUdpPathInfo_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpPathInfo_,sizeof(fullUdpPathInfo_)-sizeof(fullUdpPathInfo_.msgEnd_));

// 	std::vector<PathPoint> pathList;
// 	std::vector<PathPoint> pathList_back;
// 	pathList.clear();
// 	PathPoint pathPoint;
// 	int loop = 0;
// 	memset(&pathPoint,0,sizeof(PathPoint));
// 	for(int i = 0; i < msg -> points.size(); i++){
// 		pathPoint.coordinate = 3;
// 		pathPoint.x = msg -> points.at(i).x;
// 		pathPoint.y = msg -> points.at(i).y;
// 		pathPoint.z = msg -> points.at(i).z;
// 		pathList.push_back(pathPoint);

// 	}
	
// 	if(20 >= msg -> points.size()){
// 		udpPathInfo.path_type = 1;
// 		udpPathInfo.package_id = 0;
// 		udpPathInfo.package_sum = 1;
// 		udpPathInfo.package_byte = msg -> points.size() * sizeof(PathPoint);
// 		udpPathInfo.package_num = 1;
// 		udpPathInfo.current_package_byte = msg -> points.size() * sizeof(PathPoint);
// 		for(int i = 0; i < pathList.size(); i++){
// 			// udpPathInfo.path_point_list[i] = pathList.at(i);
// 			pathList_back.push_back(pathList.at(i));

// 		}
// 		// fullUdpPathInfo_.udpPathInfo_ = udpPathInfo;
// 		memcpy(sendBuf,&fullUdpPathInfo_.msgHeader_,sizeof(fullUdpPathInfo_.msgHeader_));
// 		memcpy(sendBuf+sizeof(MsgHeader),pathList_back.data(),pathList_back.size() * sizeof(PathPoint));
// 		memcpy(sendBuf+sizeof(MsgHeader)+pathList_back.size() * sizeof(PathPoint),&fullUdpPathInfo_.msgEnd_,sizeof(MsgEnd));
// 		udp_send_ptr->send(sendBuf, sizeof(MsgHeader)+pathList_back.size() * sizeof(PathPoint)+sizeof(MsgEnd), netInfo.cloud_ip, netInfo.cloud_port);
// 		pathList_back.clear();
// 	}
// 	else{
// 		udpPathInfo.path_type = 0x01;
// 		udpPathInfo.package_sum = pathList.size() / 20 + ((pathList.size() % 20 == 0) ? 0:1);
// 		std::cout << "包数：" << udpPathInfo.package_sum << std::endl;
// 		udpPathInfo.package_byte = pathList.size() * sizeof(PathPoint);
// 		udpPathInfo.package_num = 1;
// 		for(int i = 0; i < udpPathInfo.package_sum; i++)
// 		{
			
// 			if(udpPathInfo.package_num != udpPathInfo.package_sum)
// 			{
// 				int k = 0;
// 				memset(udpPathInfo.path_point_list,0,sizeof(udpPathInfo.path_point_list));
// 				for(int j = 20 * (udpPathInfo.package_num - 1); j < 20 * (udpPathInfo.package_num - 1) + 20; j++)
// 				{
// 					udpPathInfo.path_point_list[k] = pathList.at(j);
// 					cout << "XXXXXXXX: " << udpPathInfo.path_point_list[k].x << endl;
// 					cout << "YYYYYZZZ: " << udpPathInfo.path_point_list[k].y << endl;
// 					cout << "ZZZZZZZZ: " << udpPathInfo.path_point_list[k].z << endl;	
// 					k++;
// 					// std::cout << "lalala" << udpPathInfo.path_point_list[k].x << std::endl;
// 				}
// 				udpPathInfo.current_package_byte = 20*sizeof(PathPoint);
// 				udpPathInfo.package_id = 0x02;
// 				if(udpPathInfo.package_num == 1)
// 				{
// 					udpPathInfo.package_id = 0x01;
// 				}
// 				udpPathInfo.package_num++;
// 				// for(int p = 0; i < 20; p++){
// 				// 	cout << "XXXXXXXX: " << udpPathInfo.path_point_list[p].x << endl;
// 				// 	cout << "YYYYYZZZ: " << udpPathInfo.path_point_list[p].y << endl;
// 				// 	cout << "ZZZZZZZZ: " << udpPathInfo.path_point_list[p].z << endl;
// 				// }
// 				fullUdpPathInfo_.udpPathInfo_ = udpPathInfo;
// 				memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
// 				udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
// 				std::cout << "enter thisss1 :" << std::endl;
// 			}
// 			else
// 			{
// 				memset(udpPathInfo.path_point_list,0,sizeof(udpPathInfo.path_point_list));
// 				for(unsigned int k = 20 * (udpPathInfo.package_num-1);k < pathList.size(); k++)
// 				{
// 					// udpPathInfo.path_point_list[k] = pathList.at(k);
// 					pathList_back.push_back(pathList.at(k));
// 				}
// 				udpPathInfo.current_package_byte = udpPathInfo.package_byte - 20 * sizeof(PathPoint) * (udpPathInfo.package_sum-1);
// 				udpPathInfo.package_id = 0x03;
// 				memcpy(sendBuf,&fullUdpPathInfo_.msgHeader_,sizeof(fullUdpPathInfo_.msgHeader_));
// 				memcpy(sendBuf+sizeof(MsgHeader),pathList_back.data(),pathList_back.size() * sizeof(PathPoint));
// 				memcpy(sendBuf+sizeof(MsgHeader)+pathList_back.size() * sizeof(PathPoint),&fullUdpPathInfo_.msgEnd_,sizeof(MsgEnd));
// 				udp_send_ptr->send(sendBuf, sizeof(MsgHeader)+pathList_back.size() * sizeof(PathPoint)+sizeof(MsgEnd), netInfo.cloud_ip, netInfo.cloud_port);
				
// 				pathList_back.clear();
// 				std::cout << "enter thisss2 :" << std::endl;
// 			}
// 		}
// 	}
// }

void CloudComm::callbackPath(const planning_msgs::TrajectoryPointArray::ConstPtr &msg){
	FullUdpPathInfo fullUdpPathInfo_;
	fullUdpPathInfo_.msgHeader_.msg_id  = 0x80B0;
	fullUdpPathInfo_.msgHeader_.msg_length = sizeof(FullUdpPathInfo);
	fullUdpPathInfo_.msgHeader_.msg_time = Acquire24AbsTime();
	fullUdpPathInfo_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
	fullUdpPathInfo_.msgHeader_.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
	fullUdpPathInfo_.msgHeader_.msg_flag = 0x00FF;
	fullUdpPathInfo_.msgHeader_.msg_num++;
	fullUdpPathInfo_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpPathInfo_,sizeof(fullUdpPathInfo_)-sizeof(fullUdpPathInfo_.msgEnd_));

	std::vector<PathPoint> pathList;
	std::vector<PathPoint> pathList_back;
	pathList.clear();
	PathPoint pathPoint;
	int loop = 0;
	memset(&pathPoint,0,sizeof(PathPoint));
	for(int i = 0; i < msg -> points.size(); i++){
		pathPoint.coordinate = 3;
		pathPoint.x = msg -> points.at(i).x;
		pathPoint.y = msg -> points.at(i).y;
		pathPoint.z = msg -> points.at(i).z;
		pathList.push_back(pathPoint);

	}
	
	if(20 >= msg -> points.size()){
		udpPathInfo.path_type = 1;
		udpPathInfo.package_id = 0;
		udpPathInfo.package_sum = 1;
		udpPathInfo.package_byte = msg -> points.size() * sizeof(PathPoint);
		udpPathInfo.package_num = 1;
		udpPathInfo.current_package_byte = msg -> points.size() * sizeof(PathPoint);

		for(int j = 0; j < msg -> points.size(); j++)
		{
			udpPathInfo.path_point_list[j] = pathList.at(j);	
		}
		fullUdpPathInfo_.udpPathInfo_ = udpPathInfo;
		memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
		udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
		// pathList_back.clear();
	}
	else{
		udpPathInfo.path_type = 0x01;
		udpPathInfo.package_sum = pathList.size() / 20 + ((pathList.size() % 20 == 0) ? 0:1);
		std::cout << "包数：" << udpPathInfo.package_sum << std::endl;
		udpPathInfo.package_byte = pathList.size() * sizeof(PathPoint);
		udpPathInfo.package_num = 1;
		for(int i = 0; i < udpPathInfo.package_sum; i++)
		{
			
			if(udpPathInfo.package_num != udpPathInfo.package_sum)
			{
				memset(udpPathInfo.path_point_list,0,sizeof(udpPathInfo.path_point_list));
				int p = 0;
				for(int j = 20 * (udpPathInfo.package_num - 1); j < 20 * (udpPathInfo.package_num - 1) + 20; j++)
				{
					udpPathInfo.path_point_list[p] = pathList.at(j);	
					p++;
				}
				udpPathInfo.current_package_byte = 20*sizeof(PathPoint);
				udpPathInfo.package_id = 0x02;
				if(udpPathInfo.package_num == 1)
				{
					udpPathInfo.package_id = 0x01;
				}
				udpPathInfo.package_num++;
				fullUdpPathInfo_.udpPathInfo_ = udpPathInfo;
				memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
				udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
				// std::cout << "enter thisss1 :" << std::endl;
			}
			else
			{
				memset(udpPathInfo.path_point_list,0,sizeof(udpPathInfo.path_point_list));
				int q = 0;
				for(unsigned int k = 20 * (udpPathInfo.package_num-1);k < pathList.size(); k++)
				{
					// udpPathInfo.path_point_list[k] = pathList.at(k);
					udpPathInfo.path_point_list[q] = pathList.at(k);
					q++;
				}
				udpPathInfo.current_package_byte = udpPathInfo.package_byte - 20 * sizeof(PathPoint) * (udpPathInfo.package_sum-1);
				udpPathInfo.package_id = 0x03;
				fullUdpPathInfo_.udpPathInfo_ = udpPathInfo;
				memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
				udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
				
				// std::cout << "enter thisss2 :" << std::endl;
			}
		}
	}
}

void CloudComm::callbackReference(const planning_msgs::TrajectoryPointArray::ConstPtr &msg){
	FullUdpPathInfo fullUdpPathInfo_;
	fullUdpPathInfo_.msgHeader_.msg_id  = 0x9080;
	fullUdpPathInfo_.msgHeader_.msg_length = sizeof(FullUdpPathInfo);
	fullUdpPathInfo_.msgHeader_.msg_time = Acquire24AbsTime();
	fullUdpPathInfo_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
	fullUdpPathInfo_.msgHeader_.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
	fullUdpPathInfo_.msgHeader_.msg_flag = 0x00FF;
	fullUdpPathInfo_.msgHeader_.msg_num++;
	fullUdpPathInfo_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpPathInfo_,sizeof(fullUdpPathInfo_)-sizeof(fullUdpPathInfo_.msgEnd_));

	std::vector<PathPoint> pathList;
	std::vector<PathPoint> pathList_back;
	pathList.clear();
	PathPoint pathPoint;
	int loop = 0;
	memset(&pathPoint,0,sizeof(PathPoint));
	for(int i = 0; i < msg -> points.size(); i++){
		pathPoint.coordinate = 3;
		pathPoint.x = msg -> points.at(i).x;
		pathPoint.y = msg -> points.at(i).y;
		pathPoint.z = msg -> points.at(i).z;
		pathList.push_back(pathPoint);

	}
	
	if(20 >= msg -> points.size()){
		udpReferenceInfo.path_type = 1;
		udpReferenceInfo.package_id = 0;
		udpReferenceInfo.package_sum = 1;
		udpReferenceInfo.package_byte = msg -> points.size() * sizeof(PathPoint);
		udpReferenceInfo.package_num = 1;
		udpReferenceInfo.current_package_byte = msg -> points.size() * sizeof(PathPoint);

		for(int j = 0; j < msg -> points.size(); j++)
		{
			udpReferenceInfo.path_point_list[j] = pathList.at(j);	
		}
		fullUdpPathInfo_.udpPathInfo_ = udpReferenceInfo;
		memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
		udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
		// pathList_back.clear();
	}
	else{
		udpReferenceInfo.path_type = 0x01;
		udpReferenceInfo.package_sum = pathList.size() / 20 + ((pathList.size() % 20 == 0) ? 0:1);
		// std::cout << "包数：" << udpReferenceInfo.package_sum << std::endl;
		udpReferenceInfo.package_byte = pathList.size() * sizeof(PathPoint);
		udpReferenceInfo.package_num = 1;
		for(int i = 0; i < udpReferenceInfo.package_sum; i++)
		{
			
			if(udpReferenceInfo.package_num != udpReferenceInfo.package_sum)
			{
				memset(udpReferenceInfo.path_point_list,0,sizeof(udpReferenceInfo.path_point_list));
				int p = 0;
				for(int j = 20 * (udpReferenceInfo.package_num - 1); j < 20 * (udpReferenceInfo.package_num - 1) + 20; j++)
				{
					udpReferenceInfo.path_point_list[p] = pathList.at(j);	
					p++;
				}
				udpReferenceInfo.current_package_byte = 20*sizeof(PathPoint);
				udpReferenceInfo.package_id = 0x02;
				if(udpReferenceInfo.package_num == 1)
				{
					udpReferenceInfo.package_id = 0x01;
				}
				udpReferenceInfo.package_num++;
				fullUdpPathInfo_.udpPathInfo_ = udpReferenceInfo;
				memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
				udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
				// std::cout << "enter thisss1 :" << std::endl;
			}
			else
			{
				memset(udpReferenceInfo.path_point_list,0,sizeof(udpReferenceInfo.path_point_list));
				int q = 0;
				for(unsigned int k = 20 * (udpReferenceInfo.package_num-1);k < pathList.size(); k++)
				{
					// udpPathInfo.path_point_list[k] = pathList.at(k);
					udpReferenceInfo.path_point_list[q] = pathList.at(k);
					q++;
				}
				udpReferenceInfo.current_package_byte = udpReferenceInfo.package_byte - 20 * sizeof(PathPoint) * (udpReferenceInfo.package_sum-1);
				udpReferenceInfo.package_id = 0x03;
				fullUdpPathInfo_.udpPathInfo_ = udpReferenceInfo;
				memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
				udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
				
				// std::cout << "enter thisss2 :" << std::endl;
			}
		}
	}
}

void CloudComm::callbackTrajectory(const planning_msgs::TrajectoryPointArray::ConstPtr &msg){
	FullUdpPathInfo fullUdpPathInfo_;
	fullUdpPathInfo_.msgHeader_.msg_id  = 0x9090;
	fullUdpPathInfo_.msgHeader_.msg_length = sizeof(FullUdpPathInfo);
	fullUdpPathInfo_.msgHeader_.msg_time = Acquire24AbsTime();
	fullUdpPathInfo_.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
	fullUdpPathInfo_.msgHeader_.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
	fullUdpPathInfo_.msgHeader_.msg_flag = 0x00FF;
	fullUdpPathInfo_.msgHeader_.msg_num++;
	fullUdpPathInfo_.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullUdpPathInfo_,sizeof(fullUdpPathInfo_)-sizeof(fullUdpPathInfo_.msgEnd_));

	std::vector<PathPoint> pathList;
	std::vector<PathPoint> pathList_back;
	pathList.clear();
	PathPoint pathPoint;
	int loop = 0;
	memset(&pathPoint,0,sizeof(PathPoint));
	for(int i = 0; i < msg -> points.size(); i++){
		pathPoint.coordinate = 3;
		pathPoint.x = msg -> points.at(i).x;
		pathPoint.y = msg -> points.at(i).y;
		pathPoint.z = msg -> points.at(i).z;
		pathList.push_back(pathPoint);

	}
	
	if(20 >= msg -> points.size()){
		udpTrajectoryInfo.path_type = 1;
		udpTrajectoryInfo.package_id = 0;
		udpTrajectoryInfo.package_sum = 1;
		udpTrajectoryInfo.package_byte = msg -> points.size() * sizeof(PathPoint);
		udpTrajectoryInfo.package_num = 1;
		udpTrajectoryInfo.current_package_byte = msg -> points.size() * sizeof(PathPoint);

		for(int j = 0; j < msg -> points.size(); j++)
		{
			udpTrajectoryInfo.path_point_list[j] = pathList.at(j);	
		}
		fullUdpPathInfo_.udpPathInfo_ = udpTrajectoryInfo;
		memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
		udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
		// pathList_back.clear();
	}
	else{
		udpTrajectoryInfo.path_type = 0x01;
		udpTrajectoryInfo.package_sum = pathList.size() / 20 + ((pathList.size() % 20 == 0) ? 0:1);
		// std::cout << "包数：" << udpTrajectoryInfo.package_sum << std::endl;
		udpTrajectoryInfo.package_byte = pathList.size() * sizeof(PathPoint);
		udpTrajectoryInfo.package_num = 1;
		for(int i = 0; i < udpTrajectoryInfo.package_sum; i++)
		{
			
			if(udpTrajectoryInfo.package_num != udpTrajectoryInfo.package_sum)
			{
				memset(udpTrajectoryInfo.path_point_list,0,sizeof(udpTrajectoryInfo.path_point_list));
				int p = 0;
				for(int j = 20 * (udpTrajectoryInfo.package_num - 1); j < 20 * (udpTrajectoryInfo.package_num - 1) + 20; j++)
				{
					udpTrajectoryInfo.path_point_list[p] = pathList.at(j);	
					p++;
				}
				udpTrajectoryInfo.current_package_byte = 20*sizeof(PathPoint);
				udpTrajectoryInfo.package_id = 0x02;
				if(udpTrajectoryInfo.package_num == 1)
				{
					udpTrajectoryInfo.package_id = 0x01;
				}
				udpTrajectoryInfo.package_num++;
				fullUdpPathInfo_.udpPathInfo_ = udpTrajectoryInfo;
				memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
				udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
				// std::cout << "enter thisss1 :" << std::endl;
			}
			else
			{
				memset(udpTrajectoryInfo.path_point_list,0,sizeof(udpTrajectoryInfo.path_point_list));
				int q = 0;
				for(unsigned int k = 20 * (udpTrajectoryInfo.package_num-1);k < pathList.size(); k++)
				{
					// udpPathInfo.path_point_list[k] = pathList.at(k);
					udpTrajectoryInfo.path_point_list[q] = pathList.at(k);
					q++;
				}
				udpTrajectoryInfo.current_package_byte = udpTrajectoryInfo.package_byte - 20 * sizeof(PathPoint) * (udpTrajectoryInfo.package_sum-1);
				udpTrajectoryInfo.package_id = 0x03;
				fullUdpPathInfo_.udpPathInfo_ = udpTrajectoryInfo;
				memcpy(sendBuf,&fullUdpPathInfo_,sizeof(fullUdpPathInfo_));
				udp_send_ptr->send(sendBuf, sizeof(FullUdpPathInfo), netInfo.cloud_ip, netInfo.cloud_port);
				
				// std::cout << "enter thisss2 :" << std::endl;
			}
		}
	}
}

// void CloudComm::callbackObstacle(const perception_msgs::PredictionObstacles::ConstPtr &msg){
	
// 	MsgHeader msgHeader;
// 	MsgEnd msgEnd;
// 	UdpObstacleFeedback udpObstacleFeedback;
// 	vector<unsigned char> obs_if_static;
// 	vector<unsigned short> obs_id;
// 	vector<unsigned char> obs_type;
// 	vector<int> obs_key;
// 	vector<XYZ> obs_value;
// 	XYZ xyz_tmp;
// 	int point_num;
// 	for(int i = 0; i < msg -> prediction_obstacles.size(); i++){
// 		obs_if_static.push_back(msg -> prediction_obstacles.at(i).is_static);
// 		obs_id.push_back(msg -> prediction_obstacles.at(i).perception_obstacle.id);
		
// 		obs_type.push_back(msg -> prediction_obstacles.at(i).perception_obstacle.type);
// 		point_num = ((msg -> prediction_obstacles.at(i).perception_obstacle.polygon.points.size() > 8) ? 8:msg -> prediction_obstacles.at(i).perception_obstacle.polygon.points.size());
// 		obs_key.push_back(point_num);

// 		for(int j = 0; j < (int)obs_key.at(i); j++){

// 			if(j >= 8){
// 				break;
// 			}
// 			xyz_tmp.x = msg -> prediction_obstacles.at(i).perception_obstacle.polygon.points.at(j).x;
// 			xyz_tmp.y = msg -> prediction_obstacles.at(i).perception_obstacle.polygon.points.at(j).y;
// 			xyz_tmp.z = msg -> prediction_obstacles.at(i).perception_obstacle.polygon.points.at(j).z;
// 			obs_value.push_back(xyz_tmp);
// 		}
// 	}
// 	vector<int> obs_acc_key(obs_key.size());
// 	std::partial_sum(obs_key.begin(),obs_key.end(),obs_acc_key.begin());
// 	msgHeader.msg_id  = 0xE050;	
// 	msgHeader.msg_length = 40 + 10 * (5 + 6*obs_key.size());	
// 	msgHeader.msg_num = msg_num_++;
// 	msgHeader.msg_time = Acquire24AbsTime();
// 	msgHeader.msg_vehicle_num = numInfo.vehicle_num;
// 	msgHeader.msg_source_add = inet_addr(netInfo.host_ip.c_str()) ;
// 	msgHeader.msg_destination_add = inet_addr(netInfo.multicast_ip.c_str()) ;
// 	msgHeader.msg_flag = 0x00FF ;


	
// 	udpObstacleFeedback.bag_byte = obs_if_static.size() * sizeof(unsigned char) + obs_id.size() * sizeof(unsigned short) + obs_type.size() * sizeof(unsigned char) + obs_key.size() * sizeof(unsigned char) + obs_value.size() * sizeof(XYZ);
	
// 	if(10>=msg -> prediction_obstacles.size()){
// 		udpObstacleFeedback.bag_type = 0x00;
// 		udpObstacleFeedback.bag_sum = 1;
// 		udpObstacleFeedback.bag_num = 0;
// 		udpObstacleFeedback.cur_obstacle_sum = obs_key.size();
// 		int count = 0;
// 		memcpy(sendBuf,&msgHeader,sizeof(msgHeader));
// 		memcpy(sendBuf+sizeof(msgHeader),&udpObstacleFeedback,sizeof(udpObstacleFeedback));
// 		count = sizeof(msgHeader) + sizeof(udpObstacleFeedback);
		
// 		for(int i = 0; i < udpObstacleFeedback.cur_obstacle_sum; i++){
// 			memcpy(sendBuf+count,&obs_if_static.at(i),sizeof(unsigned char));
// 			memcpy(sendBuf+count+1,&obs_id.at(i),sizeof(unsigned short));
// 			memcpy(sendBuf+count+3,&obs_type.at(i),sizeof(unsigned char));
// 			memcpy(sendBuf+count+4,&obs_key.at(i),sizeof(unsigned char));
// 			for(int j = 0; j < obs_key.at(i); j++){
// 				if(j >= 8){
// 					break;
// 				}
// 				memcpy(sendBuf+count+5+sizeof(XYZ)*j,&obs_value.at(obs_acc_key.at(i)-obs_key.at(i)),sizeof(XYZ));
// 			}
// 			obs_key.at(i) = ((obs_key.at(i) > 8) ? 8 : obs_key.at(i));
// 			count += 5+ sizeof(XYZ)*obs_key.at(i);
// 		}
// 		memcpy(sendBuf+count,&msgEnd,sizeof(msgEnd));
// 		udp_send_ptr->send(sendBuf, count+sizeof(msgEnd), netInfo.multicast_ip, netInfo.cloud_port);
// 		cout << "send_ptr, size<10" << endl;
// 	}
// 	else{
		
// 		udpObstacleFeedback.bag_sum = (obs_key.size()) / 10 + ((obs_key.size()%10==0)?0:1);
// 		udpObstacleFeedback.bag_num = 1;
// 		for(int s = 0; s < udpObstacleFeedback.bag_sum; s++){
			
// 			if(udpObstacleFeedback.bag_num != udpObstacleFeedback.bag_sum){
// 				udpObstacleFeedback.cur_obstacle_sum = 10;
// 				udpObstacleFeedback.bag_type = ((s==0)?0x01:0x02);
// 				int count = 0;
// 				memcpy(sendBuf,&msgHeader,sizeof(msgHeader));
// 				memcpy(sendBuf+sizeof(msgHeader),&udpObstacleFeedback,sizeof(udpObstacleFeedback));
// 				count += sizeof(msgHeader) + sizeof(udpObstacleFeedback);
// 				for(int i = s*udpObstacleFeedback.cur_obstacle_sum; i < (s+1)*udpObstacleFeedback.cur_obstacle_sum; i++){
					
// 					memcpy(sendBuf+count,&obs_if_static.at(i),sizeof(unsigned char));
// 					memcpy(sendBuf+count+1,&obs_id.at(i),sizeof(unsigned short));
// 					memcpy(sendBuf+count+3,&obs_type.at(i),sizeof(unsigned char));
// 					memcpy(sendBuf+count+4,&obs_key.at(i),sizeof(unsigned char));
// 					for(int j = 0; j < obs_key.at(i); j++){
// 						if(j >= 8){
// 							break;
// 						}
// 						memcpy(sendBuf+count+5+sizeof(XYZ)*j,&obs_value.at(j),sizeof(XYZ));
// 					}
// 					obs_key.at(i) = ((obs_key.at(i) > 8) ? 8 : obs_key.at(i));
// 					count += 5+ sizeof(XYZ)*obs_key.at(i);
// 				}	
// 				memcpy(sendBuf+count,&msgEnd,sizeof(msgEnd));
// 				udp_send_ptr->send(sendBuf, count+sizeof(msgEnd), netInfo.multicast_ip, netInfo.cloud_port);
// 				udpObstacleFeedback.bag_num++;
// 			}
// 			else{
// 				udpObstacleFeedback.cur_obstacle_sum = obs_key.size() - 10*udpObstacleFeedback.bag_num;
// 				udpObstacleFeedback.bag_type = 0x03;
// 				int count = 0;
// 				memcpy(sendBuf,&msgHeader,sizeof(msgHeader));
// 				memcpy(sendBuf+sizeof(msgHeader),&udpObstacleFeedback,sizeof(udpObstacleFeedback));
// 				count += sizeof(msgHeader) + sizeof(udpObstacleFeedback);
// 				for(int i = 10*(udpObstacleFeedback.bag_sum-1); i < obs_key.size(); i++){
// 					memcpy(sendBuf+count,&obs_if_static.at(i),sizeof(unsigned char));
// 					memcpy(sendBuf+count+1,&obs_id.at(i),sizeof(unsigned short));
// 					memcpy(sendBuf+count+3,&obs_type.at(i),sizeof(unsigned char));
// 					memcpy(sendBuf+count+4,&obs_key.at(i),sizeof(unsigned char));
// 					for(int j = 0; j < obs_key.at(i); j++){
// 						if(j >= 8){
// 							break;
// 						}
// 						memcpy(sendBuf+count+5+sizeof(XYZ)*j,&obs_value.at(j),sizeof(XYZ));
// 					}
// 					obs_key.at(i) = ((obs_key.at(i) > 8) ? 8 : obs_key.at(i));
// 					count += 5+ sizeof(XYZ)*obs_key.at(i);
// 				}	
// 				memcpy(sendBuf+count,&msgEnd,sizeof(msgEnd));
// 				udp_send_ptr->send(sendBuf, count+sizeof(msgEnd), netInfo.multicast_ip, netInfo.cloud_port);
				
// 			}
// 		}
// 	}

// }

void CloudComm::callbackObstacle(const perception_msgs::PredictionObstacles::ConstPtr &msg){
	FullObstaclesInfo fullObstaclesInfo;
	fullObstaclesInfo.msgHeader_.msg_id  = 0xE050;
	fullObstaclesInfo.msgHeader_.msg_length = sizeof(FullObstaclesInfo);
	fullObstaclesInfo.msgHeader_.msg_time = Acquire24AbsTime();
	fullObstaclesInfo.msgHeader_.msg_source_add = inet_addr(netInfo.host_ip.c_str());
	fullObstaclesInfo.msgHeader_.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
	fullObstaclesInfo.msgHeader_.msg_flag = 0x00FF;
	fullObstaclesInfo.msgHeader_.msg_num++;
	fullObstaclesInfo.msgEnd_.msg_checksum = getCheckSum((uint8 *)&fullObstaclesInfo,sizeof(fullObstaclesInfo)-sizeof(fullObstaclesInfo.msgEnd_));
	int obs_nums_ = msg->prediction_obstacles.size();

	if(10 >= obs_nums_){
		fullObstaclesInfo.udpObstaclesInfo_.bag_type = 0x00;
		fullObstaclesInfo.udpObstaclesInfo_.bag_sum = 1;
		fullObstaclesInfo.udpObstaclesInfo_.bag_num = 0;
		fullObstaclesInfo.udpObstaclesInfo_.cur_obstacle_sum = obs_nums_;
		for(int i=0; i<obs_nums_; i++){
			int obsPoints_nums = msg->prediction_obstacles[i].perception_obstacle.polygon.points.size();
			fullObstaclesInfo.udpObstaclesInfo_.obstacleData[i].type_status = msg->prediction_obstacles[i].is_static;
			fullObstaclesInfo.udpObstaclesInfo_.obstacleData[i].obstaclePoints.nums = obsPoints_nums;
			for(int j=0; j<obsPoints_nums; j++){
				fullObstaclesInfo.udpObstaclesInfo_.obstacleData[i].obstaclePoints.points[j].x = msg->prediction_obstacles[i].perception_obstacle.polygon.points[j].x;
				fullObstaclesInfo.udpObstaclesInfo_.obstacleData[i].obstaclePoints.points[j].y = msg->prediction_obstacles[i].perception_obstacle.polygon.points[j].y;
				fullObstaclesInfo.udpObstaclesInfo_.obstacleData[i].obstaclePoints.points[j].z = msg->prediction_obstacles[i].perception_obstacle.polygon.points[j].z;
			}
		}
		memcpy(sendBuf,&fullObstaclesInfo,sizeof(fullObstaclesInfo));
		udp_send_ptr->send(sendBuf, sizeof(FullObstaclesInfo), netInfo.cloud_ip, netInfo.cloud_port);

	}
	else{
		fullObstaclesInfo.udpObstaclesInfo_.bag_sum = obs_nums_/10 + (obs_nums_%10 == 0 ? 0 : 1);
		fullObstaclesInfo.udpObstaclesInfo_.bag_num = 1;
		for(int i=0; i<fullObstaclesInfo.udpObstaclesInfo_.bag_sum; i++){
			if(fullObstaclesInfo.udpObstaclesInfo_.bag_num != fullObstaclesInfo.udpObstaclesInfo_.bag_sum){
				fullObstaclesInfo.udpObstaclesInfo_.bag_type = 0x01;
				fullObstaclesInfo.udpObstaclesInfo_.cur_obstacle_sum = 10;
				for(int j=0; j<10; j++){
					int obsPoints_nums = msg->prediction_obstacles[10*i+j].perception_obstacle.polygon.points.size();
					fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].type_status = msg->prediction_obstacles[10*i+j].is_static;
					fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].obstaclePoints.nums = obsPoints_nums;
					for(int k=0;k<obsPoints_nums; k++){
						fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].obstaclePoints.points[k].x = msg->prediction_obstacles[10*i+j].perception_obstacle.polygon.points[k].x;
						fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].obstaclePoints.points[k].y = msg->prediction_obstacles[10*i+j].perception_obstacle.polygon.points[k].y;
						fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].obstaclePoints.points[k].z = msg->prediction_obstacles[10*i+j].perception_obstacle.polygon.points[k].z;
					}

				}
				if(fullObstaclesInfo.udpObstaclesInfo_.bag_num >=2){
					fullObstaclesInfo.udpObstaclesInfo_.bag_type = 0x02;
				}
				memcpy(sendBuf,&fullObstaclesInfo,sizeof(fullObstaclesInfo));
				udp_send_ptr->send(sendBuf, sizeof(FullObstaclesInfo), netInfo.cloud_ip, netInfo.cloud_port);
				fullObstaclesInfo.udpObstaclesInfo_.bag_num++;

			}
			else{
				fullObstaclesInfo.udpObstaclesInfo_.bag_type = 0x03;
				fullObstaclesInfo.udpObstaclesInfo_.cur_obstacle_sum = obs_nums_ - 10*(fullObstaclesInfo.udpObstaclesInfo_.bag_sum - 1);
				for(int j=0; j<fullObstaclesInfo.udpObstaclesInfo_.cur_obstacle_sum;j++){
					int obs_num = 10*(fullObstaclesInfo.udpObstaclesInfo_.bag_sum - 1)+j;
					int obsPoints_nums = msg->prediction_obstacles[obs_num].perception_obstacle.polygon.points.size();
					fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].type_status = msg->prediction_obstacles[obs_num].is_static;
					fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].obstaclePoints.nums = obsPoints_nums;
					for(int k=0; k<obsPoints_nums; k++){
						fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].obstaclePoints.points[k].x = msg->prediction_obstacles[obs_num].perception_obstacle.polygon.points[k].x;
						fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].obstaclePoints.points[k].y = msg->prediction_obstacles[obs_num].perception_obstacle.polygon.points[k].y;
						fullObstaclesInfo.udpObstaclesInfo_.obstacleData[j].obstaclePoints.points[k].z = msg->prediction_obstacles[obs_num].perception_obstacle.polygon.points[k].z;
					}
				}
				memcpy(sendBuf,&fullObstaclesInfo,sizeof(fullObstaclesInfo));
				udp_send_ptr->send(sendBuf, sizeof(FullObstaclesInfo), netInfo.cloud_ip, netInfo.cloud_port);

			}

		}
	}
}


void CloudComm::callbackConfig(const configuration_msgs::ConfigurationUp::ConstPtr &msg){
	udpConfigFeedback.config_type = msg ->config_type;
	udpConfigFeedback.lead_vehicle = msg ->lead_vehicle;
	udpConfigFeedback.follow_type = msg ->follow_type;
	udpConfigFeedback.config_load = msg ->config_load;
	udpConfigFeedback.config_speed = msg ->config_speed;
	udpConfigFeedback.config_space = msg ->config_space;
	udpConfigFeedback.config_status = msg ->config_status;
	configFeedbackFlag = true;
}


	


driver_msgs::PowerCmd CloudComm::udpCmd2MsgPower(const UdpControlCommand &udpControlCommand)
{
	driver_msgs::PowerCmd msgPowerCmd_;
	
	if (true==udpControlCommand.power_flag)
		{	
			msgPowerCmd_.engine_on_off=1;
		}
	if (false==udpControlCommand.power_flag)
		{	
			msgPowerCmd_.engine_on_off=0;
		}
	return msgPowerCmd_;
}



driver_msgs::GearCmd CloudComm::udpCmd2MsgGear(const UdpControlCommand &udpControlCommand,driver_msgs::ParkingBrakeCmd &msgParkingcmd)
{
	driver_msgs::GearCmd msgGearCmd_;
	
	if(0x00==udpControlCommand.gear_position)
		{
			msgGearCmd_.gear_location=0;
			msgParkingcmd.parking_brake=1;

		}
	else if(0x01==udpControlCommand.gear_position)
		{
			msgParkingcmd.parking_brake=0;
			msgGearCmd_.gear_location=7;
		}
	else if(0x02==udpControlCommand.gear_position)
		{
		    msgParkingcmd.parking_brake=0;
			msgGearCmd_.gear_location=1;
		}
		else{
		    msgGearCmd_.gear_location=0;
			msgParkingcmd.parking_brake=1;
		}

	return msgGearCmd_;
}

driver_msgs::LightHornWiperCmd CloudComm::udpCmd2MsgLight(const UdpControlCommand &udpControlCommand)
{
	driver_msgs::LightHornWiperCmd msgLightCmd_;
	
	if(0x00==udpControlCommand.turn_signal)
		{
			msgLightCmd_.left_light=false;
			msgLightCmd_.right_light=false;
		}
	if(0x01==udpControlCommand.turn_signal)
		{
			msgLightCmd_.left_light=true;
		}
	if(0x02==udpControlCommand.turn_signal)
		{
			msgLightCmd_.right_light=true;
		}
	
	if(0x00==udpControlCommand.emergency_light)
		{
			msgLightCmd_.emergency_light=false;
		}
	if(0x01==udpControlCommand.emergency_light)
		{
			msgLightCmd_.emergency_light=true;
		}
	
	return msgLightCmd_;
}


driver_msgs::EstopCmd CloudComm::udpCmd2MsgEstop(const UdpControlCommand &udpControlCommand)
{
	driver_msgs::EstopCmd msgEstopCmd_;
	
	msgEstopCmd_.estop=udpControlCommand.emergency_brake_flag;
	
	return msgEstopCmd_;
}


driver_msgs::SteeringWheelCmd CloudComm::udpCmd2MsgSteering(const UdpControlCommand &udpControlCommand)
{
	driver_msgs::SteeringWheelCmd msgSteeringCmd_;
	
	msgSteeringCmd_.steering_wheel_angle=udpControlCommand.steering_target;
	msgSteeringCmd_.steering_wheel_angle_speed = 200;
	return msgSteeringCmd_;
}

driver_msgs::DriveCmd CloudComm::udpCmd2MsgDrive(const UdpControlCommand &udpControlCommand)
{
	driver_msgs::DriveCmd msgDriveCmd_;
	
	msgDriveCmd_.throttle_pedal= (int)udpControlCommand.throttle>10 ? 10:udpControlCommand.throttle;
	//msgDriveCmd_.brake_pedal=udpControlCommand.brake;
	msgDriveCmd_.acc_target = udpControlCommand.brake < 5? 0:-2;


	return msgDriveCmd_;
}

std_msgs::Float32 CloudComm::udpCmd2MsgSpeed(const UdpControlCommand &udpControlCommand)
{
	std_msgs::Float32 msgSpeedCmd_;
	msgSpeedCmd_.data=udpControlCommand.speed;
	return msgSpeedCmd_;
}

void CloudComm::heartOpt(char data[]){
	UdpHeart udpHeart;
	driver_msgs::EstopCmd msgEstopCmd;
	memcpy(&udpHeart,data+sizeof(MsgHeader),sizeof(udpHeart));
	if(udpHeart.equipment_status < 0 || udpHeart.equipment_status > 7){
		msgEstopCmd.estop = 1;
		estop_cmd_pub_.publish(msgEstopCmd);
		// udpConfigFeedback.control_type = 2;
	}
}

void CloudComm::pointTobePlanned(char data[]){
	planning_msgs::TrajectoryPointArray trajectoryPointArray;
	planning_msgs::TrajectoryPoint trajectoryPoint;
	UdpTobePlanned udpTobePlanned;	
	memcpy(&udpTobePlanned,data+sizeof(MsgHeader),sizeof(UdpTobePlanned));
	for (int i = 0; i < udpTobePlanned.point_nums; i++){
		trajectoryPoint.x = udpTobePlanned.point_list[i].x;
		trajectoryPoint.y = udpTobePlanned.point_list[i].y;
		trajectoryPoint.z = udpTobePlanned.point_list[i].z;
		trajectoryPointArray.points.push_back(trajectoryPoint);
	}
	points_planned_pub_.publish(trajectoryPointArray);
}

void CloudComm::queueControl(char data[]){
	platoon_msgs::PlatoonMission platoonMission;
	std::cout << "enter thissss::" << std::endl;
	QueueControlCommand queueControlCommand;
	memcpy(&queueControlCommand,data+sizeof(MsgHeader),sizeof(queueControlCommand));
	platoonMission.config.policy = queueControlCommand.queueConfig.follow_type;
	std::cout << "跟车方式： " << queueControlCommand.queueConfig.follow_type << std::endl;
	platoonMission.config.time = queueControlCommand.queueConfig.time_constant;
	platoonMission.config.distance = queueControlCommand.queueConfig.distance;
	platoonMission.config.stop_distance = queueControlCommand.queueConfig.stop_distance;
	platoonMission.config.lateral_offset = queueControlCommand.queueConfig.diamond_distance;
	platoonMission.config.deceleration = queueControlCommand.queueConfig.deceleration;
	platoonMission.command_type = queueControlCommand.command_type;
	std::cout << "控制指令" << (int)queueControlCommand.command_type << std::endl;
	platoonMission.num = queueControlCommand.vehicle_num;
	std::cout << "强制编队数目： " << (int)queueControlCommand.platoon_count << std::endl;
	std::cout << "车辆编号lalalal" << (int)queueControlCommand.vehicle_num << std::endl;
	for(int i=0; i<queueControlCommand.platoon_count; i++){
		platoonMission.vehicle_list.push_back(queueControlCommand.queue_order[i]);
	}

	platoon_control_pub_.publish(platoonMission);
}

void CloudComm::pathInfoOpt(char data[])
{
	
	count++;
	std::cout << "count:" << count;
	std::cout <<"路径信息"<<std::endl;
	planning_msgs::TrajectoryPointArray msgTrajectoryPointArray;
	UdpPathInfo udpPathInfo;
	memcpy(&udpPathInfo,data+sizeof(MsgHeader),sizeof(UdpPathInfo));

	std::cout <<"多包标识："<<udpPathInfo.package_id<<std::endl;
	std::cout <<"数据总包数："<<udpPathInfo.package_sum<<std::endl;
	std::cout <<"路径数据总字节数："<<udpPathInfo.package_byte<<std::endl;
	std::cout <<"包序号："<<udpPathInfo.package_num<<std::endl;
	std::cout <<"当前包内路径字节数："<<udpPathInfo.current_package_byte<<std::endl;

	path_pub_flag=false;
	msgTrajectoryPointArray = udpPath2MsgPath(udpPathInfo);
	cout << "路径点个数" << msgTrajectoryPointArray.points.size() << endl;
	for(int i = 0; i < msgTrajectoryPointArray.points.size(); i++){
		cout << "xxxxxxxx" << msgTrajectoryPointArray.points[i].x << endl;
		cout << "yyyyyyyy" << msgTrajectoryPointArray.points[i].y << endl;
		cout << "zzzzzzzz" << msgTrajectoryPointArray.points[i].z << endl;
	}
	if(path_pub_flag==true){
		// endPointX = msgTrajectoryPointArray.points.back().x;
		// endPointY = msgTrajectoryPointArray.points.back().y;
		// endPointZ = msgTrajectoryPointArray.points.back().z;
		// if(udpPathInfo.is_forward_shift==0){
		// 	std::cout <<"----is_forward_shift--0 comupte --endPoint-"<<std::endl;
		// 	endPointX = msgTrajectoryPointArray.points[0].x;
		// 	endPointY = msgTrajectoryPointArray.points[0].y;
		// 	endPointZ = msgTrajectoryPointArray.points[0].z;
		// }
		// msgTrajectoryPointArray.is_forward_shift = udpPathInfo.is_forward_shift;
		std::string recode_data_dir  = ros::package::getPath("launch_node");
		std::string recode_data_store_file = recode_data_dir + std::string("/data/gpsData_receive.txt");
		std::ofstream f_gps_data_out;
		f_gps_data_out = std::ofstream(recode_data_store_file);
		f_gps_data_out<< std::fixed;
		f_gps_data_out.precision(8); //设置输出精度
		f_gps_data_out <<"x        "<<"y        "<<"z        "<<"latitude        "<<"longitude         "<<"altitude         "<<std::endl;
		for(int i = 0; i < msgTrajectoryPointArray.points.size(); i++){
			f_gps_data_out <<msgTrajectoryPointArray.points[i].x<<"  "
			<<msgTrajectoryPointArray.points[i].y<<"  "
			<<msgTrajectoryPointArray.points[i].z<<"  "<<
			"0.000000        "<<"0.0000000         "<<"0.000000         "<<std::endl;
		}
		f_gps_data_out.flush();
		f_gps_data_out.close();
		path_info_pub_.publish(msgTrajectoryPointArray);
	}
	return;
}

void CloudComm::multipleTaskPoints(char data[])
{
	
	count++;
	std::cout << "count:" << count;
	std::cout <<"路径信息"<<std::endl;
	route_msgs::MultiPoint msgTaskPointArray;
	UdpTaskBlockInfo udpTaskBlockInfo;
	memcpy(&udpTaskBlockInfo,data+sizeof(MsgHeader),sizeof(UdpTaskBlockInfo));

	std::cout <<"多包标识："<<udpTaskBlockInfo.package_id<<std::endl;
	std::cout <<"数据总包数："<<udpTaskBlockInfo.package_sum<<std::endl;
	std::cout <<"路径数据总字节数："<<udpTaskBlockInfo.package_byte<<std::endl;
	std::cout <<"包序号："<<udpTaskBlockInfo.package_num<<std::endl;
	std::cout <<"当前包内路径字节数："<<udpTaskBlockInfo.current_package_byte<<std::endl;
	
	
	task_pub_flag=false;
	msgTaskPointArray = udpTask2MsgTask(udpTaskBlockInfo);
	cout << "路径点个数" << msgTaskPointArray.poses.size() << endl;
	for(int i = 0; i < msgTaskPointArray.poses.size(); i++){
		cout << "xxxxxxxx" << msgTaskPointArray.poses[i].position.x << endl;
		cout << "yyyyyyyy" << msgTaskPointArray.poses[i].position.y << endl;
		cout << "zzzzzzzz" << msgTaskPointArray.poses[i].position.z << endl;
	}
	if((task_pub_flag==true)){

		multiple_point_pub_.publish(msgTaskPointArray);
	}
	return;
}

void CloudComm::obstacleOpt(char data[]){

	UdpObstaclesInfo udpObstaclesInfo;
	memcpy(&udpObstaclesInfo,data+sizeof(MsgHeader),sizeof(UdpObstaclesInfo));
	std::cout << "数据总包数： " << udpObstaclesInfo.bag_sum << std::endl;
	std::cout << "数据总字节数： " << udpObstaclesInfo.bag_byte << std::endl;
	obs_pub_flag = true;
	int loop_coun = 0;
	if (0x01==udpObstaclesInfo.bag_type)
		{
			std::vector<perception_msgs::PredictionObstacle>().swap(msgObstacleArray.prediction_obstacles);
			// msgTaskPointArray.clear();
			loop_coun =10;
		}
	if (0x02==udpObstaclesInfo.bag_type)
		{
			loop_coun =10;
		}
	if (0x03==udpObstaclesInfo.bag_type)
		{
			loop_coun =udpObstaclesInfo.cur_obstacle_sum;
			obs_pub_flag=true;
		}	
	if (0x00==udpObstaclesInfo.bag_type)
		{
			std::vector<perception_msgs::PredictionObstacle>().swap(msgObstacleArray.prediction_obstacles);
			// msgTaskPointArray.clear();
			loop_coun =(udpObstaclesInfo.cur_obstacle_sum);
			std::cout << "zhangaiwugeshu: " << loop_coun << std::endl;
			obs_pub_flag=true;
		}
	// std::cout <<"loop_coun："<<loop_coun<<std::endl;
	if (loop_coun == 0){
		std::vector<perception_msgs::PredictionObstacle>().swap(msgObstacleArray.prediction_obstacles);
	}
	for (unsigned int i = 0;i<loop_coun;i++)
	{
		perception_msgs::PredictionObstacle prediction_obs;
		prediction_obs.is_static = ((udpObstaclesInfo.bag_type==1) ? 1:0);
		for(unsigned int j = 1;j<udpObstaclesInfo.obstacleData[i].obstaclePoints.nums;j++){
			geometry_msgs::Point32 obsPoint_;
			obsPoint_.x = udpObstaclesInfo.obstacleData[i].obstaclePoints.points[j].x;
			obsPoint_.y = udpObstaclesInfo.obstacleData[i].obstaclePoints.points[j].y;
			obsPoint_.z = udpObstaclesInfo.obstacleData[i].obstaclePoints.points[j].z;
			prediction_obs.perception_obstacle.polygon.points.push_back(obsPoint_);
		}
		prediction_obs.perception_obstacle.position.x = udpObstaclesInfo.obstacleData[i].obstaclePoints.points[0].x;
		prediction_obs.perception_obstacle.position.y = udpObstaclesInfo.obstacleData[i].obstaclePoints.points[0].y;
		prediction_obs.perception_obstacle.position.z = udpObstaclesInfo.obstacleData[i].obstaclePoints.points[0].z;
		
		msgObstacleArray.prediction_obstacles.push_back(prediction_obs);
	}
	std::cout << "障碍物总个数："  << msgObstacleArray.prediction_obstacles.size() << std::endl;
	if(obs_pub_flag == true){
		obs_pub_.publish(msgObstacleArray);
	}

}

void CloudComm::blockPoints(char data[])
{
	
	count++;
	std::cout << "count:" << count;
	std::cout <<"路径信息"<<std::endl;
	route_msgs::Replan msgBlockPointArray;
	UdpTaskBlockInfo udpTaskBlockInfo;
	memcpy(&udpTaskBlockInfo,data+sizeof(MsgHeader),sizeof(UdpTaskBlockInfo));

	std::cout <<"多包标识："<<udpTaskBlockInfo.package_id<<std::endl;
	std::cout <<"数据总包数："<<udpTaskBlockInfo.package_sum<<std::endl;
	std::cout <<"路径数据总字节数："<<udpTaskBlockInfo.package_byte<<std::endl;
	std::cout <<"包序号："<<udpTaskBlockInfo.package_num<<std::endl;
	std::cout <<"当前包内路径字节数："<<udpTaskBlockInfo.current_package_byte<<std::endl;
	
	
	block_pub_flag=false;
	msgBlockPointArray = udpBlock2MsgBlock(udpTaskBlockInfo);
	// cout << "路径点个数" << msgTrajectoryPointArray.points.size() << endl;
	// for(int i = 0; i < msgTrajectoryPointArray.points.size(); i++){
	// 	cout << "xxxxxxxx" << msgTrajectoryPointArray.points[i].x << endl;
	// 	cout << "yyyyyyyy" << msgTrajectoryPointArray.points[i].y << endl;
	// 	cout << "zzzzzzzz" << msgTrajectoryPointArray.points[i].z << endl;
	// }

	if( (block_pub_flag==true) && (!msgBlockPointArray.points.empty()))
	{
		// while (block_point_pub_.getNumSubscribers() == 0) ros::Duration(0.05).sleep();
		std::cout << "开始重规划下发 :" << std::endl;
		block_point_pub_.publish(msgBlockPointArray);
	}
	return;
}

void CloudComm::mapSign(char data[]){
	uint8 udpMapSign;
	// route_msgs::MapSign map_sign;
	std_msgs::UInt8 map_sign;
	memcpy(&udpMapSign,data+sizeof(MsgHeader),sizeof(udpMapSign));

	map_sign.data = udpMapSign;
	// map_sign.origin_lat = udpMapSign.origin_lat;
	// map_sign.origin_lon = udpMapSign.origin_lon;
	// map_sign.origin_alt = udpMapSign.origin_alt;
	map_sign_pub_.publish(map_sign);

}

void CloudComm::satState(char data[]){
	uint8 sat_state_;
	std_msgs::UInt8 sat_state;
	memcpy(&sat_state_,data+sizeof(MsgHeader),sizeof(sat_state_));

	sat_state.data = sat_state_;
	satellite_mode_pub_.publish(sat_state);

}


void CloudComm::initPoint(char data[]){
	TaskBlockPoint init_point;
	route_msgs::InitPoint init_point_;
	memcpy(&init_point,data+sizeof(MsgHeader),sizeof(init_point));
	init_point_.pose.position.x = init_point.x;
	init_point_.pose.position.y = init_point.y;
	init_point_.pose.position.z = init_point.z;
	tf2::Quaternion q;
	q.setRPY(0, 0, init_point.heading);
	init_point_.pose.orientation = tf2::toMsg(q);
	std::cout << "enter thisssss  chushidian" << std::endl;
	initPoint_pub_.publish(init_point_);

}

void CloudComm::mapRequest(char data[]){
	std::cout << "enter this1" << std::endl;
	std::string pkg_launch_dir = ros::package::getPath("launch_node");
	string yaml_location = pkg_launch_dir + std::string("/param/global/global_config.yaml");
	string map_location = pkg_launch_dir + std::string("/data/cloudmap.osm") + std::string(";") + yaml_location;
	MsgHeader msgHeader;
	MsgEnd msgEnd;
	msgHeader.msg_id  = 0x9040;	
	msgHeader.msg_length = map_location.length();	
	msgHeader.msg_num = msg_num_++;
	msgHeader.msg_time = Acquire24AbsTime();
	msgHeader.msg_vehicle_num = numInfo.vehicle_num;
	msgHeader.msg_source_add = inet_addr(netInfo.host_ip.c_str()) ;
	msgHeader.msg_destination_add = inet_addr(netInfo.cloud_ip.c_str()) ;
	msgHeader.msg_flag = 0x00FF ;
	std::cout << "enter this2" << std::endl;
	memcpy(sendBuf,&msgHeader,sizeof(msgHeader));
	memcpy(sendBuf+sizeof(msgHeader),map_location.c_str(),map_location.length());
	memcpy(sendBuf+sizeof(msgHeader)+map_location.length(),&msgEnd,sizeof(msgEnd));
	udp_send_ptr->send(sendBuf, sizeof(MsgHeader)+map_location.length()+sizeof(MsgEnd), netInfo.cloud_ip, netInfo.cloud_port);

	std::cout << "地图存放位置： " << map_location << std::endl;
	std::cout << "配置文件存放位置： " << yaml_location << std::endl;
}

void CloudComm::chassisControl(char data[])
{
	
	UdpChassisControl udpChassisControl;
	driver_msgs::PowerCmd msgPowerCmd;
	driver_msgs::ModeCmd msgModeCmd;
	driver_msgs::GearCmd msgGearCmd;
	driver_msgs::ParkingBrakeCmd msgParkingcmd;
	driver_msgs::DriveCmd msgDriveCmd;
	driver_msgs::SteeringWheelCmd msgSteeringCmd;
	driver_msgs::LightHornWiperCmd msgLightCmd;
	driver_msgs::EstopCmd msgStopCmd;
	driver_msgs::ChassisCmd msgChassisCmd;
	std_msgs::Float32 msgSpeedCmd;
	std_msgs::Float64 msgCruiseCmd;

	driver_msgs::MotionStartCmd msgMotionCmd;
	//UdpPathInfo udpPathInfo;
	memcpy(&udpChassisControl,data+sizeof(MsgHeader),sizeof(udpChassisControl));

	// std::cout <<"路径类型："<<udpChassisControl.path_type<<std::endl;
	// std::cout <<"多包标识："<<udpChassisControl.package_id<<std::endl;
	// std::cout <<"数据总包数："<<udpChassisControl.package_sum<<std::endl;
	// std::cout <<"路径数据总字节数："<<udpChassisControl.package_byte<<std::endl;
	// std::cout <<"包序号："<<udpChassisControl.package_num<<std::endl;
	// std::cout <<"当前包内路径字节数："<<udpChassisControl.current_package_byte<<std::endl;

	if (1 ){
	     msgPowerCmd.engine_on_off = udpChassisControl.engine_on_off;
		 msgMotionCmd.motion_start = udpChassisControl.engine_on_off;
	     power_cmd_pub_.publish(msgPowerCmd);
		 motion_cmd_pub_.publish(msgMotionCmd);
	}
	if (1){
		// cout << "发动机" << udpChassisControl.engine_on_off << endl;
		msgModeCmd.driving_mode = udpChassisControl.driving_mode;

		msgModeCmd.steer_mode = udpChassisControl.steer_mode;
		msgModeCmd.mode_flag = udpChassisControl.autonomous_mode;
		if (msgModeCmd.driving_mode)
		{
		    if ((0 == udpChassisFeedback.driving_mode) ||
				((1 == udpChassisFeedback.driving_mode) && (udpChassisFeedback.autonomous_mode != msgModeCmd.mode_flag))){
				
				//driver_msgs::ModeCmd tmpMsgModeCmd;
				//tmpMsgModeCmd.driving_mode = 0;
				//tmpMsgModeCmd.mode_flag = 0;
				//mode_cmd_pub_.publish(tmpMsgModeCmd);
				//usleep(500 * 1000);
		    }
		}
		// cout << "驾驶模式" << udpChassisControl.driving_mode << endl;
		// cout << "無人模式细分：" << (int)udpChassisControl.autonomous_mode << endl;
		// msgModeCmd.rear_steer_mode = udpChassisControl.rear_steer_mode;
		mode_cmd_pub_.publish(msgModeCmd);
	}
    
	msgGearCmd.gear_location = udpChassisControl.gear_location;
	// cout << "档位模式" << (int)udpChassisControl.gear_location << endl;
	// msgGearCmd.tran_gear = udpChassisControl.tran_gear;
	gear_cmd_pub_.publish(msgGearCmd);

	msgParkingcmd.parking_brake=udpChassisControl.parking_brake;
	// cout << "駐車制動" << (int)udpChassisControl.parking_brake << endl;
	parking_cmd_pub_.publish(msgParkingcmd);

	// msgDriveCmd.engine_torque_target = udpChassisControl.engine_torque_target;
	//  cout << "油門" << udpChassisControl.engine_torque_target << endl;
	msgDriveCmd.brake_pedal = udpChassisControl.brake_pedal;
	msgDriveCmd.throttle_pedal = (int)udpChassisControl.engine_torque_target>10 ? 10:udpChassisControl.engine_torque_target;
	// msgDriveCmd.brake_pedal=udpChassisControl.brake_pedal;
	msgDriveCmd.acc_target = udpChassisControl.brake_pedal / 100.0 * (-5.0);
	// cout << "剎車" << udpChassisControl.brake_pedal << endl;

	drive_cmd_pub_.publish(msgDriveCmd);

	// msgChassisCmd.mode_flag = udpChassisControl.autonomous_mode;
	// msgChassisCmd.velocity_target = udpChassisControl.speed;
	// msgChassisCmd.air_lamp = udpChassisControl.blackout_light;
	// chassis_cmd_pub_.publish(msgChassisCmd);

	msgSpeedCmd.data = udpChassisControl.speed;
	msgCruiseCmd.data = udpChassisControl.speed;

	speed_cmd_pub_.publish(msgSpeedCmd);
	cruise_cmd_pub_.publish(msgCruiseCmd);

	msgSteeringCmd.steering_wheel_angle = udpChassisControl.steering_wheel_angle;
	// cout << "轉角" << udpChassisControl.steering_wheel_angle << endl;

	msgSteeringCmd.steering_wheel_angle_speed = udpChassisControl.steering_wheel_angle_speed;
	// cout << "轉角速度" << udpChassisControl.steering_wheel_angle_speed << endl;
	steering_cmd_pub_.publish(msgSteeringCmd);

	msgLightCmd.low_light = udpChassisControl.low_light;
	msgLightCmd.high_light = udpChassisControl.high_light;
	msgLightCmd.left_light = udpChassisControl.left_light;
	msgLightCmd.right_light = udpChassisControl.right_light;
	msgLightCmd.front_foggy_light = udpChassisControl.foggy_light;
	msgLightCmd.rear_foggy_light = udpChassisControl.reverse_light;
	msgLightCmd.emergency_light = udpChassisControl.emergency_light;

	msgLightCmd.position_light = udpChassisControl.position_light;
	msgLightCmd.reverse_light = udpChassisControl.rear_light;
	// msgLightCmd.brake_light = udpChassisControl.brake_light;
	msgLightCmd.horn = udpChassisControl.air_horn;
	// msgLightCmd.electric_horn = udpChassisControl.electric_horn;    //喇叭无区分
	//无室内灯
	light_cmd_pub_.publish(msgLightCmd);

	
	if (1){
	    msgStopCmd.estop = udpChassisControl.estop;
	    estop_cmd_pub_.publish(msgStopCmd);
	}
	// cout << "级艇" << udpChassisControl.estop << endl;

	return;
}

void CloudComm::viewChange(char data[]){
	UdpViewChange udpViewChange;
	memcpy(&udpViewChange,data+sizeof(MsgHeader),sizeof(udpViewChange));

	

}

void CloudComm::paramConfig(char data[]){
	UdpParamConfig udpParamConfig;
	configuration_msgs::ConfigurationDown configurationDown;
	memcpy(&udpParamConfig,data+sizeof(MsgHeader),sizeof(udpParamConfig));
	configurationDown.config_type = udpParamConfig.config_type;
	configurationDown.lead_vehicle = udpParamConfig.lead_vehicle;
	configurationDown.follow_type = udpParamConfig.follow_type;
	configurationDown.config_load = udpParamConfig.config_load;
	configurationDown.config_speed = udpParamConfig.config_speed;
	configurationDown.config_space = udpParamConfig.config_space;

	configuration_pub_.publish(configurationDown);
}

// void  CloudComm::saveData(planning_msgs::TrajectoryPointArray msgTrajectoryPointArray)
// {   
// 	std::string recode_data_dir  = ros::package::getPath("launch_node");
// 	std::string recode_data_store_file = recode_data_dir + std::string("/data/gpsDataRemote.txt");	
// 	std::ofstream f_gps_data_out;

// 	f_gps_data_out = std::ofstream(recode_data_store_file);
// 	f_gps_data_out<< std::fixed;
// 	f_gps_data_out.precision(8); //设置输出精度

// 	f_gps_data_out <<"x        "<<"y        "<<"z        "<<"latitude        "<<"longitude         "<<"altitude         "<<std::endl;
//     for (auto &point :msgTrajectoryPointArray.points)
//     {
// 	    f_gps_data_out <<point.x <<"  " <<"  "<<point.y;
// 	    f_gps_data_out <<std::endl;
//     }
	
// 	f_gps_data_out.flush();
// 	f_gps_data_out.close();

// 	return;
// }




planning_msgs::TrajectoryPointArray  CloudComm::udpPath2MsgPath(const UdpPathInfo &udpPathInfo)
{
	int loop_coun = 0;
	
	if (0x01==udpPathInfo.package_id)
		{
			std::vector<planning_msgs::TrajectoryPoint>().swap(msgTrajectoryPointArray.points);
			//msgTrajectoryPointArray.points.clear();
			loop_coun =20;
		}
	if (0x02==udpPathInfo.package_id)
		{
			loop_coun =20;
		}
	if (0x03==udpPathInfo.package_id)
		{
			loop_coun =(udpPathInfo.current_package_byte)/28;
			path_pub_flag=true;
		}	
	if (0x00==udpPathInfo.package_id)
		{
			std::vector<planning_msgs::TrajectoryPoint>().swap(msgTrajectoryPointArray.points);
			//msgTrajectoryPointArray.points.clear();
			loop_coun =(udpPathInfo.current_package_byte)/28;
			path_pub_flag=true;
		}
	// std::cout <<"loop_coun："<<loop_coun<<std::endl;
	for (unsigned int i = 0;i<loop_coun;i++)
		{
			planning_msgs::TrajectoryPoint trajectoryPoint_;
			trajectoryPoint_.x=udpPathInfo.path_point_list[i].x;
			trajectoryPoint_.y=udpPathInfo.path_point_list[i].y;
			trajectoryPoint_.z=udpPathInfo.path_point_list[i].z;

			//如果下发的是经纬高
			//geometry_msgs::Point trajectoryPoint;
			//trajectoryPoint.x=udpPathInfo.path_point_list[i].x;
			//trajectoryPoint.y=udpPathInfo.path_point_list[i].y;
			//trajectoryPoint.z=udpPathInfo.path_point_list[i].z;
			
			//trajectoryPoint = projector.forward(trajectoryPoint);

			//trajectoryPoint_.x=trajectoryPoint.x;
			//trajectoryPoint_.y=trajectoryPoint.y;
			//trajectoryPoint_.z=trajectoryPoint.z;
			
			msgTrajectoryPointArray.points.push_back(trajectoryPoint_);
		}
	
	// std::cout << "路径点数量"<<msgTrajectoryPointArray.points.size()<<std::endl;
	
	return msgTrajectoryPointArray;
}

route_msgs::MultiPoint CloudComm::udpTask2MsgTask(const UdpTaskBlockInfo &udpTaskBlockInfo)
{
	int loop_coun = 0;
	
	if (0x01==udpTaskBlockInfo.package_id)
		{
			std::vector<geometry_msgs::Pose>().swap(msgTaskPointArray.poses);
			// msgTaskPointArray.clear();
			loop_coun =20;
		}
	if (0x02==udpTaskBlockInfo.package_id)
		{
			loop_coun =20;
		}
	if (0x03==udpTaskBlockInfo.package_id)
		{
			loop_coun =(udpTaskBlockInfo.current_package_byte)/32;
			task_pub_flag=true;
		}	
	if (0x00==udpTaskBlockInfo.package_id)
		{
			std::vector<geometry_msgs::Pose>().swap(msgTaskPointArray.poses);
			// msgTaskPointArray.clear();
			loop_coun =(udpTaskBlockInfo.current_package_byte)/32;
			task_pub_flag=true;
		}
	// std::cout <<"loop_coun："<<loop_coun<<std::endl;
	for (unsigned int i = 0;i<loop_coun;i++)
		{
			geometry_msgs::Pose taskPoint_;
			tf2::Quaternion q;
			q.setRPY(0, 0, udpTaskBlockInfo.path_point_list[i].heading);
			taskPoint_.orientation = tf2::toMsg(q);

			taskPoint_.position.x=udpTaskBlockInfo.path_point_list[i].x;
			taskPoint_.position.y=udpTaskBlockInfo.path_point_list[i].y;
			taskPoint_.position.z=udpTaskBlockInfo.path_point_list[i].z;


			
			msgTaskPointArray.poses.push_back(taskPoint_);
		}
	
	// std::cout << "路径点数量"<<msgTrajectoryPointArray.points.size()<<std::endl;
	
	return msgTaskPointArray;
}

route_msgs::Replan CloudComm::udpBlock2MsgBlock(const UdpTaskBlockInfo &udpTaskBlockInfo)
{
	int loop_coun = 0;
	
	if (0x01==udpTaskBlockInfo.package_id)
		{
			std::vector<geometry_msgs::Point>().swap(msgBlockPointArray.points);
			// msgTaskPointArray.clear();
			loop_coun =20;
		}
	if (0x02==udpTaskBlockInfo.package_id)
		{
			loop_coun =20;
		}
	if (0x03==udpTaskBlockInfo.package_id)
		{
			loop_coun =(udpTaskBlockInfo.current_package_byte)/32;
			block_pub_flag=true;
		}	
	if (0x00==udpTaskBlockInfo.package_id)
		{
			std::vector<geometry_msgs::Point>().swap(msgBlockPointArray.points);
			// msgTaskPointArray.clear();
			loop_coun =(udpTaskBlockInfo.current_package_byte)/32;
			block_pub_flag=true;
		}
	// std::cout <<"loop_coun："<<loop_coun<<std::endl;
	for (unsigned int i = 0;i<loop_coun;i++)
		{
			geometry_msgs::Point blockPoint_;
			blockPoint_.x=udpTaskBlockInfo.path_point_list[i].x;
			blockPoint_.y=udpTaskBlockInfo.path_point_list[i].y;
			blockPoint_.z=udpTaskBlockInfo.path_point_list[i].z;


			
			msgBlockPointArray.points.push_back(blockPoint_);
		}
	
	// std::cout << "路径点数量"<<msgTrajectoryPointArray.points.size()<<std::endl;
	
	return msgBlockPointArray;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cloud_comm_node");

    ros::NodeHandle nh;
    CloudComm CloudComm_(nh);
    ros::spin();
    return 0;
}

