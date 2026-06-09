#include "info_hub/info_hub.h"
#include <pwd.h>

//构造函数
InfoHub::InfoHub(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{ 

	std::string vehicle_platform_file;
	private_nh_.param<std::string>("vehicle_platform_file", vehicle_platform_file, "vehicle_platform.yaml");
    common::getPlatformParam(vehicle_platform_file,platformParam);
	
	std::cout <<platformParam.vehicle_type<<std::endl;
    std::cout <<platformParam.ins_type<<std::endl;
    std::cout <<platformParam.id<<std::endl;
    std::cout <<platformParam.num<<std::endl;

	private_nh_.param<std::string>(platformParam.id +std::string("/dsrc_ip"), netInfo.dsrc_ip, "");
	private_nh_.param<int>(platformParam.id +std::string("/dsrc_port"), netInfo.dsrc_port, 9999);
	private_nh_.param<std::string>(platformParam.id +std::string("/host_ip"), netInfo.host_ip, "");
	private_nh_.param<int>(platformParam.id +std::string("/host_port"), netInfo.host_port, 9999);
	private_nh_.param<int>("timeOutmilliSecond", netInfo.timeOutmilliSecond, 1000);
	
    /*socket初始化*/
    socketInit(netInfo.timeOutmilliSecond);
    sleep(1);
    //"/PlatoonMember"订阅
    platoon_member_sub = nh_.subscribe("PlatoonMember_self", 10, &InfoHub::platoonmemberCallback, this);

    //"/PlatoonConfig_Leader"订阅（作为领航车 ，接收ui的命令，转成UDP发出去）
    platoon_config_sub= nh_.subscribe("PlatoonConfig_self", 10, &InfoHub::platoonconfigCallback, this);

    //"/PlatoonMission_Leader"订阅（作为领航车 ，接收ui的命令，转成UDP发出去）
    platoon_mission_sub= nh_.subscribe("PlatoonMission_self", 10, &InfoHub::platoonmissionCallback, this);

	/*发布*/

    //"/PlatoonConfig"发布(作为跟随车从UDP收，然后发给其他模块)
    platoon_config_pub=nh_.advertise<platoon_msgs::PlatoonConfig>("/PlatoonConfig",1);

    //"/PlatoonMission"发布(作为跟随车从UDP收，然后发给其他模块)
    platoon_mission_pub=nh_.advertise<platoon_msgs::PlatoonMission>("/PlatoonMission",1);

    //"/PlatoonMember"发布(作为跟随车从UDP收，然后发给其他模块)
    platoon_member_pub=nh_.advertise<platoon_msgs::PlatoonMember>("/PlatoonMember",1);
	
	msgsFunc.push_back(&InfoHub::platoonNoneOpt);
	msgsFunc.push_back(&InfoHub::platoonMemberOpt);
	msgsFunc.push_back(&InfoHub::platoonConfigOpt);
	msgsFunc.push_back(&InfoHub::platoonMissionOpt);
    new_PlatoonMember_flag = false;
	new_PlatoonConfig_flag = false;
	new_PlatoonMission_flag = false;
	
    /*开启线程*/
	udp_thread_ = std::thread (&InfoHub::udpThreadFunc,this);
	
	udp_thread_.detach();
}



void InfoHub::udpThreadFunc()
{
	FullUdpUdpPlatoonMember  fullUdpPlatoonMember_;
	FullUdpPlatoonMission    fullUdpPlatoonMission_;
	FullUdpPlatoonConfig     fullUdpPlatoonConfig_;
	
	fullUdpPlatoonMember_.NetHeader_.usMsgType  = PLATOON_MEMBER;
	fullUdpPlatoonMission_.NetHeader_.usMsgType = PLATOON_MISSION;
	fullUdpPlatoonConfig_.NetHeader_.usMsgType  = PLATOON_CONFIG;
	
    std::cout <<"member data size "<<sizeof(UdpPlatoonMember )<<std::endl;
    std::cout <<"mission size "<<sizeof(UdpPlatoonMission)<<std::endl;
    std::cout <<"config size "<<sizeof(UdpPlatoonConfig)<<std::endl;
	char recvBuf[1024];
	char sendBuf[1024];
	memset(sendBuf,0,1024);
	sendBuf[0] = 1;


	
    while (true)
    {
	
	    if (new_PlatoonMember_flag){
			//std::cout <<"new_PlatoonMission_flag"<<std::endl;
			new_PlatoonMember_flag = false;
            fullUdpPlatoonMember_.NetHeader_.uiMsgNo++;
			/*桌面测试使用*/
			//udpMemTest(UdpPlatoonMember_.member);
		    fullUdpPlatoonMember_.member = platoonMember_;
			memcpy(sendBuf,&fullUdpPlatoonMember_,sizeof(fullUdpPlatoonMember_));
			udp_send_ptr->send(sendBuf, sizeof(sendBuf), netInfo.dsrc_ip, netInfo.dsrc_port);
        } 
		
		if (new_PlatoonMission_flag){
			
			std::cout <<"new_PlatoonMission_flag"<<std::endl;
			new_PlatoonMission_flag = false;
	        fullUdpPlatoonMission_.NetHeader_.uiMsgNo++;
			
			/*桌面测试使用*/
			//udpMisTest(UdpPlatoonMission_.mission);
			
			fullUdpPlatoonMission_.mission = platoonMission_;
			memcpy(sendBuf,&fullUdpPlatoonMission_,sizeof(fullUdpPlatoonMission_));
			udp_send_ptr->send(sendBuf, sizeof(sendBuf), netInfo.dsrc_ip, netInfo.dsrc_port);
		}
		
		if (new_PlatoonConfig_flag){
			
			std::cout <<"new_PlatoonConfig_flag"<<std::endl;
			new_PlatoonConfig_flag = false;       
			fullUdpPlatoonConfig_.NetHeader_.uiMsgNo++;
			
			/*桌面测试使用*/
			//udpConTest(UdpPlatoonConfig_.config);
	
			fullUdpPlatoonConfig_.config = platoonConfig_;
			memcpy(sendBuf,&fullUdpPlatoonConfig_,sizeof(fullUdpPlatoonConfig_));
			udp_send_ptr->send(sendBuf, sizeof(sendBuf), netInfo.dsrc_ip, netInfo.dsrc_port);
		}
		
		int recvLength = udp_recv_ptr->receive(recvBuf, sizeof(recvBuf));
        if ((recvLength <= 0 ) || ( recvLength < (int)sizeof(NetHeader)))
			continue;
		NetHeader NetHeader_;
        memcpy(&NetHeader_,recvBuf+DSRC_RECV_HEADER_RES,sizeof(NetHeader_));
		assert(NetHeader_.usMsgType >= PLATOON_MEMBER);
		assert(NetHeader_.usMsgType <= PLATOON_MISSION);
	    (this->*msgsFunc[NetHeader_.usMsgType])(recvBuf);
    }
}

bool InfoHub::socketInit(int milliSecond)
{
	 udp_send_ptr = std::make_shared<udpServer>();
	 udp_recv_ptr = std::make_shared<udpServer>();
	 
	 std::cout <<"host_port "<<netInfo.host_port<<std::endl;
	 std::cout <<"host_ip "<<netInfo.host_ip<<std::endl;
	 
	 udp_recv_ptr->setup(netInfo.host_port,netInfo.host_ip);
	 udp_recv_ptr->setTimeOut(100);
	 return true;
}


void InfoHub::platoonNoneOpt(char data[])
{  
    std::cout <<"error platoonNoneOpt "<<std::endl;
	return;
}

void InfoHub::printMember(UdpPlatoonMember &udpMember)
{
    
	std::cout <<"printMember"<<std::endl;
	std::cout <<"id:  "<<udpMember.id<<std::endl;
	std::cout <<"num :"<<(int)udpMember.num<<std::endl;
	std::cout <<"role: "<<(int)udpMember.role<<std::endl;
	std::cout <<"type: "<<(int)udpMember.type<<std::endl;
	std::cout <<"x :"<<udpMember.x<<std::endl;
	std::cout <<"y: "<<udpMember.y<<std::endl;
	std::cout <<"heading: "<<udpMember.heading<<std::endl;
	std::cout <<"velocity: "<<udpMember.linear_velocity<<std::endl;
	std::cout <<"linear_acceleration: "<<udpMember.linear_acceleration<<std::endl;
	std::cout <<"gear :"<<(int)udpMember.gear<<std::endl;
	std::cout <<"driving_mode: "<<(int)udpMember.driving_mode<<std::endl;
	std::cout <<"spacing_distance: "<<udpMember.spacing_distance<<std::endl;
	std::cout <<"front_overhang_m: "<<udpMember.front_overhang_m<<std::endl;
	std::cout <<"rear_overhang_m: "<<udpMember.rear_overhang_m<<std::endl;
	std::cout <<"wheel_base_m: "<<udpMember.wheel_base_m<<std::endl;
	std::cout <<"vehicle_width_m: "<<udpMember.vehicle_width_m<<std::endl;
}


void InfoHub::printConfig(UdpPlatoonConfig &udpConfig)
{  
	std::cout <<"printConfig"<<std::endl;
	std::cout <<"policy: "<<udpConfig.policy<<std::endl;
	std::cout <<"time: "<<udpConfig.time<<std::endl;
	std::cout <<"distance: "<<udpConfig.distance<<std::endl;
	std::cout <<"lateral_offset: "<<udpConfig.lateral_offset<<std::endl;
	std::cout <<"use_path_planning: "<<(int)udpConfig.use_path_planning<<std::endl;
}

void InfoHub::printMission(UdpPlatoonMission &udpMission)
{
	
	std::cout <<"printMission"<<std::endl;
	std::cout <<"num: "<<udpMission.num<<std::endl;
	std::cout <<"command_type: "<<udpMission.command_type<<std::endl;
	printConfig(udpMission.config);
	std::cout <<"vehicle_list: ";
	for (auto &num:udpMission.vehicle_list)
		std::cout <<(int)num<<"  ";
	std::cout <<std::endl;
}

void InfoHub::platoonMemberOpt(char data[])
{  
	platoon_msgs::PlatoonMember msgMember;
	UdpPlatoonMember udpMember;
	memcpy(&udpMember,data+DSRC_RECV_HEADER_RES+sizeof(NetHeader),sizeof(UdpPlatoonMember));
	
	/*测试是否接收到UDP数据*/
    //printMember(udpMember);
    
	msgMember = UdpMember2MsgMember(udpMember);
	platoon_member_pub.publish(msgMember);
	return;
}

void InfoHub::platoonConfigOpt(char data[])
{
	platoon_msgs::PlatoonConfig msgConfig;
	UdpPlatoonConfig udpConfig;
	memcpy(&udpConfig,data+DSRC_RECV_HEADER_RES+sizeof(NetHeader),sizeof(UdpPlatoonConfig));
	
	/*测试是否接收到UDP数据*/
    printConfig(udpConfig);
	
	msgConfig = UdpConfig2MsgConfig(udpConfig);
	platoon_config_pub.publish(msgConfig);
	return;

}

void InfoHub::platoonMissionOpt(char data[])
{
	platoon_msgs::PlatoonMission msgMission;
	UdpPlatoonMission udpMission;
	memcpy(&udpMission,data+DSRC_RECV_HEADER_RES+sizeof(NetHeader),sizeof(UdpPlatoonMission));
	
	/*测试是否接收到UDP数据*/
    printMission(udpMission);
	
	msgMission = UdpMission2MsgMission(udpMission);
	platoon_mission_pub.publish(msgMission);
	return;
}


platoon_msgs::PlatoonMember InfoHub::UdpMember2MsgMember(const UdpPlatoonMember  &udpMember)
{
    platoon_msgs::PlatoonMember  msgMember;
	//std::cout <<"udp id length "<<udpMember.id_length<<std::endl;
	std::string recv_id;
	recv_id.resize(udpMember.id_length);
	for (unsigned int i = 0; i< udpMember.id_length;i++)
		recv_id[i] = udpMember.id[i];
	//std::cout <<"recv_id -------------------   "<<recv_id<<std::endl;
	msgMember.id = recv_id;
	msgMember.num = udpMember.num;
	msgMember.role = udpMember.role;
	msgMember.type = udpMember.type;
	msgMember.position.x  = udpMember.x;
	msgMember.position.y = udpMember.y;
	msgMember.heading = udpMember.heading;
	msgMember.linear_velocity = udpMember.linear_velocity;
	msgMember.linear_acceleration = udpMember.linear_acceleration;
	msgMember.gear = udpMember.gear;
	msgMember.driving_mode = udpMember.driving_mode;
	msgMember.spacing_distance = udpMember.spacing_distance;
	msgMember.front_overhang_m = udpMember.front_overhang_m;
	msgMember.rear_overhang_m = udpMember.rear_overhang_m;
	msgMember.vehicle_width_m = udpMember.vehicle_width_m;
	msgMember.wheel_base_m = udpMember.wheel_base_m;
	
	return msgMember;
}


UdpPlatoonMember InfoHub::MsgMember2UdpMember(const platoon_msgs::PlatoonMember::ConstPtr &msgMember)
{
    //std::cout <<"id:------- "<<msgMember->id<<"length "<<msgMember->id.length()<<std::endl;
    UdpPlatoonMember udpMember;
	//std::cout <<"sizeof  "<<sizeof(udpMember) <<std::endl;
	//std::cout <<"siezeof double  "<<sizeof(double)<<std::endl;
	//std::cout <<"siezeof int  "<<sizeof(udpMemberTest)<<std::endl;
	
	memset(&udpMember,sizeof(udpMember),0x00);
	udpMember.id_length = msgMember->id.length();
    for (unsigned int i  = 0; i< msgMember->id.length();i++)
		udpMember.id[i] =  msgMember->id[i];
	//memcpy(udpMember.id,msgMember->id.c_str(),msgMember->id.length());
	udpMember.num  = msgMember->num;
	udpMember.role = msgMember->role;
	udpMember.type = msgMember->type;
	udpMember.x = msgMember->position.x;
	udpMember.y = msgMember->position.y;
	udpMember.heading = msgMember->heading;
	udpMember.linear_velocity = msgMember->linear_velocity;
	udpMember.linear_acceleration = msgMember->linear_acceleration;
	udpMember.gear = msgMember->gear;
	udpMember.driving_mode = msgMember->driving_mode;
	udpMember.spacing_distance  = msgMember->spacing_distance;
	udpMember.front_overhang_m  = msgMember->front_overhang_m;
	udpMember.rear_overhang_m   = msgMember->rear_overhang_m;
	udpMember.wheel_base_m      = msgMember->wheel_base_m;
	udpMember.vehicle_width_m   = msgMember->vehicle_width_m; 
	return udpMember;
}


platoon_msgs::PlatoonConfig InfoHub::UdpConfig2MsgConfig(const UdpPlatoonConfig &udpConfig)
{
    platoon_msgs::PlatoonConfig  msgConfig;

	msgConfig.policy  = udpConfig.policy;
	msgConfig.time = udpConfig.time;
	msgConfig.distance = udpConfig.distance;
	msgConfig.lateral_offset = udpConfig.lateral_offset;
	msgConfig.use_path_planning = udpConfig.use_path_planning;
	return msgConfig;
       
}


UdpPlatoonConfig InfoHub::MsgConfig2UdpConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msgConfig)
{
    UdpPlatoonConfig  udpConfig;
	udpConfig.policy            = msgConfig->policy;
	udpConfig.time              = msgConfig->time;
	udpConfig.distance          = msgConfig->distance;
	udpConfig.lateral_offset    = msgConfig->lateral_offset;
	udpConfig.use_path_planning = msgConfig->use_path_planning;
	return udpConfig;

}


UdpPlatoonConfig InfoHub::MsgConfig2UdpConfig(const platoon_msgs::PlatoonConfig &msgConfig)
{
    UdpPlatoonConfig  udpConfig;
	udpConfig.policy            = msgConfig.policy;
	udpConfig.time              = msgConfig.time;
	udpConfig.distance          = msgConfig.distance;
	udpConfig.lateral_offset    = msgConfig.lateral_offset;
	udpConfig.use_path_planning = msgConfig.use_path_planning;
	return udpConfig;

}


platoon_msgs::PlatoonMission InfoHub::UdpMission2MsgMission(const UdpPlatoonMission &udpMission)
{
    platoon_msgs::PlatoonMission msgMission;
	msgMission.command_type = udpMission.command_type;
	assert(msgMission.command_type >= PlatoonType::BUILD);
	assert(msgMission.command_type <= PlatoonType::RUNNING);
    msgMission.config = UdpConfig2MsgConfig(udpMission.config);
	msgMission.num = udpMission.num;
	for (unsigned int i = 0; i < udpMission.count ;i++)
	{
		msgMission.vehicle_list.push_back(udpMission.vehicle_list[i]);
	}
	return msgMission;
}


UdpPlatoonMission InfoHub::MsgMission2UdpMission(const platoon_msgs::PlatoonMission::ConstPtr &msgMission)
{
    UdpPlatoonMission  udpMission;
	memset(&udpMission,sizeof(udpMission),0x00);
	udpMission.command_type = msgMission->command_type;
	
    udpMission.config = MsgConfig2UdpConfig(msgMission->config);

	for (unsigned int i  = 0 ; i < msgMission->vehicle_list.size();i++)
	{
	    udpMission.vehicle_list[i] =  msgMission->vehicle_list[i]; 
	}
	udpMission.count = msgMission->vehicle_list.size();
	udpMission.num   = msgMission->num;

	return udpMission;
}

void InfoHub::platoonmemberCallback(const platoon_msgs::PlatoonMember::ConstPtr &msg)
{  
    //std::cout <<"InfoHub: platoonmemberCallback 1"<<std::endl;
	platoonMember_ = MsgMember2UdpMember(msg);
	new_PlatoonMember_flag =  true;
    //std::cout <<"InfoHub: platoonmemberCallback 2"<<std::endl;
	//char sendBuf[1024];
	//udp_send_ptr->send(sendBuf, 1024, "192.168.10.11", 4040);
}


//"/PlatoonConfig_Leader"回调函数
void InfoHub::platoonconfigCallback(const platoon_msgs::PlatoonConfig::ConstPtr &msg)
{

    std::cout <<"InfoHub: platoonconfigCallback 1"<<std::endl;
	platoonConfig_ = MsgConfig2UdpConfig(msg);
	new_PlatoonConfig_flag =  true;
    std::cout <<"InfoHub: platoonconfigCallback 2"<<std::endl;
}

//"/PlatoonMission_Leader"回调函数
void InfoHub::platoonmissionCallback(const platoon_msgs::PlatoonMission::ConstPtr &msg)
{
    std::cout <<"InfoHub: platoonmissionCallback,mission 1: "<<(int)msg->command_type<<std::endl;
	platoonMission_ = MsgMission2UdpMission(msg);
	new_PlatoonMission_flag =  true;
    std::cout <<"InfoHub: platoonmissionCallback,mission 2: "<<(int)msg->command_type<<std::endl;
}


void InfoHub::udpMemTest(UdpPlatoonMember  &member)
{
	std::string id("vehicle1");
	memcpy(member.id,id.c_str(),id.length());
	member.num=11;
	member.role=12;
	member.type=13;
	member.x=14.0;
	member.y=15.0;
	member.heading=16.0;
	member.linear_velocity=17.0;
	member.linear_acceleration=18.0;
	member.gear=19;
    member.driving_mode= 20;
	member.spacing_distance=21.0 ;
	member.front_overhang_m=22.0;
	member.rear_overhang_m=23.0;
	member.wheel_base_m=24.0;
	member.vehicle_width_m=25.0;
	
}

void InfoHub::udpMisTest(UdpPlatoonMission  &mission)
{
    mission.command_type=5;
	mission.config.policy=6;
	mission.config.time=7.0;
	mission.config.distance=8.0;
	mission.config.lateral_offset=9.0;
	mission.config.use_path_planning=10;
	mission.vehicle_list[0] = 1;
	mission.vehicle_list[1] = 2;
	mission.vehicle_list[2] = 3;
	mission.vehicle_list[3] = 4;
	
}
void InfoHub::udpConTest(UdpPlatoonConfig  &config)
{
	config.policy=6;
	config.time=7.0;
	config.distance=8.0;
	config.lateral_offset=9.0;
	config.use_path_planning=10;
}


