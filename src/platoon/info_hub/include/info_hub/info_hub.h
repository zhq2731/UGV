#ifndef INFO_HUB_H_
#define INFO_HUB_H_
#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <sstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/package.h>
#include <chrono>
#include <fstream>
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "amathutils_lib/amathutils.hpp"
#include "platoon_common/platoon_common.h"

#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMission.h"


#include <tf/transform_broadcaster.h>
#include <cstring>
#include "common/common.h"

#include "yaml-cpp/yaml.h"
#include "udp/udpServer.h"
#include <mutex>

#pragma pack(1)


typedef double  float64;
typedef float  float32;
typedef unsigned char uint8;
typedef int int32;

#define DSRC_RECV_HEADER_RES 128
#define DSRC_RECV_TAIL_RES 4
#define DSRC_SEND_HEAD_RES 27

struct NetHeader
{
    int usMsgType; /*报文信息标识*/
    int uiMsgNo;   /*报文序号*/
};

struct UdpPlatoonMember
{
	float64 spacing_distance ;
	float64 front_overhang_m;
	float64 rear_overhang_m;
	float64 wheel_base_m;
	float64 vehicle_width_m;
	float64 x;
	float64 y;
	float64 heading;
	float64 linear_velocity;
	float64 linear_acceleration;
	char    id[16];
	uint8   num ;
	uint8   role;
	uint8   type;
	uint8   gear;
	uint8   driving_mode;
    uint8   id_length;
	uint8   reserve[2];  
};


struct UdpPlatoonConfig
{   
	float32 time;
	float32 distance;
	float32 lateral_offset;
	float32 reserve1;
	int32   policy ;
	uint8   use_path_planning;
	uint8   reserve2[3];
};

struct UdpPlatoonMission
{
	UdpPlatoonConfig config;
    int   command_type ;
	uint8 vehicle_list[12];
	int   count;
	int num;
};

struct FullUdpPlatoonConfig
{
    char reserve[DSRC_SEND_HEAD_RES];
	NetHeader NetHeader_;
	UdpPlatoonConfig config;
};

struct FullUdpUdpPlatoonMember
{
    char reserve[DSRC_SEND_HEAD_RES];
    NetHeader NetHeader_;
	UdpPlatoonMember member;
};

struct FullUdpPlatoonMission
{
    char reserve[DSRC_SEND_HEAD_RES];
    NetHeader NetHeader_;
	UdpPlatoonMission mission;
};

struct NetInfo
{
    std::string dsrc_ip;
	int dsrc_port;

	std::string host_ip;
	int  host_port;

	int timeOutmilliSecond;
};


enum UdpMsgType
{
    PLATOON_NONE = 0X00,
    PLATOON_MEMBER = 0X01,
    PLATOON_CONFIG = 0X02,
    PLATOON_MISSION = 0X03
};




class InfoHub
{
public:
	 typedef void (InfoHub::*msgOpt)(char data[]);
     InfoHub( ros::NodeHandle &nh);
private:
    ros::NodeHandle nh_;	
    ros::NodeHandle private_nh_;

    void platoonconfigCallback(const platoon_msgs::PlatoonConfig::ConstPtr &msg);
    void platoonmissionCallback(const platoon_msgs::PlatoonMission::ConstPtr &msg);
	void platoonmemberCallback(const platoon_msgs::PlatoonMember::ConstPtr &msg);

	void platoonNoneOpt(char data[]);
	void platoonMemberOpt(char data[]);
	void platoonConfigOpt(char data[]);
	void platoonMissionOpt(char data[]);
	
	UdpPlatoonConfig  MsgConfig2UdpConfig(const platoon_msgs::PlatoonConfig &msgConfig);
	UdpPlatoonConfig  MsgConfig2UdpConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msgConfig);
	UdpPlatoonMember  MsgMember2UdpMember(const platoon_msgs::PlatoonMember::ConstPtr &msgMember);
	UdpPlatoonMission MsgMission2UdpMission(const platoon_msgs::PlatoonMission::ConstPtr &msgMission);
	
	platoon_msgs::PlatoonMember  UdpMember2MsgMember(const UdpPlatoonMember  &udpMember);
	platoon_msgs::PlatoonConfig  UdpConfig2MsgConfig(const UdpPlatoonConfig &udpConfig);
	platoon_msgs::PlatoonMission UdpMission2MsgMission(const UdpPlatoonMission &udpMission);

	void printMember(UdpPlatoonMember &udpMember);
	void printConfig(UdpPlatoonConfig &udpConfig);
	void printMission(UdpPlatoonMission &udpMission);

	
	void udpMemTest(UdpPlatoonMember  &member);
	void udpMisTest(UdpPlatoonMission	&mission);
	void udpConTest(UdpPlatoonConfig  &config);

	bool socketInit(int milliSecond);
    void udpThreadFunc();
	void udpTest();
	

    ros::Subscriber platoon_member_sub;
    ros::Subscriber platoon_config_sub;
    ros::Subscriber platoon_mission_sub;
	
	ros::Publisher platoon_config_pub;
	ros::Publisher platoon_mission_pub;
	ros::Publisher platoon_member_pub;
	
	UdpPlatoonConfig platoonConfig_;
	bool new_PlatoonConfig_flag;
	UdpPlatoonMember platoonMember_;
	bool new_PlatoonMember_flag;
	UdpPlatoonMission  platoonMission_;
	bool new_PlatoonMission_flag;

	std::vector<msgOpt> msgsFunc;
	
	common::PlatformParam platformParam;
	std::thread udp_thread_;
	NetInfo	 netInfo;
	std::shared_ptr<udpServer> udp_send_ptr;
	std::shared_ptr<udpServer> udp_recv_ptr;
};

#endif 
