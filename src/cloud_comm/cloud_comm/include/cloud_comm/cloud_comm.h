#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>
#include <memory>
#include <vector>
#include <thread>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include "joystickserial.h"

#include "yaml-cpp/yaml.h"

//#include "localization_msgs/Localization.h"
#include "driver_msgs/MotionStartCmd.h"
#include "driver_msgs/ChassisReport.h"

#include "driver_msgs/ChassisCmd.h"
#include "driver_msgs/DriveCmd.h"
#include "driver_msgs/GearCmd.h"
#include "driver_msgs/ModeCmd.h"
#include "driver_msgs/LightHornWiperCmd.h"
#include "driver_msgs/ParkingBrakeCmd.h"
#include "driver_msgs/PowerCmd.h"
#include "driver_msgs/SteeringWheelCmd.h"
#include "driver_msgs/EstopCmd.h"
#include "configuration_msgs/ConfigurationDown.h"
#include "configuration_msgs/ConfigurationUp.h"
#include "localization_msgs/Localization.h"
#include "perception_msgs/PerceptionObstacles.h"
#include "perception_msgs/PredictionObstacles.h"
#include "errorcode_msgs/Errorcode.h"

#include <ros/package.h>
#include "udp/udpServer.h"
#include <amathutils_lib/amathutils.hpp>
#include "heartbeat_msgs/Heartbeat.h"
#include "platoon_msgs/PlatoonMission.h"
#include "platoon_msgs/PlatoonMember.h"
#include "error_code.h"
//#include "logger/logger.h"
#include "planning_msgs/TrajectoryPointArray.h"
#include <ctime>

#include "common/common.h"
#include "numeric"

//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PointStamped.h>
//#include "utm/UTM.h"

#include "udp/udp_multicast.h"
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "route_msgs/MultiPoint.h"
#include "route_msgs/Replan.h"
#include "route_msgs/InitPoint.h"
// #include "route_msgs/MapSign.h"
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <iomanip>


typedef double  float64;
typedef float  float32;
typedef int int32;
typedef unsigned int uint32;
typedef short int16;
typedef unsigned short uint16;
typedef unsigned char uint8;
typedef signed char int8;

#pragma pack(1)
struct MsgHeader
{
    uint16 msg_id;
	uint16 msg_length;
    uint32 msg_num;
	uint32 msg_time;
	uint32 msg_source_add;
	uint32 msg_destination_add;
	uint8  msg_vehicle_num;
	uint8  msg_remote_num;
	uint16 msg_flag;//顺序问题
	uint16 msg_reserve;
};

struct MsgEnd
{
    uint8 msg_reserve[3];
	uint8 msg_checksum;
};

struct XYZ{
	float x;
	float y;
	float z;
};

struct HardwareStatus{
	uint8 num_error;
	uint16 code_error[3];
};

struct SoftwareStatus{
	uint8 num_error;
	uint16 code_error[20];
};
struct Coordinate{
	uint8 coordinate_system;
	uint8 reserved;
	uint8 UTM;
	uint8 sign;
	float64 x;
	float64 y;
	float64 z;
};
struct Posture{
	float64 orientation;
	float64 pitch;
	float64 roll;
};

struct PathPoint
{
	uint8 coordinate;
	uint8 pathpoint_type;
	uint8 utm_num;
	uint8 south_flag;
	float64 x;
	float64 y;
	float64 z;
};

struct UdpMapSign{
	uint8 map_sign_;
	float64 origin_lat;
	float64 origin_lon;
	float64 origin_alt;
};

struct UdpPathInfo
{
	uint8 path_type;
	uint8 package_id;
	uint16 package_sum; 
	uint32 package_byte;
	uint16 package_num;
	uint16 current_package_byte;
	PathPoint path_point_list[20];
};

struct UdpTobePlanned{
	uint8 point_nums;
	PathPoint point_list[10];
};

struct UdpControlCommand
{
	uint8 control_vehicle_num;
	uint8 control_type;
	bool power_flag; 
	uint8 driving_mode;
	uint8 gear_position;
	uint8 turn_signal;
	uint8 emergency_light;
	bool emergency_brake_flag;
	float32 steering_target;
	float32 throttle;
	float32 brake;
	float32 speed;
	bool transport_flag;
	uint8 transport_type;
	uint8 task_num;
	float32 x;
	float32 y;
};

struct QueueConfig{
	int32 follow_type;
	float32 time_constant;
	float32 distance;
	float32 stop_distance;
	float32 diamond_distance;
	float32 deceleration;
};

struct QueueControlCommand{
	QueueConfig queueConfig;
	int32 command_type;
	uint8 vehicle_num;
	uint8 queue_order[6];
	uint8 platoon_count;
};

// struct UdpStatusFeedback
// {
// 	uint8 vehicle_id;
// 	uint8 start_status;
// 	uint8 driving_mode;
// 	uint8 gear_position;
// 	float32 speed;
// 	uint8 turn_signal_feedback;
// 	uint8 emergency_light_feedback;
// 	float32 steering_target_feedback;
// 	float32 throttle_feedback;
// 	float32 brake_feedback;
// 	uint8 chassis_fault_status;
// 	uint16 location;
// 	float32 oil;
// 	float32 mileage;
// 	uint8 gnss_status;
// 	float64 lon;
// 	float64 lat;
// 	float32 height;
// 	float32 heading;
// 	float32 pitch;
// 	float32 roll;
// };

struct UdpHeartFeedback{
	uint16 year;
	uint8 month;
	uint8 day;
	uint8 hour;
	uint8 minute;
	uint8 second;
	uint16 millisecond;
	uint8 control_status;
	uint8 reserved;
};


struct UdpChassisFeedback{
	uint16 code_error;
	uint8 error_status;
	// 底盘无人驾驶开关，00-关闭,01-闭合
	bool driving_mode_switch;
	// 驾驶模式反馈 0:人工, 1:无人
	bool driving_mode;
	// 自动驾驶模式细分
	bool autonomous_mode;
	float64 oil;
	float64 mile;
	// 发动机转速
	float64 EngineSpeedFbk;
	float64 EngineTorque;
	// 当前油门踏板 0-100% 
	float64 throttle_pedal;
	// 制动踏板位置
	float64 BrakePedalSTA;
	float64 max_speed;
	// 车速反馈 km/h
	float64 VehicleFbk;
	// 方向盘转角
	float64 SteerWheelFbk;
	uint8 steer_mode;
	// 驻车制动状态 0释放 1使能
	bool ParkStaFbk;
	// 当前档位  空挡：0 前进档：1-6 倒档：7 无效：8
	uint8 gear_location;
	bool position_light;
	bool left_light;
	bool right_light;
	bool rear_light;
	bool blackout_light;
	bool high_light;
	bool low_light;
	bool reverse_light;
	bool emergency_light;
	bool foggy_light;
	bool inner_light;
	bool air_horn;
	bool electric_horn;


	

// 	// 前桥转向模式 0x00-手动机械，10-手动助力 20-角度控制 41-人工干预
// 	uint8 FrontAxleStrModelSta;
// 	// 后桥转向模式 00-后桥锁止，01-后桥转向
// 	uint8 RearAxleStrModelSta;
// 	// 1桥转向角度
// 	float32 Axle1StrSta;
// 	// 4桥转向角度
// 	float32 Axle4StrSta;
// 	// 1桥右转向角度
// 	float32 Axle1RStrSta;
// 	// 方向盘力距
// 	float32 SteerTorFbk;
// 	// 转向系统故障码
// 	uint16 StrFaultCode;
// 	// 电动柱管故障码1
// 	uint8 CEPSFaultCode1;
// 	// 电动柱管故障码2
// 	uint8 CEPSFaultCode2;
// 	// 发动机控制器，00-离线,01-在线
// 	bool ECUOnlineSta;
// 	// 变速器控制器，00-离线,01-在线
// 	bool TCUOnlineSta;
// 	// EBS控制器，00-离线,01-在线
// 	bool EBSOnlineSta;
// 	// EPB控制器，00-离线,01-在线
// 	bool EPBOnlineSta;
// 	// 组合仪表，00-离线,01-在线
// 	bool DashOnlineSta;
// 	// 电动柱管控制器，00-离线,01-在线
// 	bool CEPSOnlineSta;
// 	// 信息管理控制器，00-离线,01-在线
// 	bool BMCOnlineSta;
// 	// 水温反馈
// 	float32 WaterTemFbk;
// 	// 机油压力反馈
// 	float32 OilPreSTA;
// 	// 进气加热指示灯，00-关闭 01-开启
// 	bool GasHeatSTA;
// 	// 燃油积水指示灯，00-关闭 01-开启
// 	bool FuelWaterSTA;
// 	// 环境温度
// 	float32 AmbTemFbk;
// 	// 分动器档位反馈  00：高档 01：低档 10：空档 
// 	int8 GearShiftFbk;
// 	// 变速器油温
// 	float32 ATFFbk;
// 	// 缓速器油温
// 	float32 ROTemFbk;
// 	// 缓速器状态
// 	float32 ROStaFbk;
// 	// 变速器当前控制模式 0有人 1无人
// 	uint8 GearBoxCurStaFbk;
// 	// 分动器当前档位  00：高档 01：低档 10：空档 
// 	uint8 TCaseCurStaFbk;
// 	// 分动器档位反馈，00未换档 01换档成功 10换档失败 11禁止换档
// 	uint8 TCaseGearFbk;
// 	// 分动器温度
// 	int8 TCaseTemFbk;
// 	// 一桥温度
// 	int8 Axle1TemFbk;
// 	// 二桥温度
// 	int8 Axle2TemFbk;
// 	// 三桥温度
// 	int8 Axle3TemFbk;
// 	// 四桥温度
// 	int8 Axle4TemFbk;

// 	// 后桥制动气压
// 	uint16 RearBraPre;
// 	//  前桥制动气压
// 	uint16 FrontBraPre;
// 	// 驻车气压
// 	float32 ParkPre;

// 	//驻车制动状态
// 	// 行车制动状态 0未制动 1行车制动
// 	bool BrakeStaFbk;
// 	// EPB红色故障灯 0灯灭 1灯亮
// 	bool EPBWarnLightRFbk;
// 	// EPB黄色故障灯 0灯灭 1灯亮
// 	bool EPBWarnLightYFbk;
// 	// ASR激活灯 0关闭 1激活
// 	bool ASRLight;
// 	// ASR功能 0开启 1关闭
// 	bool ASRFun;
// 	// ABS全面运转 0无意义 1全面运转
// 	bool ASRRun;
// 	// EBS红色故障灯 0灯灭 1灯亮
// 	bool EBSWarnLightRFbk;
// 	// EBS黄色故障灯 0灯灭 1灯亮
// 	bool EBSWarnLightYFbk;
// 	// zhidongdeng
// 	bool brake_light;
// 	// 防空制动灯状态  0关闭,1开启
// 	bool AirBrakeLampSTA;
// 	uint8 reserve[2];
};

struct UdpLocationFeedback{
	float64 latitude;
	float64 longitude;
	float64 altitude;
	float64 east_speed;
	float64 north_speed;
	float64 sky_speed;
	float64 origin_latitude_;
	float64 origin_longitude_;
	uint8 system_status;
	uint8 satellite_status = 1;   //1 2 3
	Coordinate coordinate_;
	Posture posture_;
	uint8   ins_state;
	uint8   gps_state;
	
};

struct UdpObstacleFeedback{
	uint8 bag_type;
	uint16 bag_sum;
	float32 bag_byte;
	uint16 bag_num;
	uint8 cur_obstacle_sum;
};

struct UdpConfigFeedback{
	uint8 config_type;
	uint8 lead_vehicle;
	uint8 follow_type;
	uint8 config_load;
	float32 config_speed;
	float32 config_space;
	uint32 config_status;
	uint8 reserved[4];
};

struct UdpPlatoonStateFeedback{
	float64 spacing;
	float64 x;
	float64 y;
	float64 orientation;
	float64 velocity;
	float64 acceleration;
	uint8 num;
	uint8 role;
	uint8 index;
	uint8 state;
	uint8 gear;
	uint8 driving_mode = 0;
	uint8 obstacle;
	uint8 reserved[2];
};



struct UdpChassisControl{
	// uint8 type; //0-无效 1-模式切换 2-发动机启停 3-急停 
	bool engine_on_off;
	bool steer_mode;
	bool parking_brake;
	uint8 gear_location;
	bool driving_mode;
	uint8 autonomous_mode;
	bool estop;
	float64 engine_torque_target;
	float64 brake_pedal;
	float64 steering_wheel_angle;
	float64 steering_wheel_angle_speed;
	float64 speed;
	bool position_light;
	bool left_light;
	bool right_light;
	bool high_light;
	bool low_light;
	bool foggy_light;
	bool reverse_light;
	bool blackout_light;
	bool air_horn;
	bool rear_light;
	bool emergency_light;
	bool inner_light;
	bool electric_horn;

};

struct UdpViewChange{
	uint8 video_mode;
	uint8 video_sign;
	uint8 upload_sign;
	uint16 video_compress;
	uint8 reserved[18];
};

struct UdpParamConfig{
	uint8 config_type;    //01-设置引导车，02-设置跟车方式，03-设置载重，04-设置速度，05-设置车间距，
	uint8 lead_vehicle;   //01-设置 02-取消
	uint8 follow_type;    //02-通信跟车，03-感知跟车
	uint8 config_load;
	float32 config_speed;
	float32 config_space;
	uint8 reserved[4];
};

struct SensorData{
	uint8 leftFrontCamera = 1;
	uint8 frontCamera = 1;
	uint8 rightFrontCamera;
	uint8 rightSideFrontCamera;
	uint8 rightSideBehindCamera = 1;
	uint8 behindCamera;
	uint8 leftSideBehindCamera = 1;
	uint8 leftSideFrontCamera;
	uint8 infraredCamera;
	uint8 frontLidar = 1;
	uint8 leftLidar;
	uint8 rightLidar;
	uint8 behindLidar;
	uint8 milimeterRadar;
	uint8 routing;
	uint8 reserved;
};

struct UdpSensorFeedback{
	SensorData sensorData;
	uint8 reserved[11];
};

struct UdpHeart
{
	uint8 equipment_status;
	uint8 msg_reserve[3];
};

struct FullUdpSensorFeedback{
	MsgHeader msgHeader_;
	UdpSensorFeedback udpSensorFeedback_;
	MsgEnd msgEnd_;
};

struct FullUdpHeartFeedback
{
	MsgHeader msgHeader_;
	UdpHeartFeedback  udpHeartFeedback_;
	MsgEnd 	  msgEnd_;
};

struct FullUdpChassisFeedback{
	MsgHeader msgHeader_;
	UdpChassisFeedback udpChassisFeedback_;
	MsgEnd msgEnd_;
};

struct FullUdpLocationFeedback{
	MsgHeader msgHeader_;
	UdpLocationFeedback udpLocationFeedback_;
	MsgEnd msgEnd_;
};

struct FullUdpPathInfo
{
	MsgHeader msgHeader_;
	UdpPathInfo udpPathInfo_;
	MsgEnd msgEnd_;
};

struct FullUdpConfigFeedback{
	MsgHeader msgHeader_;
	UdpConfigFeedback udpConfigFeedback_;
	MsgEnd msgEnd_;
};

struct FullUdpPlatoonStateFeedback{
	MsgHeader msgHeader_;
	UdpPlatoonStateFeedback udpPlatoonStateFeedback_;
	MsgEnd msgEnd_;
};


struct NetInfo
{
    std::string cloud_ip;
	int cloud_port;
	std::string multicast_ip;
	int multicast_port;
	std::string host_ip;
	int  host_port;
	int timeOutmilliSecond;
};

struct OriginInfo
{
	double origin_longitude;
	double origin_latitude;
	double origin_altitude;
};

struct NumInfo
{
	int vehicle_num;
	int remote_num;
};

struct TaskBlockPoint{
	double heading;
	double x;
	double y;
	double z;
};

struct UdpTaskBlockInfo
{
	uint8 path_type;
	uint8 package_id;
	uint16 package_sum; 
	uint32 package_byte;
	uint16 package_num;
	uint16 current_package_byte;
	TaskBlockPoint path_point_list[20];
};

struct ObstaclePoints{
	uint8 nums;
	XYZ points[5];
};

struct ObstacleData{
	uint8 type_status;
	uint16 id;
	uint8 type_recognition;
	ObstaclePoints obstaclePoints;
};

struct UdpObstaclesInfo{
	uint8 bag_type;
	uint16 bag_sum;
	uint32 bag_byte;
	uint16 bag_num = 0x04;
	uint8 cur_obstacle_sum;
	ObstacleData obstacleData[10];
};

struct FullObstaclesInfo{
	MsgHeader msgHeader_;
	UdpObstaclesInfo udpObstaclesInfo_;
	MsgEnd msgEnd_;
};

#pragma pack()

class CloudComm
{
public:
	typedef void(CloudComm::*msgOpt)(char data[]);
	CloudComm(ros::NodeHandle &nh);
	~CloudComm();
	YAML::Node config_;
private:
    ros::NodeHandle nh_;	
    ros::NodeHandle private_nh_;
	::common::PlatformParam platform_param;
	
    //void callbackCurrentPose(const localization_msgs::Localization::ConstPtr &msg);
    // void callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg);
    // void callbackMotionStart(const driver_msgs::MotionStartCmd::ConstPtr &msg);
	// void callbackTimerStatusFeedback(const ros::TimerEvent &event);
	// void callbackHeart(const driver_msgs::ChassisReport::ConstPtr &msg);
	// void callbackTimerHeart(const ros::TimerEvent &event);
    //   add new
	void callbackChassisStatus(const driver_msgs::ChassisReport::ConstPtr &msg);
	void callbackObstacle(const perception_msgs::PredictionObstacles::ConstPtr &msg);
	void callbackError(const errorcode_msgs::Errorcode::ConstPtr &msg);
	void callbackLocalization(const localization_msgs::Localization::ConstPtr &msg);
	void callbackConfig(const configuration_msgs::ConfigurationUp::ConstPtr &msg);
	void callbackPath(const planning_msgs::TrajectoryPointArray::ConstPtr &msg);
	void callbackPlatoonState(const platoon_msgs::PlatoonMember::ConstPtr &msg);

	void callbackTimerHeartFeedback(const ros::TimerEvent &event);
	void callbackTimerStatusFeedback(const ros::TimerEvent &event);
	void callbackTimerLocationFeedback(const ros::TimerEvent &event);
	void callbackTimerPlatoonFeedback(const ros::TimerEvent &event);
	void callbackTimerSensorFeedback(const ros::TimerEvent &event);
	void callbackTimerConfigFeedback(const ros::TimerEvent &event);

	// void heartbeatPub();

	void pointTobePlanned(char data[]);
	void pathInfoOpt(char data[]);
	void queueControl(char data[]);
	// void heartOpt(char data[]);
	// void controlCommandOpt(char data[]);
    
	 // add new
	void heartOpt(char data[]);
	void chassisControl(char data[]);
	void viewChange(char data[]);
	void paramConfig(char data[]);

	

	uint8 getCheckSum(uint8 *data,int size);
	uint32 Acquire24AbsTime();
	//void  saveData(planning_msgs::TrajectoryPointArray msgTrajectoryPointArray);
	
	// UdpStatusFeedback  msgLocalization2UdpStatus(const localization_msgs::Localization::ConstPtr           &msgLocalization);
	// UdpStatusFeedback  msgChassis2UdpStatus(const driver_msgs::ChassisReport::ConstPtr &msgChassisReport);
	// UdpStatusFeedback  msgMotionStart2UdpStatus(const driver_msgs::MotionStartCmd::ConstPtr &msgMotionStart);
	planning_msgs::TrajectoryPointArray	udpPath2MsgPath(const UdpPathInfo &udpPathInfo);              //有用




	driver_msgs::PowerCmd 			udpCmd2MsgPower(const UdpControlCommand &udpControlCommand);
	driver_msgs::ModeCmd 			udpCmd2MsgMode(const UdpControlCommand &udpControlCommand);
	driver_msgs::GearCmd 			udpCmd2MsgGear(const UdpControlCommand &udpControlCommand,driver_msgs::ParkingBrakeCmd &msgParkingcmd);
	driver_msgs::LightHornWiperCmd 	udpCmd2MsgLight(const UdpControlCommand &udpControlCommand);
	driver_msgs::EstopCmd 			udpCmd2MsgEstop(const UdpControlCommand &udpControlCommand);
	driver_msgs::SteeringWheelCmd 	udpCmd2MsgSteering(const UdpControlCommand &udpControlCommand);
	driver_msgs::DriveCmd 			udpCmd2MsgDrive(const UdpControlCommand &udpControlCommand);
	std_msgs::Float32 				udpCmd2MsgSpeed(const UdpControlCommand &udpControlCommand);

	bool socketInit(int milliSecond);
    void udpThreadFunc();
	void serialThreadFunc();
	// void logger();
	
    ros::Subscriber current_pose_sub_;
    ros::Subscriber chassis_sub_;
	ros::Subscriber motion_start_sub_;
	ros::Subscriber heart_sub_;

	ros::Subscriber chassis_status_feedback_;
	ros::Subscriber obs_feedback_;
	ros::Subscriber error_feedback_;
	ros::Subscriber localization_feedback_;
	ros::Subscriber configuration_feedback_;
	ros::Subscriber path_feedback_;
	ros::Subscriber platoon_state_;
	ros::Subscriber platoon_state_self;
    
	ros::Publisher power_cmd_pub_;
	ros::Publisher mode_cmd_pub_;
	ros::Publisher gear_cmd_pub_;
	ros::Publisher light_cmd_pub_;
	ros::Publisher steering_cmd_pub_;
	ros::Publisher drive_cmd_pub_;
	ros::Publisher parking_cmd_pub_;
	ros::Publisher estop_cmd_pub_;
	ros::Publisher configuration_pub_;
	ros::Publisher heart_pub_;
	ros::Publisher path_info_pub_;
	ros::Publisher points_planned_pub_;
	ros::Publisher chassis_cmd_pub_;
	ros::Publisher speed_cmd_pub_;
	ros::Publisher platoon_control_pub_;
	ros::Publisher motion_cmd_pub_;
	ros::Publisher cruise_cmd_pub_;
	

	ros::Timer timer_status_;
	ros::Timer timer_heart_;
	ros::Timer timer_location_;
	ros::Timer timer_obstacle_;
	ros::Timer timer_platoonState_;
	ros::Timer timer_sensorStatus_;




	std::thread udp_thread;
	std::thread serial_thread;
	
	int SocketHandle;
	sockaddr_in addr_host;
	sockaddr_in server_addr;
	NetInfo	 netInfo;
	NumInfo numInfo;
	NumInfo numPlatoon;
	OriginInfo originInfo;
	std::shared_ptr<udpServer> udp_send_ptr;
	std::shared_ptr<udpServer> udp_recv_ptr;
	std::shared_ptr<MulticastSender> udp_send_ptr_multi;
	std::shared_ptr<MulticastReceiver> udp_recv_ptr_multi;

	char recvBuf[1024];
	char sendBuf[1024];
	
	//UdpStatusFeedback 	udpStatusFeedback;
	UdpHeartFeedback udpHeartFeedback;
	planning_msgs::TrajectoryPointArray msgTrajectoryPointArray;

	
	//nav_msgs::Odometry odom;

	UdpChassisFeedback udpChassisFeedback;
	UdpLocationFeedback udpLocationFeedback;
	UdpConfigFeedback udpConfigFeedback;
	UdpPathInfo udpPathInfo;
	UdpSensorFeedback udpSensorFeedback;

	std::map<int,msgOpt> msgsFunc;
	
	bool path_back_flag;
	bool path_pub_flag;
	double endPointX;
	double endPointY;
	double endPointZ;

	std::vector<XYZ> obs_points;
	


	int count = 0;
	int32 msg_num_;
	vector<unsigned short> soft_errcode;
	vector<unsigned short> hard_errcode;
	std::vector<uint16> error_code_ = {0x0001,0x0002};
	JoyStickSerial serial_node;

	bool heartFeedbackFlag  = false;
	bool statusFeedbackFlag = false;
	bool locationFeedbackFlag = false;
	bool platoonFeedbackFlag = false;
	bool sensorFeedbackFlag = false;
	bool configFeedbackFlag = false;

	std::map<int, UdpPlatoonStateFeedback>    platoonFeedBack;  
	std::map<int, bool> infos;  

	//department two display
	route_msgs::MultiPoint msgTaskPointArray;
	route_msgs::MultiPoint udpTask2MsgTask(const UdpTaskBlockInfo &udpTaskBlockInfo);
	void multipleTaskPoints(char data[]);
	bool task_pub_flag;

	UdpPathInfo udpReferenceInfo;


	route_msgs::Replan msgBlockPointArray;
	route_msgs::Replan udpBlock2MsgBlock(const UdpTaskBlockInfo &udpTaskBlockInfo);
	void blockPoints(char data[]);
	bool block_pub_flag;

	perception_msgs::PredictionObstacles msgObstacleArray;
	bool obs_pub_flag;

	void mapSign(char data[]);

	void satState(char data[]);

	void obstacleOpt(char data[]);

	void initPoint(char data[]);

	void mapRequest(char data[]);

	void callbackOrigin(const geometry_msgs::Point::ConstPtr &msg);

	void callbackRequest(const std_msgs::UInt8::ConstPtr &msg);

	void callbackReference(const planning_msgs::TrajectoryPointArray::ConstPtr &msg);
	void callbackTrajectory(const planning_msgs::TrajectoryPointArray::ConstPtr &msg);

	UdpPathInfo udpTrajectoryInfo;

	ros::Publisher multiple_point_pub_;
	ros::Publisher block_point_pub_;
	ros::Publisher map_sign_pub_;
	ros::Publisher satellite_mode_pub_;
	ros::Publisher obs_pub_;
	ros::Publisher initPoint_pub_;
	
	ros::Subscriber map_request_;
	ros::Subscriber origin_feedback_;
	ros::Subscriber reference_feedback_;
	ros::Subscriber trajectory_feedback_;

};

