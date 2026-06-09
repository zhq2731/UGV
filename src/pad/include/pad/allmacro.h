#ifndef ALLMACRO_H
#define ALLMACRO_H

#define CAN_NUM 2 /*CAN端口數量*/
#define PI 3.1415926
#define DEGREE_RADIAN PI / 180.0
#define RADIAN_DEGREE 180.0 / PI

/**********************************数据记录宏定义*************************************************/
#define RECORD_UNKNOWN 0 /*未知记录*/
#define RECORD_PROCESS 1 /*程序运行记录*/
#define RECORD_DATA 2    /*数据记录*/
#define RECORD_ERROR 3   /*错误记录*/

/**********************************************Interface
 * Macro***********************************************************/

#define PNC_IP "172.16.2.88"  /*决策控制单元*/
#define PNC_AUTODRIVING_PORT 8080 /*自动驾驶端口*/
#define PNC_CHASSIS_PORT 8080     /*底盘端口*/
#define PNC_PERCEPTION_PORT 8080  /*感知端口*/
// #define TANK_IP "127.0.0.1" /*任务下发端口（云控端口）*/
#define TANK_TASKPOINTS_PORT 9007 /*任务下发端口*/
#define TANK_REMOTE_PORT 9009 /*遥控设备端口  运动开始  运动停止   端口号*/
#define REMOTE_CONTROL_PORT 9006 /*遥控设备端口  运动开始  运动停止   端口号*/



#define PERCETION_IP "192.168.3.120" /*感知单元*/
#define PERCETION_PORT 9008    //9007

#define REMOTE_CONTROL_IP "172.16.2.8" /*远程遥控单元*/
#define REMOTE_CONTROL_CHASSIS_PORT 8080
#define REMOTE_CONTROL_AUTO_DRIVING_PORT 8080

#define PNC_TO_REMOTE_CONTROL_STATE_MSG 0x40B0
#define TANK_TO_REMOTE_CONTROL_STATE_MSG 0x8020
#define REMOTE_CONTROL_TO_PNC_CHASSIS_EVENT_MSG 0x3060

#define REMOTE_DRIVE_CYCLE_MSG 0x4010

#define REMOTE_CONTROL_TO_PNC_CHASSIS_CYCLE_MSG 0x50B0

#define PATH_TYPE_GLOBAL_ROUTE 0x01/*路径类型-全局路径*/
#define PATH_TYPE_LOCAL_PLAN   0x02/*路径类型-局部规划轨迹*/
/**********************************************Interface
 * Macro***********************************************************/

#define FP2VC_HEART_MSG 0x1010           /*心跳报文*/
#define FP2VC_MOVEPATH_BIND_MSG 0X1020   /*跟踪路径装订报文*/
#define FP2VC_COORDINATE_BIND_MSG 0x1030 /*底盘运动坐标系装订报文*/
#define FP2VC_MOVEPATH_ASK_MSG 0x1040    /*跟踪路径查询报文*/
#define FP2VC_COORDINATE_ASK_MSG 0x1050  /*底盘运动坐标系查询报文*/
#define FP2VC_ACK_MSG 0x10E0             /*接收确认报文*/
#define FP2VC_CHASEQCTRL_MSG 0x3050      /*底盘设备控制报文*/
#define FP2VC_MOVECTRL_MSG 0x3060        /*底盘运动控制报文*/

#define VC2FP_HEART_MSG 0x6010               /*心跳报文*/
#define VC2FP_MOVEPATH_BIND_ACK_MSG 0X6020   /*跟踪路径装订结果上传报文*/
#define VC2FP_COORDINATE_BIND_ACK_MSG 0x6030 /*底盘运动坐标系装订结果上传报文*/
#define VC2FP_ACK_MSG 0x60E0                 /*接收确认报文*/
#define VC2FP_ABNORMAL_MSG 0x60F0            /*异常信息报文*/
#define VC2FP_CANTRANS_MSG 0x8060            /*CAN命令应答和状态报文透传*/

#define VC2PERCPT_HEART_MSG 0xD010   /*心跳报文*/
#define VC2PERCPT_VEH_POS_MSG 0xD060 /*车辆位置信息报文*/
#define VC2PERCPT_ACK_MSG 0xD0E0     /*接收确认报文*/

#define PERCPT2VC_HEART_MSG 0xE010 /*心跳报文*/
#define PERCPT2VC_ACK_MSG 0xE0E0   /*接收确认报文*/

#define FP_EQ_STATE_ON_UNKONOWN 0 /*设备加电但状态未知*/
#define FP_EQ_STATE_ON_OK 1       /*设备正常*/
#define FP_EQ_STATE_ON_ERROR 2    /*设备故障*/
#define FP_EQ_STATE_OFF 3         /*设备断电*/

/**********************************************CAN使用宏定义***********************************************************/
/*CAN ID*/
#define CANID_P_ALARM 1    /*报警报文*/
#define CANID_P_CMD 2      /*命令报文*/
#define CANID_P_CMDACK 3   /*命令应答报文*/
#define CANID_P_STATE 4    /*状态信息报文*/
#define CANID_P_STATEACK 5 /*状态应答报文*/
#define CANID_P_WORKDATA 6 /*工作数据报文*/

#define CANID_ADDR_VC 0x1       /*综合控制设备*/
#define CANID_ADDR_CHASSIS 0x60 /*底盘设备*/

#define CANID_DP_SINGLE 0 /*单帧*/

#define CANID_PE_SINGLE 0    /*单帧*/
#define CANID_PE_MULTFIRST 1 /*多帧起始帧*/
#define CANID_PE_MULTMID 2   /*多帧中间帧*/
#define CANID_PE_MULTLST 3   /*多帧结束帧*/

#define CHAS_WORK_DATA_FRAME_NUM 5 /*底盘工作数据报文多帧数*/

#define CAN_EQ_STATE_OFF 0         /*设备断电*/
#define CAN_EQ_STATE_ON_OK 1       /*设备正常*/
#define CAN_EQ_STATE_ON_ERROR 2    /*设备故障*/
#define CAN_EQ_STATE_ON_UNKONOWN 3 /*设备加电但状态未知*/

/**********************************************CAN使用宏定义***********************************************************/
/*底盘使用宏定义*/
#define CMD_CHAS_BASIC_STATE_ASK 0x1    /*基本状态查询*/
#define CMD_CHAS_SELF_CHECK 0x3         /*自检命令*/
#define CMD_CHAS_GEAR 0x6               /*档位设置命令*/
#define CMD_CHAS_MOVE 0x7               /*运动控制命令*/
#define CMD_CHAS_VEH_BASIC_CTRL 0x8     /*车灯笛等附属件控制命令*/
#define CMD_CHAS_WORK_MODE_SET 0x9      /*被控模式命令*/
#define CMD_CHAS_PARKING_CTRL 0xA       /*驻车制动/解除驻车制动命令*/
#define CMD_CHAS_BRAKING_CTRL 0xB       /*紧急制动命令*/
#define CMD_CHAS_CLEAR_BRAK_CTRL 0xC    /*解除紧急制动命令*/
#define CMD_CHAS_ENGINE_START_CTRL 0x11 /*发动机启动命令*/
#define CMD_CHAS_ENGINE_END_CTRL 0x12   /*发动机停止命令*/

#define STATE_POWER_OFF 0xAA /*设备未加电*/
#define STATE_UNKNOWN 0x0    /*设备已加电，但状态未知*/
#define STATE_OK 0x0F        /*设备正常*/
#define STATE_ERROR 0xF0     /*设备故障*/

#define CHAS_EQ_STATE_OFF 0         /*设备断电*/
#define CHAS_EQ_STATE_ON_OK 1       /*设备正常*/
#define CHAS_EQ_STATE_ON_ERROR 2    /*设备故障*/
#define CHAS_EQ_STATE_ON_UNKONOWN 3 /*设备加电但状态未知*/

/*被控设备*/
#define EQUIP_SERVO 1    /*伺服*/
#define EQUIP_DETECTOR 2 /*光电*/
#define EQUIP_LAUNCH 3   /*发控*/
#define EQUIP_VEHCTRL 4  /*综合控制设备*/
#define EQUIP_CHASSIS 5  /*底盘*/

#define CHAS_WORK_MODE_MAN 0x1        /*人控模式*/
#define CHAS_WORK_MODE_AUTO_DRIVE 0x2 /*兼容模式*/

#define GEAR_D1 1 /*D1档*/
#define GEAR_D2 2 /*D2档*/
#define GEAR_D3 3 /*D3档*/
#define GEAR_D4 4 /*D4档*/
#define GEAR_D5 5 /*D5档*/
#define GEAR_D6 6 /*D6档*/
#define GEAR_D 7  /*D档*/
#define GEAR_R 8  /*R档*/
#define GEAR_N 9  /*N档*/

#define CHAS_PARK 0x55         /*驻车制动*/
#define CHAS_RELEASE_PARK 0xAA /*解除驻车制动*/

#define CHAS_EMERGENCY_BRAK 0x55     /*紧急制动中*/
#define CHAS_NOT_EMERGENCY_BRAK 0xAA /*无紧急制动*/

#define CHAS_MOVE_START 0x55 /*运动开始*/
#define CHAS_MOVE_STOP 0xAA  /*运动停止*/

#define SEND_MSG_WAIT_ACK 1   /*发送报文需等待命令应答*/
#define SEND_MSG_WAIT_STATE 2 /*发送报文需等待状态报文*/

#define SEND_MSG_RECV_NONE 0  /*发送未收到命令应答或状态报文*/
#define SEND_MSG_RECV_ACK 1   /*发送报文已收到命令应答*/
#define SEND_MSG_RECV_STATE 2 /*发送报文已收到状态报文*/

#define MOVE_MODE_INVALID 0     /*无效*/
#define MOVE_MODE_COURSE_KEEP 1 /*航向保持*/
#define MOVE_MODE_PATH_TRACK 2  /*路径跟踪*/
#define MOVE_MODE_REMOTE_CTRL 3 /*远程遥控*/
#define MOVE_MODE_BRAK 4        /*刹车控制*/

#define DISP_WARNING 1 /*显示警告信息*/
#define DISP_ERROR 2   /*显示错误信息*/
#define DISP_PROCESS 3 /*显示流程信息*/

/*Mems的INS状态宏定义*/
#define INS_STATE_INIT 0                 /*初始化*/
#define INS_STATE_COARSE_ALIGN 1         /*粗对准*/
#define INS_STATE_FINE_ALIGN 2           /*精对准*/
#define INS_STATE_ONE_ANTENA_LOCATE 3    /*单天线定位*/
#define INS_STATE_DOUBLE_ANTENA_LOCATE 4 /*双天线定位*/
#define INS_STATE_ONE_ANTENA_RTK 5       /*单天线差分*/
#define INS_STATE_MILEMETER_COMB 6       /*里程计组合(优先级低于GNSS组合)*/
#define INS_STATE_ONLY_IMU 8             /*纯惯*/
#define INS_STATE_ZERO_SPEED_REVISE 9    /*零速校正*/
#define INS_STATE_DOUBLE_ANTENA_RTK 0xB  /*双天线差分*/
#define INS_STATE_DYNAMIC_ALIGN 0xC      /*动态对准*/
#define INS_STATE_ERROR 0xF              /*系统异常*/

#define PAD2PNC_HEART_MSG 0x3060           /*心跳报文*/
#define PAD2PNC_MEMSUPSET_MSG 0x3010       /*Mems透传设置*/
#define PAD2PNC_CHASUPSET_MSG 0x3020       /*底盘透传设置*/
#define PAD2PNC_CTRL_MSG 0x3060            /*控制报文*/
#define PAD2TANK_TASK_MSG 0x4060            /*控制报文*/
#define PAD2TANK_REMOTE_MSG 0x4010            /*控制报文*/


#define PAD2PNC_MAP_ORIGIN_LOAD_MSG 0x4010 /*地图原点经纬高装订报文*/
#define PAD2PNC_PATH_LOAD_MSG 0x4020       /*全局路径装订报文*/
#define PAD2PNC_MAP_ORIGIN_ASK_MSG 0x4030  /*地图原点经纬高查询报文*/
#define PAD2PNC_PATH_ASK_MSG 0x4040        /*全局路径查询报文*/
#define PAD2PNC_ROUTE_REQUEST_MSG 0x4050   /*导航请求(路径规划)控制报文*/
#define PERC2VC_OBS_CLUSTER_MSG 0xE030     /*激光雷达障碍物聚类后的信息报文*/

#define REMOTE_CONTROL_TO_PNC_MOTION_CMD_MSG 0x50B0 /*遥控器发送至pad的运动命令报文*/
#define REMOTE_CONTROL_TO_PNC_CHASSIS_CMD_MSG 0x50C0 /*遥控器发送至pad的运动命令报文*/


#define PNC2PAD_HEART_MSG 0x6010              /*心跳报文*/
#define PNC2PAD_CANTRANSUP_MSG 0x8010         /*CAN报文透传上报报文*/
#define PNC2PAD_CHAS_MSG 0x8020               /*底盘数据报文*/
#define PNC2PAD_INS_MSG 0x8030                /*定位定向数据报文*/
#define TANK_Dilixinxi_MSG 0xE040                /*地理信息报文*/
#define TANK_Obstacle_MSG 0xE030              /*障碍物信息报文*/


#define PNC2PAD_MOVE_DATA_MSG 0x8050          /*决策控制数据上报报文*/
#define PNC2PAD_TRAJECTORY_UP_MSG 0x80B0      /*导航路径&预测轨迹上报报文*/
#define PNC2PAD_GLOBAL_UP_MSG 0x8040      /*导航路径&预测轨迹上报报文*/

#define PNC2PAD_PROCESS_INFO_MSG 0x80D0       /*流程信息上报报文*/
#define PNC2PAD_LANE_SEGMENT_ROUTE_MSG 0x80E0 /*路段规划上报报文*/

#define CMD_PNC_TRACK_AVOID_OBS 0x04     /*循迹避障控制*/
#define CMD_PNC_ONE_KEY_RETURN 0x09      /*一键返航*/
#define CMD_PNC_AEB 0x0B                 /*辅助紧急制动*/
#define CMD_PNC_CHASSIS_DIRECT_CTRL 0x0C /*底盘直接控制*/

#define NET_MSG_NOACK 0x00FF
#define NET_MSG_ACK 0xFF00
#endif  // ALLMACRO_H
