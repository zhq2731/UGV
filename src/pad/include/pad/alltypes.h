#ifndef ALLTYPES_H
#define ALLTYPES_H
#include <string>
typedef unsigned char UINT8;
typedef signed char INT8;
typedef unsigned short int UINT16;
typedef short int INT16;
typedef unsigned int UINT32;
typedef int INT32;

typedef unsigned short int uint16;
typedef unsigned char uint8;
typedef unsigned int uint32;
typedef bool boolean;

typedef unsigned char u_8;
typedef unsigned short u_16;
typedef unsigned int u_32;
typedef signed int s_32;
typedef unsigned long u_long;

#pragma pack(1) /*1字节对齐*/

typedef struct
{
  uint8 left_light;
  uint8 right_light;
  uint8 high_light;
  uint8 low_light;

  uint8 brake_light;
  uint8 emergency_light;

  uint8 front_foggy_light;
  uint8 rear_foggy_light;
  uint8 position_light;
  uint8 reverse_light;

  uint8 horn;
  uint8 wiper;
  uint8 head_light;
} ChassisLightHornWiper;

// 星网宇达5651类型
typedef struct
{
  double fTimeStampS;
  //@verbatim (language="comment",  text="时间戳_纳秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampNs;
  uint16 combined_week_time;
  double combined_week_second;
  uint16 utc_year;
  uint8 utc_month;
  uint8 utc_day;
  uint8 utc_hour;
  uint8 utc_minute;
  uint8 utc_second_int_part;
  float utc_second_float_part;
  double latitude;
  double longitude;
  float height;
  float height_ellipsoid;
  float east_speed;
  float north_speed;
  float up_speed;
  float ground_speed;
  float angle_heading;
  float angle_pitch;
  float angle_roll;
  float latitude_sigma;
  float longitude_sigma;
  float height_sigma;
  float east_speed_sigma;
  float north_speed_sigma;
  float up_speed_sigma;
  float angle_heading_sigma;
  float angle_pitch_sigma;
  float angle_roll_sigma;
  uint8 combined_navigation_state;
  float dmi_state;
  uint8 gps_state;
  uint8 external_gear;
  float external_speed;
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  uint16 gps_week;
  float gps_second;
  uint32 gps_utc_date;
  float gps_utc_time;
  float gps_latitude;
  float gps_longtitude;
  float gps_height;
  float gps_height_ellipsoid;
  float gps_east_speed;
  float gps_north_speed;
  float gps_up_speed;
  float gps_ground_speed;
  float gps_angle_heading;
  float gps_angle_pitch;
  float gps_base_line;
  float gps_track_angle;
  uint8 front_star;
  uint8 back_star;
  float gps_hdop;
  uint16 gps_age;
  uint16 gps_base_id;
  float navUncertainty;
  float ellip_height;
} XW5651ST;

typedef struct
{
  uint8 heart_beat;
  uint8 flag;
} HeartBeatST;

typedef struct
{
  std::string utc;
  double time_unix;
  bool is_valid;
  uint16 gps_week;
  double gps_time;
  double latitude;
  double longitude;
  float altitude;
  float ellip_height;
  float east_speed;
  float north_speed;
  float up_speed;
  float ground_speed;
  float angle_heading;
  float angle_pitch;
  float angle_roll;
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  double utm_x;
  double utm_y;
  double utm_z;
  uint8 main_star_num;
  uint8 aux_star_num;
  uint8 main_star_num_searching;
  uint8 aux_star_num_searching;
  float nav_uncertainty;
  uint8 satellite_status; // 卫星状态 0 不定位不定向 1 单点定位定向 2伪距差分定位定向  3 组合推算 4 RTK固定解 5 RTK浮点解 6 单点定位不定向 7 伪距差分定位不定向 8 RTK稳定解定位不定向 9 RTK浮点解定位不定向
  uint8 system_state;     // 系统状态 0 初始化 1 卫星导航 2 组合导航 3 纯惯性
} GNSS_IMU_ST;

// ABCDE底盘反馈状态类型
typedef struct
{
  bool ReadySts;
  uint8 DriveModeSts;
  uint8 FaultLevl;
  uint8 EBSFaultLevl;
  uint8 GearLeverLocation;
  uint8 GearCurrentLocation;
  float BrakePedalPosition;
  float AccPedalPosition;
  uint8 AccPedalFault;
  float VehicleSpeed;
  uint8 DriveModeSwitchSts;
  uint8 VehicleFaultCode;
  uint8 LifeCounter;

  float SteeringWheelAngle;
  float SteeringWheelAngleSpeed;
  uint8 SteeringFaultLevl;
  float SteeringAngleLimitLeft;
  float SteeringAngleLimitRight;
  boolean SteeringMode;
  boolean ParkBrakeSts;
  boolean ParkBrakeMode;
  uint8 ParkBrakeFaultLevl;
  boolean EmcyBarkeSts;

  boolean LeftLightSts;
  boolean RightLightSts;
  boolean EmcyFlasherSts;
  boolean LowBeamSts;
  boolean HighBeamSts;
  boolean HonkSts;
  boolean EmcyBrakeSwitchSts;
  float BrakeAirPressure;
  float AirTankPressure;
  uint16 Range;
  uint16 FuelCapacity;
  float HPVehicleSpeed;
  float EngineSpeed;
  float EnginePercentTroque;
  float TransmissionOutputShaftSpeed;
  float TransmissionOutputTroque;
  float Steer_axle_speed;
  float Rel_speed_steer_axle_left;
  float Rel_speed_steer_axle_right;
  float Rel_speed_rear_axle_left;
  float Rel_speed_rear_axle_right;
} VehicleABCDEChassisStateST;

// lingtuqingka
typedef struct
{
  uint8 driving_mode;
  uint8 mode_flag ;
  uint8 adu_mode;
  uint8 eps_mode;
  uint8 epb_mode;
  uint8 bcm_mode;
  uint8 xbr_mode;
  uint8 xbr_active_mode;
  uint8 xbr_system_state;
  uint8 auto_to_manual_tips;
  uint8 steer_mode;
  float steering_wheel_angle;
  float steering_wheel_angle_speed;
  float steering_wheel_hand_moment_signal;
  float front_wheel_angle;
  float rear_wheel_angle;
  float throttle_pedal;
  float brake_pedal;
  bool brake_pedal_manual;
  bool throttle_pedal_manual;
  // 当前档位 空挡：0 前进档：1 - 6 倒档：7 无效：8
  uint8 gear_location;
  uint8 parking_brake;
  uint8 epb_state;
  uint8 parking_brake_switch;
  bool emergency_stop;
  uint8 low_voltage_signal;
  uint8 high_voltage_signal;
  // #当前车速 单位：m / s
  float current_velocity;
  // #当前加速度 单位：m / s ^ 2
  float current_acceleration;
  // #左前轮轮速 单位：m / s
  float wheel_speed_fl;
  // #右前轮轮速 单位：m / s
  float wheel_speed_fr;
  // #左后轮轮速 单位：m / s
  float wheel_speed_rl;
  // #右后轮轮速 单位：m / s
  float wheel_speed_rr;
  // #发动机当前转速 单位：rpm
  float current_engine_speed;
  // #发动机当前扭矩 单位：N *m
  float current_engine_torque;
  // #发送机温度
  float current_engine_temperature;
  // #行驶里程
  float total_kilometres;
  // #剩余电量
  float remaining_electricity;
  // #剩余油量
  float remaining_oil;
  // #左转向灯状态 true : 开, false : 关
  bool left_light;
  // #右转向灯状态 true : 开, false : 关
  bool right_light;
  // #远光灯状态 true : 开, false : 关
  bool high_light;
  // #近光灯状态 true : 开, false : 关
  bool low_light;
  // #刹车灯状态 true : 开, false : 关
  bool brake_light;
  // #危险报警灯状态 true : 开, false : 关
  bool emergency_light;
  // #雾灯状态 true : 开, false : 关
  bool front_foggy_light;
  bool rear_foggy_light;
  // #喇叭 true : 开, false : 关
  bool horn;
  bool position_light;
  bool wiper;
  bool reverse_light;
  bool auto_switch;
  // 自动驾驶开关
  bool brake_intervene;
  // 人为刹车干预
  bool estop_intervene;
  // 急停按钮
  bool steer_intervene;
  // 超时状态
  bool timeout_status;
  // 远程按钮状态
  uint8 remote_button_status;

  bool is_ready;

} VehicleChassisStateST;

// 车辆运动状态
typedef struct
{
  double x;
  double y;
  double z;
  double theta;
  double dtheta;
  double v;
  double a;
  double kappa;
  double dkappa;
  double a_n;
  double gps_lat;
  double gps_lon;
  double gps_alt;
} VehicleMotionStateST;

/*Mems数据*/
typedef struct
{
  double fLatitude; /*组合纬度*/
  double fLongitude;
  float fHeight;
} LongLatHeightST; /*经纬高*/

typedef struct
{
  double x;
  double y;
  double z; /*84坐标系*/
} WGS84CorST;

typedef struct
{
  double x;
  double y;
  double z; /*高斯坐标系*/
} GaoSiCorST;

typedef struct
{
  double fTime;
  double x;
  double y;
  double z;
} ENUCorST; /*世界坐标系*/

typedef struct
{
  ENUCorST stENUPoint; /*路径点*/
  double fAngle2X_Rad; /*与世界坐标系X轴的夹角，单位rad，-π~+π*/
  double fCurve;       /*曲率*/
  double fCurveDiff;   /*曲率导数*/
  double fs;           /*路径累计s*/
} SmoothPathPointST;   /*经过平滑后的路径点结构体*/

typedef struct
{
  SmoothPathPointST stLocalPath;
  double fSpeed;
  double fAcc;
  double fTime;
} LocalPathPointST; /*局部路径平滑*/

typedef struct
{
  double lat_error;
  double angle_error_deg;
  double throttle_cmd;
  double de_acc_cmd;
  double steer_angle_cmd;
  double steer_angle_speed_cmd;
  double front_angle_cmd;

  double vehicle_v;
  uint8 vehicle_gear;
  boolean park_state;
  boolean engine;

  double vehicle_x;
  double vehicle_y;
  double vehicle_theta;
  double vehicle_kappa;

  double pp_look_ahead_dist;
  double pp_look_ahead_point_index;
  uint8 controller_state;
} ControlErrorST; /*局部路径平滑*/

typedef struct
{
  UINT16 usGPSWeek;  /*GPS周*/
  float fGPSWeekSec; /*GPS周秒*/

  UINT16 usUTCYear;  /*UTC年*/
  UINT8 ucUTCMonth;  /*UTC月*/
  UINT8 ucUTCDay;    /*UTC日*/
  UINT8 ucUTCHour;   /*UTC时*/
  UINT8 ucUTCMinute; /*UTC分*/
  UINT8 ucUTCSecInt; /*UTC整秒*/
  UINT8 ucUTCSecDOt; /*UTC秒小数部分，1LSB = 0.01s*/

  double fLatitude;  /*组合纬度*/
  double fLongitude; /*组合经度*/

  float fHeight;          /*组合高度*/
  float fHeightEllipsoid; /*海拔椭球高度差*/

  float fEastSpeed;   /*东向速度*/
  float fNorthSpeed;  /*北向速度*/
  float fSkySpeed;    /*天向速度*/
  float fGroundSpeed; /*水平地速*/

  float fAzim;  /*组合航向*/
  float fPitch; /*组合俯仰*/
  float fRoll;  /*组合横滚*/

  UINT8 ucState;        /*组合状态*/
  UINT8 ucDMIState;     /*DMI状态*/
  UINT8 ucGPSState;     /*GPS状态*/
  UINT8 ucExternGear;   /*外部输入档位*/
  UINT32 uiExternSpeed; /*外部输入车速*/

  double fActX;  /*X轴加计*/
  double fActY;  /*Y轴加计*/
  double fActZ;  /*Z轴加计*/
  double fGyroX; /*X轴陀螺*/
  double fGyroY; /*Y轴陀螺*/
  double fGyroZ; /*Z轴陀螺*/

  INT16 sLine;       /*GPS基线*/
  UINT8 ucFrontStar; /*前天线星数*/
  UINT8 ucBackStar;  /*后天线星数*/
} INS2VCDataST;      /*惯导组合导航数据*/

typedef struct
{
  UINT16 usUTCYear;  /*UTC年*/
  UINT8 ucUTCMonth;  /*UTC月*/
  UINT8 ucUTCDay;    /*UTC日*/
  UINT8 ucUTCHour;   /*UTC时*/
  UINT8 ucUTCMinute; /*UTC分*/
  UINT8 ucUTCSecInt; /*UTC整秒*/
  UINT8 ucUTCSecDOt; /*UTC秒小数部分，1LSB = 0.01s*/

  UINT8 ucINSState; /*导航状态*/
  UINT8 ucGPSState; /*GPS状态*/

  INT32 iHeight;    /*车辆高程*/
  INT32 iLongitude; /*车辆经度*/
  INT32 iLatitude;  /*车辆纬度*/

  UINT32 uiVehCourse; /*车辆航向*/
  INT32 iVehPitch;    /*车辆俯仰角*/
  INT32 iVehRoll;     /*车辆横滚角*/

  UINT16 usGroundSpeed; /*水平地速*/
  INT16 sAngleSpeed;    /*Z轴角速度*/

  INT16 sLine;       /*GPS基线*/
  UINT8 ucFrontStar; /*前天线星数*/
  UINT8 ucBackStar;  /*后天线星数*/

  INT32 iActX;      /*X轴加计*/
  INT32 iActY;      /*Y轴加计*/
  INT32 iActZ;      /*Z轴加计*/
  INT32 iGyroX;     /*X轴陀螺*/
  INT32 iGyroY;     /*Y轴陀螺*/
  INT32 iGyroZ;     /*Z轴陀螺*/
} INS2VCWorkDataST; /*惯导发向综合控制设备的工作数据*/

typedef struct _INSWeekSec
{
  UINT16 usGPSWeek;    /*GPS周*/
  UINT32 uiGPSWeekSec; /*GPS周秒*/
} INSWeekSecST;        /*组合导航输出GPS周秒*/

typedef struct _INSUTCTime
{
  UINT16 usUTCYear;  /*UTC年*/
  UINT8 ucUTCMonth;  /*UTC月*/
  UINT8 ucUTCDay;    /*UTC日*/
  UINT8 ucUTCHour;   /*UTC时*/
  UINT8 ucUTCMinute; /*UTC分*/
  UINT8 ucUTCSecInt; /*UTC整秒*/
  UINT8 ucUTCSecDOt; /*UTC秒小数部分，1LSB = 0.01s*/
} INSUTCTimeST;      /*组合导航输出UTC时间*/

typedef struct _INSLongLat
{
  INT32 iLatitude;  /*组合纬度*/
  INT32 iLongitude; /*组合经度*/
} INSLongLatST;     /*组合导航输出经纬度*/

typedef struct _INSHeight
{
  INT32 iHeight;          /*组合高度*/
  INT32 iHeightEllipsoid; /*海拔椭球高度差*/
} INSHeightST;            /*组合导航高度*/

typedef struct _INSSpeed
{
  INT16 sEastSpeed;     /*东向速度*/
  INT16 sNorthSpeed;    /*北向速度*/
  INT16 sSkySpeed;      /*天向速度*/
  UINT16 usGroundSpeed; /*水平地速*/
} INSSpeedST;           /*组合速度*/

typedef struct _PosDeviation
{
  UINT16 usLatitudeDevia;  /*组合纬度标准差*/
  UINT16 usLongitudeDevia; /*组合经度标准差*/
  UINT16 usHeightDevia;    /*组合高度标准差*/
} INSPosDeviationST;       /*组合位置标准差*/

typedef struct _SpeedDevialtion
{
  UINT16 usEastSpeedDevia;  /*组合东速标准差*/
  UINT16 usNorthSpeedDevia; /*组合北速标准差*/
  UINT16 usSkySpeedDevia;   /*组合天速标准差*/
} INSSpeedDevialtionST;     /*组合速度标准差*/

typedef struct _AngleDevialtion
{
  UINT16 usAzimDevia;   /*组合航向标准差*/
  UINT16 usPitchDevia;  /*组合俯仰标准差*/
  UINT16 usRollDevia;   /*组合横滚标准差*/
} INSAngleDevialtionST; /*组合航姿标准差*/

typedef struct _INSState
{
  UINT8 ucState;        /*组合状态*/
  UINT8 ucDMIState;     /*DMI状态*/
  UINT8 ucGPSState;     /*GPS状态*/
  UINT8 ucExternGear;   /*外部输入档位*/
  UINT32 uiExternSpeed; /*外部输入速度*/
} INSStateST;           /*组合导航状态位*/

typedef struct _IMU1
{
  INT32 iActX; /*X轴加计*/
  INT32 iActY; /*Y轴加计*/
} IMU1ST;      /*IMU输出*/

typedef struct _IMU2
{
  INT32 iActZ;  /*Z轴加计*/
  INT32 iGyroX; /*X轴陀螺*/
} IMU2ST;       /*IMU输出*/

typedef struct _IMU3
{
  INT32 iGyroY;
  INT32 iGyroZ;
} IMU3ST; /*IMU输出*/

typedef struct _GPSWeekSec
{
  UINT16 usGPSWeek;    /*GPS周*/
  UINT32 uiGPSWeekSec; /*GPS周秒*/
} GPSWeekSecST;        /*GPS输出GPS周秒*/

typedef struct _GPSUTCTime
{
  UINT32 usDate; /*UTC日期*/
  UINT32 usTime; /*UTC时间*/
} GPSUTCTimeST;  /*GPS输出GPS周秒*/

typedef struct _GPSLongLat
{
  INT32 uiLatitude;  /*纬度*/
  INT32 uiLongitude; /*经度*/
} GPSLongLatST;      /*GPS输出经纬度*/

typedef struct _GPSHeight
{
  INT32 iHeight;          /*组合高度*/
  INT32 iHeightEllipsoid; /*海拔椭球高度差*/
} GPSHeightST;            /*GPS高度*/

typedef struct _GPSSpeed
{
  INT16 sEastSpeed;     /*东向速度*/
  INT16 sNorthSpeed;    /*北向速度*/
  INT16 sSkySpeed;      /*天向速度*/
  UINT16 usGroundSpeed; /*水平地速*/
} GPSSpeedST;           /*GPS速度*/

typedef struct _GPSAngle
{
  UINT16 usAzim;         /*GPS航向*/
  INT16 sPitch;          /*GPS俯仰*/
  INT16 sLine;           /*GPS基线*/
  UINT16 usGPSSpeedAzim; /*GPS航迹角*/
} GPSAngleST;            /*GPS航姿*/

typedef struct _GPSStar
{
  UINT8 ucFrontStar;  /*前天线星数*/
  UINT8 ucBackStar;   /*后天线星数*/
  UINT16 usGPSHdop;   /*GPShdop*/
  UINT16 usGPSAge;    /*GPS差分age*/
  UINT16 usGPSBaseID; /*GPS差分基站ID*/
} GPSStarST;          /*GPS星数*/

typedef struct _CANID
{
  UINT8 PE2 : 2;   /*结束标志*/
  UINT8 DP_L6 : 6; /*数据页*/

  UINT8 DP_H2 : 2; /*数据页*/
  UINT8 DA_L6 : 6; /*目的地址*/

  UINT8 DA_H2 : 2; /*目的地址*/
  UINT8 SA_L6 : 6; /*源地址*/

  UINT8 SA_H2 : 2; /*源地址*/
  UINT8 P3 : 3;    /*优先级*/
  UINT8 : 3;
} CANIDST;

typedef union
{
  CANIDST stCanID;
  UINT32 uiCANID;
} CANIDUN;

/**
 * struct can_frame - basic CAN frame structure
 * @can_id:  CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @can_dlc: frame payload length in byte (0 .. 8) aka data length code
 *           N.B. the DLC field from ISO 11898-1 Chapter 8.4.2.3 has a 1:1
 *           mapping of the 'data length code' to the real payload length
 * @__pad:   padding
 * @__res0:  reserved / padding
 * @__res1:  reserved / padding
 * @data:    CAN frame payload (up to 8 byte)
 */
typedef struct canframe
{
  CANIDUN unCANID; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  UINT8 ucDlc;     /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  UINT8 __pad;     /* padding */
  UINT8 __res0;    /* reserved / padding */
  UINT8 __res1;    /* reserved / padding */
  UINT8 data[8];
} CANFrameST;

typedef struct
{
  UINT8 ucVersionSecond : 5; /*0～26,对应A～Z*/
  UINT8 ucVersionFirst : 3;  /*1~4*/
  UINT8 ucVersionThird;      /*00~99*/
} SoftVersionST;             /*软件版本*/

typedef union
{
  SoftVersionST stSoftVersion; /*软件版本*/
  UINT16 usSoftVersion;
} SoftVersionUN; /*软件版本*/

typedef struct
{
  UINT8 ucHour; /*命令发出时间*/
  UINT8 ucMinute;
  UINT8 ucSecond;
  UINT16 usMSecond;
  UINT8 ucCmdType;        /*控制命令*/
  UINT8 ucGear;           /*档位命令值*/
  UINT16 usVehSpeed;      /*车速命令*/
  INT16 sTurnAngle;       /*前轮转角命令*/
  UINT8 ucTurnAngleSpeed; /*前轮转角速度命令*/
  UINT8 ucLight;          /*车灯控制*/
  UINT8 ucHorn;           /*车笛控制*/
  UINT8 ucWorkMode;       /*被控模式标识*/
  UINT8 ucPark;           /*驻车制动标识*/
} VC2ChasCmdST;           /*综控发向底盘的命令*/

typedef struct _NetHeader
{
  UINT16 usMsgType; /*报文信息标识*/
  UINT16 usMsgLen;  /*报文长度*/
  UINT32 uiMsgNo;   /*报文序号*/
  UINT32 uiMsgTime; /*数据产生时刻*/
  UINT32 uiSrcIP;   /*信源设备标识*/
  UINT32 uiDestIP;  /*信宿设备标识*/
  UINT16 usAck;     /*确认标志*/
  UINT16 usReserve; /*保留字*/
} NetHeaderST;      /*以太网帧头*/

typedef struct _NetEnd
{
  UINT16 Reserve;
  UINT8 ReserveByte;
  UINT8 ucCheckSum; /*校验和*/
} NetEndST;         /*以太网帧尾*/

/*无人驾驶台发向综控数据*/
typedef struct _FP2VCHeartMsg
{
  NetHeaderST stNetHeader;

  UINT8 ucEquipState; /*设备状态*/
  UINT8 Reserve[5];   /*保留字节*/

  NetEndST stNetEnd;
} FP2VCHeartMsgST; /*无人驾驶台发向综合控制设备的心跳报文*/

typedef struct _FP2VCPathBindMsg
{
  NetHeaderST stNetHeader;

  UINT8 ucFrameMultiFlag;           /*多包标识*/
  UINT8 Reserve;                    /*保留字节，凑偶数字节*/
  UINT16 usFrameNum;                /*数据包数*/
  UINT32 uiTotalByteNum;            /*装订数据总字节数*/
  UINT16 usFrameNo;                 /*装订包序号*/
  UINT16 usFrameDataBytes;          /*当前包内装订字节数*/
  SmoothPathPointST stPathData[20]; /*路径数据*/

  NetEndST stNetEnd;
} FP2VCPathBindMsgST; /*无人驾驶台发向综合控制设备的路径装订报文*/

typedef struct
{
  //@verbatim (language="comment",  text="坐标系类型 0：未知 1：车体坐标系 2：WGS84坐标系 3：UTM坐标系 4：局部坐标系")
  uint8 emCoordinate;
  //@verbatim (language="comment",  text="保留填0，凑偶数字节使用")
  uint8 Reserve;
  //@verbatim (language="comment",  text="UTM时区 [1..60]")
  uint8 ucUTMZoneID;
  //@verbatim (language="comment",  text="南半球标志，1：南半球 0：北半球")
  uint8 bIsSouth;
  //@verbatim (language="comment",  text="UTM位置坐标")
  float fX;
  float fY;
  float fZ;
} GlobalPositionST;

typedef struct
{
  double lon;
  double lat;
} localPositionST; // 全局路径报文结构体

typedef struct
{
  double fX;
  double fY;
  double fZ;
  float heading;
  uint8 type;
  uint8 reserve[3];
} DilixinxiST; // 地理信息数据报文结构体

typedef struct
{
  float fX_1, fY_1, fZ_1;
  float fX_2, fY_2, fZ_2;
  float fX_3, fY_3, fZ_3;
  float fX_4, fY_4, fZ_4;
  uint16 id;
  uint8 type;
  uint8 reserve;
} ObstacleST; // 障碍物数据报文结构体

typedef struct
{
  NetHeaderST stNetHeader;
  UINT8 ucFrameMultiFlag;        /*多包标识*/
  UINT16 usFrameNum;             /*数据包数*/
  UINT16 usFrameNo;              /*装订包序号*/
  UINT8 ObstacleNum;             /*障碍物个数*/
  ObstacleST stObstacleData[10]; /*障碍物数据*/
  uint16 reserve;
  NetEndST stNetEnd;
} ObstacleBindMsg;

typedef struct
{
  NetHeaderST stNetHeader;

  DilixinxiST stDilixinxi;

  NetEndST stNetEnd;
} FP2DilixinxiBindMsgST; /*车端发向指控端地理信息装订信息*/

typedef struct
{
  LongLatHeightST stLLHOriginPos; /*原点*/
  ENUCorST stLLHEndPos;           /*终点*/
} MapCorditCFST;                  /*自动驾驶坐标系参数结构体*/

typedef struct
{
  LongLatHeightST stLLHOriginPos; /*原点*/
  GlobalPositionST stLLHEndPos;   /*终点*/
} MapCorditST;                    /*自动驾驶坐标系参数结构体*/

typedef struct _FP2VCSEPtCourseBindMsg
{
  NetHeaderST stNetHeader;

  MapCorditST stMapCordit; /*自动驾驶坐标系参数*/

  NetEndST stNetEnd;
} FP2VCCorditBindMsgST; /*无人驾驶台发向综合控制设备的运动坐标系装订信息*/

typedef struct _FP2VCCmdMsg
{
  NetHeaderST stNetHeader; /*报文头*/

  UINT8 Reserve[4]; /*保留字段*/

  NetEndST stNetEnd;
} FP2VCCmdMsgST; /*命令报文*/ /*跟踪路径查询和运动坐标系查询报文使用此结构体*/

typedef struct _FP2VCAckMsg
{
  NetHeaderST stNetHeader; /*报文头*/

  UINT32 uiMsgNo;   /*接收的报文序号*/
  UINT8 Reserve[4]; /*保留字段*/

  NetEndST stNetEnd;
} FP2VCAckMsgST; /*接收确认报文*/

typedef struct _FP2VCChasCtrlMsg
{
  NetHeaderST stNetHeader; /*报文头*/

  VC2ChasCmdST stVC2ChasCmd; /*底盘的控制命令*/

  NetEndST stNetEnd;
} FP2VCChasCtrlMsgST; /*无人驾驶台发向综合控制设备的底盘设备控制报文*/

typedef struct _FP2VCChasMoveCtrlMsg
{
  NetHeaderST stNetHeader;

  UINT8 ucMoveCtrlMode; /*1：定航向控制 2：路径点控制 3:遥控控制*/
  UINT8 ucMoveFlag;     /*55H:运动开始  AAH：运动停止*/
  UINT16 usVehSpeed;    /*巡航速度，1LSB = 0.1m/s*/

  NetEndST stNetEnd;
} FP2VCChasMoveCtrlMsgST; /*无人驾驶台发向综合控制设备的运动控制报文*/

/*综控发向无人驾驶台数据*/
typedef struct
{
  UINT8 VC : 2; /*综控*/
  UINT8 : 6;

  UINT8 Chassis : 2; /*底盘*/
  UINT8 : 6;

  UINT8 Mems : 2;     /*定位定向设备*/
  UINT8 PercepEq : 2; /*感知设备*/
  UINT8 : 4;

  UINT8 : 8;
} FPEqStateST; /*无人车各设备状态*/

typedef struct
{
  UINT8 EMS : 2; /*发动机控制器*/
  UINT8 EBS : 2; /*电子刹车系统*/
  UINT8 EPB : 2; /*电子驻车制动系统*/
  UINT8 EPS : 2; /*电子助力转向系统*/

  UINT8 TCU : 2;    /*自动变速箱*/
  UINT8 Ctrler : 2; /*底盘控制器*/
  UINT8 BCM : 2;    /*车身控制系统*/
  UINT8 : 2;

  UINT8 : 8;
  UINT8 : 8;
} ChasDetailStateST; /*底盘设备状态*/

typedef struct _CHAS2VCWorkData
{
  UINT32 uiRecvWorkDataCnt;      /*接收到的工作数据报文计数*/
  ChasDetailStateST stChasState; /*底盘设备状态*/
  UINT8 ucParkState;             /*驻车制动状态*/
  UINT8 ucWorkMode;              /*底盘被控模式*/
  UINT8 ucChasJudgeCtrlerState;  /*底盘端判定上装状态*/

  UINT8 ucBrakState;      /*刹车状态*/
  UINT16 usVehSpeed;      /*车速*/
  INT16 sTurnAngle;       /*前轮转角,左负右正*/
  UINT8 ucTurnAngleSpeed; /*前轮转角速度*/
  UINT8 ucTransGear;      /*变速箱档位*/

  UINT16 usEngineRotSpeed; /*发动机转速*/
  INT16 sEngineTorque;     /*发动机扭矩*/
  UINT8 ucOilOpening;      /*油门开度*/
  UINT8 ucTurnArmOpening;  /*转向开度*/
  UINT8 ucBrakOpening;     /*刹车开度*/

  UINT8 ucEMSWorkState;     /*发动机控制单元工作状态*/
  UINT8 ucOilRemain;        /*油量*/
  UINT8 ucAutoDriveOilOpen; /*自动驾驶油门开度命令*/
  UINT8 ucCtrlAnomalCode;   /*控制异常码*/
  UINT8 ucEmergBrakState;   /*紧急制动状态*/
  UINT16 usBatVolt;         /*电池电压*/

  UINT32 uiMileage;         /*底盘里程*/
  UINT16 usEMSCmdRotSpeed;  /*发动机命令转速*/
  UINT16 usGearBoxOutShaft; /*变速箱输出轴*/
} Chas2VCWorkDataST;        /*底盘工作数据合集*/

typedef struct _CameraDataST
{
  UINT8 ucCameraOn;              /*图像设备在线*/
  UINT8 ucCameraInfoValid;       /*图像信息有效标志*/
  INT16 sCameraToCenterLineDist; /*摄像头中心距车道线中间距离，左负右正*/
  INT16 Reserve;                 /*保留字段*/
  INT16 sCamera_Line_Angle;      /*摄像头纵轴与车道线夹角*/
} CameraDataST;                  /*摄像头检测信息*/

typedef struct _RadarDataST
{
  UINT8 ucRadarOn;          /*激光雷达在线*/
  UINT8 ucRadarInfoValid;   /*激光雷达信息有效标志*/
  INT16 sRadarENUX;         /*激光雷达在东北天坐标系中的X坐标*/
  INT16 sRadarENUY;         /*激光雷达在东北天坐标系中的Y坐标*/
  INT16 sRadar_North_Angle; /*激光雷达纵轴与真北夹角*/
} RadarDataST;              /*激光雷达检测信息*/

typedef struct _VC2FPHeartMsg
{
  NetHeaderST stNetHeader;

  FPEqStateST stEqState;               /*车上各设备状态*/
  Chas2VCWorkDataST stChas2VCWorkData; /*底盘工作数据*/
  INS2VCWorkDataST stINS2VCWorkData;   /*INS工作数据*/

  UINT8 ucVehPosValid;         /*车辆位置有效性*/
  ENUCorST stENUVehFrontWheel; /*车辆前轮ENU坐标位置*/
  ENUCorST stENUVehCenterPos;  /*车辆质心ENU坐标位置*/
  ENUCorST stENUVehBackWheel;  /*车辆后轮ENU坐标位置*/

  INT32 iTrackPointIndex; /*前置跟踪点在路径列表中的索引号*/
  UINT8 ucMoveCtrlFlag;   /*运动控制标识*/

  VC2ChasCmdST stVC2ChasCmd; /*综控发向底盘的控制命令*/

  UINT32 uiPercptMsgNum;     /*感知报文数*/
  CameraDataST stCameraData; /*图像检测信息*/
  RadarDataST stRadarData;   /*激光雷达检测信息*/

  UINT8 ucCanSendChnl;        /*CAN发送报文通道，取值0或1*/
  UINT8 ucReadConfigFileRslt; /*取值0x55为读取文件成功，0xAA为读取文件失败*/
  UINT8 ucVCSimuFlag;         /*取值1为仿真状态，其他为其他状态*/
  UINT8 Reserve[5];           /*保留字节0*/

  NetEndST stNetEnd;
} VC2FPHeartMsgST; /*综合控制设备向战斗操作台发送的心跳报文*/

typedef struct _VC2FPPathBindRsltMsg
{
  NetHeaderST stNetHeader;

  UINT8 ucFrameMultiFlag;           /*多包标识*/
  UINT8 Reserve;                    /*凑数字节*/
  UINT16 usFrameNum;                /*数据包数*/
  UINT32 uiTotalByteNum;            /*装订数据总字节数*/
  UINT16 usFrameNo;                 /*装订包序号*/
  UINT16 usFrameDataBytes;          /*当前包内装订字节数*/
  SmoothPathPointST stPathData[20]; /*路径数据*/

  NetEndST stNetEnd;
} VC2FPPathBindRsltMsgST; /*综合控制设备向战斗操作台发送的路径装订结果反馈报文*/

typedef struct _VC2FPCorditBindRsltMsg
{
  NetHeaderST stNetHeader;

  MapCorditCFST stMapCordit; /*自动驾驶坐标系参数*/

  NetEndST stNetEnd;
} VC2FPCorditBindRsltMsgST; /*综合控制设备发向无人驾驶台的运动坐标系装订结果上传信息*/

typedef struct _VC2FPAckMsg
{
  NetHeaderST stNetHeader; /*报文头*/

  UINT32 uiMsgNo;   /*接收的报文序号*/
  UINT8 Reserve[4]; /*保留字段*/

  NetEndST stNetEnd;
} VC2FPAckMsgST; /*接收确认报文*/

typedef struct
{
  UINT16 usCtrlFrameType; /*控制帧标识*/
  UINT16 usCtrlCmd;       /*控制指令*/
  UINT16 usAnomalCode;    /*异常码*/
} AnomalyDataST;          /*异常信息内容*/

typedef struct
{
  NetHeaderST stNetHeader; /*报文头*/

  AnomalyDataST stAnomalyData; /*异常信息内容*/
  UINT8 Reserve[6];

  NetEndST stNetEnd;
} VC2FPAnomalyMsgST; /*异常信息报文*/

typedef struct
{
  NetHeaderST stNetHeader; /*报文头*/

  UINT8 ucHour; /*命令发出时间*/
  UINT8 ucMinute;
  UINT8 ucSecond;
  UINT16 usMSecond;
  CANFrameST stCanFrame; /*CAN报文*/

  NetEndST stNetEnd;
} VC2FPCanTransMsgST; /*CAN透传报文*/

/*综控发向底盘数据*/
struct CANFrameFlagST
{
  UINT8 ucDestAddr;                                          /*目的地址*/
  UINT8 ucFrameNo;                                           /*发送报文帧号*/
  UINT8 ucHaveSendChnl;                                      /*已发送过的总线*/
  bool operator<(const CANFrameFlagST &stCANFrameFlag) const /*重载结构体的不等于符号*/
  {
    if (stCANFrameFlag.ucDestAddr != ucDestAddr)
    {
      return stCANFrameFlag.ucDestAddr < ucDestAddr;
    }
    return stCANFrameFlag.ucFrameNo < ucFrameNo;
  }
  bool operator==(const CANFrameFlagST &stCANFrameFlag) const /*重载结构体的等于符号*/
  {
    return ((stCANFrameFlag.ucDestAddr == ucDestAddr) && (stCANFrameFlag.ucFrameNo == ucFrameNo));
  }
}; /*CAN发送报文标识*/

struct SendMsgWaitInfoST
{
  UINT8 ucWaitAckType;  /*等待应答的标识，1：只等待应答报文 2：等待应答报文和状态报文*/
  UINT32 uiHaveExeTime; /*已执行时间,单位ms*/
  UINT8 ucRecvAckFlag;  /*已收到应答的标识，1：已收到应答报文 2：已收到状态报文*/
  UINT32 uiMaxExeTime;  /*最大执行时间，单位ms*/
};                      /*发送命令报文的等待时间和已收到应答的标识*/

typedef struct _VC2CHASBasicCmdMsg
{
  UINT8 ucCmd;     /*命令码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum;     /*校验*/
} VC2ChasBasicCmdMsgST; /*基本状态查询*/

typedef struct _VC2CHASSelfCheckMsg
{
  UINT8 ucCmd;     /*命令码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum;      /*校验*/
} VC2ChasSelfCheckMsgST; /*自检命令报文*/

typedef struct _VC2CHASGearMsg
{
  UINT8 ucCmd;     /*命令码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 ucGear;    /*变速档位*/
  UINT8 Reserve[4];
  UINT8 ucCheckSum; /*校验*/
} VC2ChasGearMsgST; /*变速档位设置命令报文*/

typedef struct
{
  UINT16 usSpeed;         /*运动速度*/
  INT16 sTurnAngle;       /*转向角度*/
  UINT8 ucTurnAngleSpeed; /*转角速度*/
} ChasHighMoveParaST;     /*底盘速度/角度/角速度控制模式下的控制参数*/

typedef struct _VC2CHASHighMoveMsg
{
  UINT8 ucCmd;                       /*命令码*/
  UINT8 ucFrameNo;                   /*帧号*/
  ChasHighMoveParaST stHighMovePara; /*运动参数*/
  UINT8 ucCheckSum;                  /*校验*/
} VC2ChasHighMoveMsgST;              /*定速模式下的运动控制命令报文*/

typedef struct _VC2CHASVehBasicMsg
{
  UINT8 ucCmd;     /*命令码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 ucLight;   /*车灯控制*/
  UINT8 ucHorn;    /*车笛*/
  UINT8 Reserve[3];
  UINT8 ucCheckSum;     /*校验*/
} VC2ChasVehBasicMsgST; /*车附属件控制命令报文*/

typedef struct _VC2CHASWorkModeMsg
{
  UINT8 ucCmd;      /*命令码*/
  UINT8 ucFrameNo;  /*帧号*/
  UINT8 ucWorkMode; /*工作模式*/
  UINT8 Reserve[4];
  UINT8 ucCheckSum;     /*校验*/
} VC2ChasWorkModeMsgST; /*工作模式设置命令报文*/

typedef struct _VC2CHASParkBrakMsg
{
  UINT8 ucCmd;      /*命令码*/
  UINT8 ucFrameNo;  /*帧号*/
  UINT8 ucParkFlag; /*/驻车制动/解除驻车标识*/
  UINT8 Reserve[4];
  UINT8 ucCheckSum;     /*校验*/
} VC2ChasParkBrakMsgST; /*驻车制动命令报文*/

typedef struct _VC2CHASBrakMsg
{
  UINT8 ucCmd;     /*命令码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum; /*校验*/
} VC2ChasBrakMsgST; /*紧急制动控制命令报文*/

typedef struct _VC2CHASClearBrakMsg
{
  UINT8 ucCmd;     /*命令码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum;      /*校验*/
} VC2ChasClearBrakMsgST; /*解除紧急制动控制命令报文*/

typedef struct _VC2CHASEngineStartMsg
{
  UINT8 ucCmd;     /*命令码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum;        /*校验*/
} VC2ChasEngineStartMsgST; /*发动机启动控制命令报文*/

typedef struct _VC2CHASEngineStopMsg
{
  UINT8 ucCmd;     /*命令码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum;       /*校验*/
} VC2ChasEngineStopMsgST; /*发动机停止控制命令报文*/

typedef struct _VC2CHASStateAckMsg
{
  UINT8 ucCmd;     /*状态码*/
  UINT8 ucFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum;     /*校验*/
} VC2ChasStateAckMsgST; /*状态应答报文*/

/*底盘发向综控数据*/
typedef struct _CHAS2VCWorkDataMsg1
{
  ChasDetailStateST stState;    /*底盘设备状态*/
  UINT8 ucParkState;            /*驻车制动状态*/
  UINT8 ucWorkMode;             /*底盘被控模式*/
  UINT8 ucChasJudgeCtrlerState; /*底盘端判定上装状态*/
  UINT8 ucCheckSum;             /*校验*/
} Chas2VCWorkDataMsg1ST;

typedef struct _CHAS2VCWorkDataMsg2
{
  UINT8 ucBrakState;      /*刹车状态*/
  UINT16 usVehSpeed;      /*车速*/
  INT16 sTurnAngle;       /*前轮转角*/
  UINT8 ucTurnAngleSpeed; /*前轮转角速度*/
  UINT8 ucTransGear;      /*变速箱档位*/
  UINT8 ucCheckSum;       /*校验*/
} Chas2VCWorkDataMsg2ST;

typedef struct _CHAS2VCWorkDataMsg3
{
  UINT16 usEngineRotSpeed; /*发动机转速*/
  INT16 sEngineTorque;     /*发动机扭矩*/
  UINT8 ucOilOpening;      /*油门开度*/
  UINT8 ucTurnArmOpening;  /*转向开度*/
  UINT8 ucBrakOpening;     /*刹车开度*/
  UINT8 ucCheckSum;        /*校验*/
} Chas2VCWorkDataMsg3ST;

typedef struct _CHAS2VCWorkDataMsg4
{
  UINT8 ucEMSWorkState;    /*发动机控制单元工作状态*/
  UINT16 usEMSCmdRotSpeed; /*发动机命令转速*/
  UINT8 ucCtrlAnomalCode;  /*控制异常码*/
  UINT8 ucEmergBrakState;  /*紧急制动状态*/
  UINT16 usBatVolt;        /*电池电压*/
  UINT8 ucCheckSum;        /*校验*/
} Chas2VCWorkDataMsg4ST;

typedef struct _CHAS2VCWorkDataMsg5
{
  UINT32 uiMileage;         /*底盘里程*/
  UINT16 usGearBoxOutShaft; /*变速箱输出轴*/
  UINT8 ucOilRemain;        /*油量*/
  UINT8 ucCheckSum;         /*校验*/
} Chas2VCWorkDataMsg5ST;

typedef struct _CHAS2VCCmdAckMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum;   /*校验*/
} Chas2VCCmdAckMsgST; /*命令应答报文*/

typedef struct _CHAS2VCCmdStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 Reserve[5];
  UINT8 ucCheckSum;     /*校验*/
} Chas2VCCmdStateMsgST; /*状态报文*/

typedef struct _CHAS2VCSelfCheckStateMsg
{
  UINT8 ucCmd;                /*命令码*/
  UINT8 ucAckFrameNo;         /*应答帧号*/
  UINT8 ucState;              /*底盘设备综合自检结果*/
  ChasDetailStateST stState;  /*详细状态*/
  UINT8 ucCheckSum;           /*校验*/
} Chas2VCSelfCheckStateMsgST; /*自检状态报文*/

typedef struct _CHAS2VCGearStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 ucGear;       /*变速档位*/
  UINT8 Reserve[3];
  UINT8 ucExeState;      /*执行状态*/
  UINT8 ucCheckSum;      /*校验*/
} Chas2VCGearStateMsgST; /*变速档位设置状态报文*/

typedef struct _CHAS2VCVehBasicStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 ucLight;      /*车灯控制*/
  UINT8 ucHorn;       /*车笛*/
  UINT8 Reserve[2];
  UINT8 ucExeState;          /*执行状态*/
  UINT8 ucCheckSum;          /*校验*/
} Chas2VCVehBasicStateMsgST; /*车附属件控制状态报文*/

typedef struct _CHAS2VCWorkModeStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 ucWorkMode;   /*工作模式*/
  UINT8 Reserve[3];
  UINT8 ucExeState;          /*执行状态*/
  UINT8 ucCheckSum;          /*校验*/
} Chas2VCWorkModeStateMsgST; /*工作模式设置状态报文*/

typedef struct _CHAS2VCParkStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 ucParkFlag;   /*驻车制动/解除驻车标识*/
  UINT8 Reserve[3];
  UINT8 ucExeState;      /*执行状态*/
  UINT8 ucCheckSum;      /*校验*/
} Chas2VCParkStateMsgST; /*驻车制动/解除驻车状态报文*/

typedef struct _CHAS2VCBrakStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 Reserve[4];
  UINT8 ucExeState;      /*执行状态*/
  UINT8 ucCheckSum;      /*校验*/
} Chas2VCBrakStateMsgST; /*紧急制动控制状态报文*/

typedef struct _CHAS2VCClearBrakCtrlStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 Reserve[4];
  UINT8 ucExeState;           /*执行状态*/
  UINT8 ucCheckSum;           /*校验*/
} Chas2VCClearBrakStateMsgST; /*解除紧急制动控制状态报文*/

typedef struct _CHAS2VCEngineStartStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 Reserve[4];
  UINT8 ucExeState;             /*执行状态*/
  UINT8 ucCheckSum;             /*校验*/
} Chas2VCEngineStartStateMsgST; /*发动机启动控制状态报文*/

typedef struct _CHAS2VCEngineStopStateMsg
{
  UINT8 ucCmd;        /*命令码*/
  UINT8 ucAckFrameNo; /*帧号*/
  UINT8 Reserve[4];
  UINT8 ucExeState;            /*执行状态*/
  UINT8 ucCheckSum;            /*校验*/
} Chas2VCEngineStopStateMsgST; /*发动机停止控制状态报文*/

/*综控发向感知设备数据*/
typedef struct _VC2PercptHeartMsg
{
  NetHeaderST stNetHeader;

  UINT8 Reserve[10];

  NetEndST stNetEnd;
} VC2PercptHeartMsgST; /*心跳报文*/

typedef struct _VC2PercptAckMsg
{
  NetHeaderST stNetHeader; /*报文头*/

  UINT32 uiMsgNo;   /*接收的报文序号*/
  UINT8 Reserve[4]; /*保留字段*/

  NetEndST stNetEnd;
} VC2PercptMsgST; /*接收确认报文*/

/*感知设备发向综控数据*/
typedef struct _Percpt2VCHeartMsg
{
  NetHeaderST stNetHeader;

  UINT8 ucEqState;           /*感知设备状态*/
  CameraDataST stCameraData; /*图像信息数据*/
  RadarDataST stRadarData;   /*激光雷达信息数据*/
  UINT8 Reserve;             /*保留字段*/

  NetEndST stNetEnd;
} Percpt2VCHeartMsgST; /*心跳报文*/

typedef struct _Percpt2VCAckMsg
{
  NetHeaderST stNetHeader; /*报文头*/

  UINT32 uiMsgNo;   /*接收的报文序号*/
  UINT8 Reserve[4]; /*保留字段*/

  NetEndST stNetEnd;
} Percpt2VCMsgST; /*接收确认报文*/

/************************系统控制命令使用的结构体******************************/

/*栅格地图中每个栅格的信息结构体，栅格地图左上角为行号/列号的0/0*/
struct GridInfoST
{
  INT32 iGridRowNo, iGridColNo;  // 栅格坐标，这里为了方便按照C++的数组来计算，栅格列号，栅格行号
  INT32 F, G, H, BrokenLineCost; // F=G+H
  GridInfoST *parent;            // parent的坐标，这里没有用指针，从而简化代码
  GridInfoST(int _iGridRowNo, int _iGridColNo)
      : iGridRowNo(_iGridRowNo), iGridColNo(_iGridColNo), F(0), G(0), H(0), BrokenLineCost(0), parent(nullptr) // 变量初始化
  {
  }
  GridInfoST()
  {
  }
};

struct GridMapIndexST
{
  INT32 iGridRowNo;  /*栅格行号*/
  INT32 iGridColNo;  /*栅格列号*/
  INT32 nObjectType; // 栅格类型 0 可行区域,1 障碍,2 未知区域,3 障碍膨胀区域,8 路径点
  GridMapIndexST(int _iGridRowNo, int _iGridColNo)
      : iGridRowNo(_iGridRowNo), iGridColNo(_iGridColNo), nObjectType(0) // 变量初始化
  {
  }
  GridMapIndexST()
  {
    iGridRowNo = 0;
    iGridColNo = 0;
    nObjectType = 0;
  }
  bool operator<(const GridMapIndexST &stCurGridIndex) const /*重载结构体的不等于符号*/
  {
    if (stCurGridIndex.iGridColNo != iGridColNo)
    {
      return stCurGridIndex.iGridColNo < iGridColNo;
    }
    return stCurGridIndex.iGridRowNo < iGridRowNo;
  }
  bool operator==(const GridMapIndexST &stCurGridIndex) const /*重载结构体的等于符号*/
  {
    return ((stCurGridIndex.iGridColNo == iGridColNo) && (stCurGridIndex.iGridRowNo == iGridRowNo));
  }
}; /*栅格地图索引结构体*/

typedef struct
{
  NetHeaderST stNetHeader; /*±šÎÄÍ·*/

  double fTime;
  double x;
  double y;
  double z;
  float m_fVehCourse; /*hangxiang 与正北夹角0~360度*/
  double fACCX;
  double fACCY;
  double fACCZ;
  double fGyroX;
  double fGyroY;
  double fGyroZ;
  float fEastSpeed;   /*Õæ¶«ËÙ¶È*/
  float fNorthSpeed;  /*Õæ±±ËÙ¶È*/
  float fSkyGround;   /*ÌìÏòËÙ¶È*/
  float fGroundSpeed; /*Ë®ÆœµØËÙ*/

  NetEndST stNetEnd;
} VC2PerceptIMUMsgST;

typedef unsigned char uint8;
typedef short int int16;
typedef unsigned short int uint16;

struct ObstHandlStrategy
{                            // 障碍处理策略
  u_8 staitcObstacle : 1;    // 静态障碍物处理   / bit0 低位
  u_8 frontDynaObstacle : 1; // 前车动态纵向障碍物处理
  u_8 meetDynaObstacle : 1;  // 会车动态纵向障碍物处理
  u_8 corssDynaObstacle : 1; // 横向穿越动态障碍物处理 / bit3 高位
  u_8 reserved : 4;
};

struct CtrlMsgContent
{

  u_8 timestamp_s[8];            // 时戳秒
  u_8 timestamp_ns[8];           // 时戳纳秒
  u_8 ctrlCarNum;                // 控制车号
  u_8 ctrlType;                  // 控制类型 enum ControlType
  bool movingFlag;               // 开始/停止标志 enum MovingFlag
  float cruisingSpeed;           // 巡航车速     float 单位:m
  bool cirMotionFlag;            // 绕圈运动标志  enum CirMotionFlag
  bool formTravelFlag;           // 编队行驶标志  enum FormTravelFlag
  u_8 formTravelNum;             // 编队行驶标号
  float followDistance;          // 跟随间距     float m
  ObstHandlStrategy obsStrategy; // 障碍处理策略
  u_8 reserved;
};

typedef struct
{
  NetHeaderST stNetHeader;
  u_8 ctrlCarNum;      // 控制车号
  u_8 timestamp_s[8];  // 时戳秒
  u_8 timestamp_ns[8]; // 时戳纳秒
  u_8 ctrlType;        // 控制类型 enum ControlType
  bool movingFlag;     // 开始/停止标志 enum MovingFlag
  // u_8 reserved;
  float cruisingSpeed;           // 巡航车速     float 单位:m
  ObstHandlStrategy obsStrategy; // 障碍处理策略
  bool cirMotionFlag;            // 绕圈运动标志  enum CirMotionFlag
  bool formTravelFlag;           // 编队行驶标志  enum FormTravelFlag
  u_8 formTravelNum;             // 编队行驶标号
  float followDistance;          // 跟随间距     float m

  NetEndST stNetEnd;
} Pad2ADASHeartMsgST;

typedef struct
{
  u_16 id;
  double lon;
  double lat;
  u_8 type;
  u_8 reserve;
} TaskPointST;

typedef struct
{
  NetHeaderST stNetHeader;
  u_8 tasktype;
  u_8 multipakgeflag;
  u_16 pakgetotalnum;
  u_32 bytetotalnum;
  u_16 pakgeNO;
  u_16 taskpakgebytenum;
  TaskPointST taskpoint[20];
  NetEndST stNetEnd;
} TaskPointsST;

typedef struct
{
  NetHeaderST stNetHeader;
  double timestamp_s;     // 时戳秒
  double timestamp_ns;    // 时戳纳秒
  bool engineupdown;      // 发动机起停
  bool emergencystop;     // 急停
  uint8 drive_mode;       // 驾驶模式
  float brake_percent;    // 刹车开度 %
  float throttle_percent; // 油门开度 %
  float steer_angle;      //  方向盘转角  度
  bool parking_stop;
  uint8 gear_location;
  uint8 reserve[3];
  NetEndST stNetEnd;
} RemoteDriveST; // 遥控驾驶

typedef struct
{
  INT16 sVertexDist2Lidar;   /*与激光雷达的顶点距离，单位0.01m*/
  INT16 sVertexAngle2LidarX; /*与激光雷达X轴的夹角，单位0.1°*/
  INT16 sVertexHeight;       /*距地面高度，单位0.01m*/
} ObsClusterVertexST;        /*障碍物聚类后的4个顶点信息*/

typedef struct
{
  ObsClusterVertexST stObsClusterInfo[4]; /*障碍物顶点信息*/
  unsigned short usObsID;                 /*障碍物ID*/
  unsigned short usLidarXRelativeSpeed;   /*物理值 = 十六进制*精度(0.01) + 偏移量(-100)*/
  unsigned short usLidarYRelativeSpeed;   /*物理值 = 十六进制*精度(0.01) + 偏移量(-100)*/
} ObsClusterInfoST;                       /*障碍物聚类后的4个顶点信息*/

typedef struct
{
  NetHeaderST stNetHeader; /*报文头*/
  double fTime;
  double x;
  double y;
  double z;
  float fVehCourse;                      /*航向角*/
  unsigned char ucFrameMultiFlag;        /*多包标识*/
  unsigned short usFrameNum;             /*数据包数*/
  unsigned short usFrameNo;              /*装订包序号*/
  unsigned char ucFrameObsNum;           /*本包内障碍物数*/
  ObsClusterInfoST stObsClusterInfo[15]; /*障碍物信息*/

  NetEndST stNetEnd;        /*报文尾*/
} Percpt2VCObsClusterMsgST; /*障碍物聚类后的信息报文*/

typedef struct _MemsMsg
{
  unsigned char INSGPSWeekSec : 1;
  unsigned char INSUTCTime : 1;
  unsigned char INSLongLat : 1;
  unsigned char INSHeight : 1;
  unsigned char INSSpeed : 1;
  unsigned char INSAngle : 1;
  unsigned char INSPosDevialtion : 1;
  unsigned char INSSpeedDevialtion : 1;

  unsigned char INSAngleDevialtion : 1;
  unsigned char INSState : 1;
  unsigned char IMU1 : 1;
  unsigned char IMU2 : 1;
  unsigned char IMU3 : 1;
  unsigned char GPSGPSWeekSec : 1;
  unsigned char GPSUTCTime : 1;
  unsigned char GPSLongLat : 1;

  unsigned char GPSHeight : 1;
  unsigned char GPSSpeed : 1;
  unsigned char GPSAngle : 1;
  unsigned char GPSStar : 1;
  unsigned char : 4;

  unsigned char : 8;
} MemsMsgFlagST; /*Mems所有的CAN消息*/

typedef union
{
  MemsMsgFlagST stMemsMsgUp; /*需上传的CAN消息*/
  unsigned int uiMemsMsgUp;
} MemsMsgFlagUN; /*需上传的Mems消息集*/

typedef struct _FP2VCMemsUpSetMsg
{
  NetHeaderST stNetHeader; /*报文头*/

  MemsMsgFlagUN unMemsMsgUp; /*设置报文是否上传*/
  bool bUpFlag;              /*上传标志*/
  unsigned char reserve;     /*保留字*/

  NetEndST stNetEnd;
} Pad2PNCMemsUpSetMsgST; /*定位定向报文透传设置内容字段*/

typedef struct
{
  NetHeaderST stNetHeader; /*报文头*/

  unsigned int uiChasMsgUp; /*设置报文是否上传*/
  bool bUpFlag;             /*上传标志*/
  unsigned char reserve;    /*保留字*/

  NetEndST stNetEnd;
} Pad2PNCChasUpSetMsgST; /*底盘报文透传设置内容字段*/

typedef struct
{
  // @verbatim (language="comment",  text="开始/停止标志，true：开始 false：停止")
  bool bStartAndStop;
  // @verbatim (language="comment",  text="巡航速度，单位m/s")
  float fSetSpeed;
  // @verbatim (language="comment",  text="循环绕圈标志，true：开启 false：关闭")
  bool bMoveLoop;
  // @verbatim (language="comment",  text="编队行驶标志，true：编队行驶 false：单车行驶")
  bool bGroupMove;
  // @verbatim (language="comment",  text="编队行驶编号：1：头车 2~N：从车")
  unsigned char ucInGroupNo;
  // @verbatim (language="comment",  text="编队行驶跟随间距，单位：m")
  float fGroupFollowSpace;
  // @verbatim (language="comment",  text="障碍物处理策略，b0:静障碍物处理 0:停车 1:自主决策 b1:前向动态纵向障碍物处理
  // 0:跟随 1:自主决策 b2:会车动态纵向障碍物处理 0:停车 1:自主决策 b3:横向穿越动态障碍物处理 0:停车 1:自主决策")
  unsigned char ucObsProcessRule;
  unsigned char reserve[16];
} TrackAndAvoidObsCtrlDataST; /*循迹避障控制数据*/

typedef struct
{
  // @verbatim (language="comment",  text="开始/停止标志，true：开始 false：停止")
  bool bStartAndStop;
  // @verbatim (language="comment",  text="巡航速度，单位m/s")
  float fSetSpeed;
  unsigned char reserve[24];
} OnekeyReturnCtrlDataST; /*一键返航控制数据*/

typedef struct
{
  // @verbatim (language="comment",  text="辅助紧急制动标志，true：开启 false：关闭")
  bool bAEB;
  unsigned char reserve[28];
} AEBCtrlDataST; /*辅助紧急制动控制数据*/

typedef struct
{
  //@verbatim (language="comment",  text="示廓灯状态 0：关 1：开")
  bool bPosLightSTA;
  //@verbatim (language="comment",  text="左转向灯状态 0：关 1：开")
  bool bLeftTurnLightSTA;
  //@verbatim (language="comment",  text="右转向灯状态 0：关 1：开")
  bool bRightTurnLightSTA;
  //@verbatim (language="comment",  text="后雾灯状态 0：关 1：开")
  bool bFogLightSTA;
  //@verbatim (language="comment",  text="防空灯状态 0：关 1：开")
  bool bAirDefenseLightSTA;
  //@verbatim (language="comment",  text="远光状态 0：关 1：开")
  bool bHighLightSTA;
  //@verbatim (language="comment",  text="近光状态 0：关 1：开")
  bool bLowLightSTA;
  //@verbatim (language="comment",  text="刹车灯状态 0：关 1：开")
  bool bBrkLightSTA;
  //@verbatim (language="comment",  text="危险报警灯灯状态 0：关 1：开")
  bool bEmergencyLightSTA;
  //@verbatim (language="comment",  text="喇叭状态 0：关 1：开")
  bool bHornSTA;
} ChasHornLightST;

typedef struct
{
  //@verbatim (language="comment",  text="油门开度，单位%")
  float fThrottle;
  //@verbatim (language="comment",  text="刹车开度，单位%")
  float fBrake;
  //@verbatim (language="comment",  text="目标速度，单位m/s")
  float fSpeed;
  //@verbatim (language="comment",  text="目标加速度，单位m²/s")
  float fAcceleration;
  //@verbatim (language="comment",  text="转向模式")
  uint8 ucSteerMode;
  //@verbatim (language="comment",  text="目标前轮转角，单位°")
  float fFrontAxleAngle;
  //@verbatim (language="comment",  text="目标前轮转角速度，单位°/s")
  float fFrontAxleAngleSpeed;
  //@verbatim (language="comment",  text="保留字段")
  uint8 reserve[1];
} ChasMoveDataST;

typedef struct
{
  //@verbatim (language="comment",  text="时间戳_秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampS;
  //@verbatim (language="comment",  text="时间戳_纳秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampNs;

  //@verbatim (language="comment",  text="控制命令")
  uint8 ucCmdType;

  //@verbatim (language="comment",  text="车电控制")
  uint8 ucKeySwitchCtrl;
  //@verbatim (language="comment",  text="高压控制")
  uint8 ucStartSwitchCtrl;

  //@verbatim (language="comment",  text="底盘运动数据")
  ChasMoveDataST stChasMoveData;
  //@verbatim (language="comment",  text="true：驻车，false：解除驻车")
  bool bParkingBrake;
  //@verbatim (language="comment",  text="驾驶模式 0：遥控 1：完全自动驾驶 2：保留 3：无效")
  uint8 ucDriveMode;
  //@verbatim (language="comment",  text="档位位置 0：空档 1：D档 2：R档 3：无效")
  uint8 ucGearLocation;
  //@verbatim (language="comment",  text="底盘灯笛状态")
  ChasHornLightST stChasHornLight;
  uint8 reserve[2];
} ChasCmdST;

typedef struct
{
  //@verbatim (language="comment",  text="时间戳_秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampS;
  //@verbatim (language="comment",  text="时间戳_纳秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampNs;

  //@verbatim (language="comment",  text="控制类型")
  uint8 ucCmdType;
  //@verbatim (language="comment",  text="开始/停止标志，true：开始 false：停止")
  // 修改数据类型 0停止 1方向运动 2前向运动
  uint8 bStartAndStop;
  //@verbatim (language="comment",  text="巡航速度，单位m/s")
  float fSetSpeed;
  //@verbatim (language="comment",  text="循环绕圈标志，true：开启 false：关闭")
  bool bMoveLoop;
  //@verbatim (language="comment",  text="编队行驶标志，true：编队行驶 false：单车行驶")
  bool bGroupMove;
  //@verbatim (language="comment",  text="编队行驶编号：1：头车 2~N：从车")
  uint8 ucInGroupNo;
  //@verbatim (language="comment",  text="编队行驶跟随间距，单位：m")
  float fGroupFollowSpace;
  //@verbatim (language="comment",  text="障碍物处理策略，b0:静障碍物处理 0:停车 1:自主决策 b1:前向动态纵向障碍物处理
  // 0:跟随 1:自主决策 b2:会车动态纵向障碍物处理 0:停车 1:自主决策 b3:横向穿越动态障碍物处理 0:停车 1:自主决策")
  uint8 ucObsProcessRule;

  //@verbatim (language="comment",  text="辅助紧急制动标志，true：开启 false：关闭")
  bool bAEB;

  //@verbatim (language="comment",  text="底盘直接控制量")
  ChasCmdST stChasCmd;
} PNCCmdST;

typedef struct
{
  NetHeaderST stNetHeader;

  unsigned char ucVehNo; /*控制车号*/
  PNCCmdST stPNCCmd;

  NetEndST stNetEnd;
} PAD2PNCCtrlMsgST; /*远程操控终端向决策控制单元的控制报文*/

typedef struct
{
  NetHeaderST stNetHeader;

  unsigned char ucVehNo;      /*控制车号*/
  unsigned char ucPassPosNum; /*途经点个数*/

  GlobalPositionST stStartPos;    /*起点*/
  GlobalPositionST stPassPos[10]; /*途经点*/
  GlobalPositionST stEndPos;      /*终点*/

  NetEndST stNetEnd;
} PAD2PNCRouteRequestMsgST; /*远程操控终端向决策控制单元的导航规划报文*/

typedef struct
{
  double x;
  double y;
  double z;
  double w;
} Quaternion;

typedef struct
{
  NetHeaderST stNetHeader;

  unsigned char Reserve;               /*保留字节，凑偶数字节*/
  unsigned char ucFrameMultiFlag;      /*多包标识*/
  unsigned short int usFrameNum;       /*数据包数*/
  unsigned int uiTotalByteNum;         /*路径数据总字节数*/
  unsigned short int usFrameNo;        /*包序号*/
  unsigned short int usFrameDataBytes; /*当前包内装订字节数*/
  GlobalPositionST stPathPoint[20];    /*路径数据*/

  NetEndST stNetEnd;
} TrajectoryMsgST; /*路径装订报文*/

struct CarDevStaDef
{
  float throttle;    // 油门开度
  float brake;       // 刹车开度
  float velocity;    // 目标速度
  float battery;     // 电量
  u_8 gearSta;       // 档位
  u_8 parkingBrak;   // 驻车制动
  u_8 driveMode;     // 驾驶模式
  u_8 chassisErrSta; // 底盘故障状态
};

typedef struct
{
  NetHeaderST stNetHeader;

  //  unsigned char ucOwnPadNo; /*命令远程操控终端编号，1~254，FFH：未受任何远程操控终端控制*/
  //  unsigned char reserve1;

  //  unsigned char ucADASFaultSTA; /*无人驾驶系统总故障状态，00H：正常 01H：轻微故障 02H：严重故障 03H：致命故障
  //                                   07H：无效*/
  //  unsigned char reserve2;
  //  unsigned char aryLidarFaultSTA[4];     /*依次为左前、前中、右前、后激光雷达故障状态*/
  //  unsigned char aryRadarFaultSTA[4];     /*依次为前、右、左毫米波雷达故障状态*/
  //  unsigned char ucUltraSonicFaultSTA[8]; /*超声波雷达故障状态*/
  //  unsigned char aryCamara[4];            /*摄像头故障状态*/
  //  float fFrontAxleAngle;                 /*车辆当前前轮转角*/

  CarDevStaDef stChassis;
  GlobalPositionST stGlobalPostion; /*车辆位置*/
  Quaternion stQuaternion;          /*车辆四元数姿态*/

  NetEndST stNetEnd;
} PNC2PadHeartMsgST; /*决策控制单元向远程操控终端反馈的心跳报文*/

// typedef struct
// {
//   NetHeaderST stNetHeader;

//   can_frame stCanFrame;

//   NetEndST stNetEnd;
// } ADAS2PadCanFrameMsgST;
/*CAN报文透传上报报文*/

typedef struct
{
  //@verbatim (language="comment",  text="时间戳_秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampS;
  //@verbatim (language="comment",  text="时间戳_纳秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampNs;
  //@verbatim (language="comment",  text="组合导航纬度，单位°")
  double fLatitude;
  //@verbatim (language="comment",  text="组合导航经度，单位°")
  double fLongitude;
  //@verbatim (language="comment",  text="组合椭球高度，单位m")
  float fEllipHeight;
  //@verbatim (language="comment",  text="海拔-椭球高度差，单位m 椭球高=海拔高+高度差")
  float fAlt_EllipHeight;
  //@verbatim (language="comment",
  // text="组合航向，与东向X轴夹角，一二象限逆时针旋转为0～+180度，四三象限顺时针旋转为0～-180度，单位°")
  float fAzimuth;
  //@verbatim (language="comment",  text="组合俯仰，单位°")
  float fPitch;
  //@verbatim (language="comment",  text="组合横滚，单位°")
  float fRoll;
  //@verbatim (language="comment",  text="组合东向速度，单位m/s")
  float fEastSpeed;
  //@verbatim (language="comment",  text="组合北向速度，单位m/s")
  float fNorthSpeed;
  //@verbatim (language="comment",  text="组合天向速度，单位m/s")
  float fSkySpeed;
  //@verbatim (language="comment",  text="组合水平速度，单位m/s")
  float fGroundSpeed;
  //@verbatim (language="comment",  text="X轴加速度，单位m/s²")
  double fActX;
  //@verbatim (language="comment",  text="Y轴加速度，单位m/s²")
  double fActY;
  //@verbatim (language="comment",  text="Z轴加速度，单位m/s²")
  double fActZ;
  //@verbatim (language="comment",  text="X轴角速度，单位°/s")
  double fGyroX;
  //@verbatim (language="comment",  text="Y轴角速度，单位°/s")
  double fGyroY;
  //@verbatim (language="comment",  text="Z轴角速度，顺时针为正，单位°/s")
  double fGyroZ;
  //@verbatim (language="comment",  text="组合状态")
  uint8 ucINSState;
  //@verbatim (language="comment",  text="GPS状态")
  uint8 ucGPSState;
  //@verbatim (language="comment",  text="前天线星数")
  uint8 ucFrontStarNum;
  //@verbatim (language="comment",  text="后天线星数")
  uint8 ucBackStarNum;
  //@verbatim (language="comment",  text="根据定位定向设备信息计算UTM位置")
  GlobalPositionST stGlobalPostion;
  //@verbatim (language="comment",  text="根据定位定向设备信息计算姿态信息")
  Quaternion stQuaternion;
} INSDataST; /*决策控制单元向远程操控终端反馈的定位定向数据报文*/

typedef struct
{
  NetHeaderST stNetHeader;

  INSDataST stINSData; /*定位信息数据表*/

  NetEndST stNetEnd;
} PNC2PadINSDataMsgST; /*决策控制单元向远程操控终端反馈的定位定向数据报文*/

typedef struct
{
  //@verbatim (language="comment",  text="时间戳_秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampS;
  //@verbatim (language="comment",  text="时间戳_纳秒，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间")
  double fTimeStampNs;

  //@verbatim (language="comment",  text="底盘故障状态")
  uint8 ucFaultSTA;
  uint16 usFaultCode;

  //@verbatim (language="comment",  text="里程，单位m")
  float fOdometer;

  //@verbatim (language="comment",  text="车电数据")
  uint8 ucKeySwitchCtrl;
  //@verbatim (language="comment",  text="高压数据")
  uint8 ucStartSwitchCtrl;

  //@verbatim (language="comment",  text="底盘运动数据")
  ChasMoveDataST stChasMoveData;
  //@verbatim (language="comment",  text="驻车状态，true驻车 false解除驻车")
  bool bParkingBrake;
  //@verbatim (language="comment",  text="驾驶模式 0：遥控 1：完全自动驾驶 2：保留 3：无效")
  uint8 ucDriveMode;
  //@verbatim (language="comment",  text="档位位置 0：空档 1：R档 2：D档 3：无效")
  uint8 ucGearLocation;
  //@verbatim (language="comment",  text="底盘灯笛状态")
  ChasHornLightST stChasHornLight;

  //@verbatim (language="comment",  text="蓄电池组电量，精度0.01%")
  float fBatSOC;

  //@verbatim (language="comment",  text="驱动电机转速(四轮平均转速)，精度1，单位rpm")
  int16 sMotorRotSpeed;
  //@verbatim (language="comment",  text="驱动电机扭矩，精度1，单位Nm")
  int16 sMotorTorque;

  //@verbatim (language="comment",  text="多模块配置模式")
  uint8 ucMultiModuleSetState;
  //@verbatim (language="comment",  text="多模块序号")
  uint8 ucMultiModuleSN;
  uint8 reserve;
} ChasDataST;

typedef struct
{
  NetHeaderST stNetHeader;

  ChasDataST stChasData; /*底盘信息数据表*/

  NetEndST stNetEnd;
} PNC2PadChassisDataMsgST; /*决策控制单元向远程操控终端反馈的底盘数据报文*/

struct MoveCaltDataST
{
  //@verbatim (language="comment",  text="前视点UTM位置")
  GlobalPositionST stGlobalPostion;
  //@verbatim (language="comment",  text="底盘运动数据")
  ChasMoveDataST stMoveCaltData;
};

typedef struct
{
  //@verbatim (language="comment",  text="轨迹横向偏差，单位m。车辆在行驶方向左侧，为负。车辆在行驶方向右侧，为正。")
  float fPathHorizErr;
} MoveEvaluateDataST;

typedef struct
{
  NetHeaderST stNetHeader;

  unsigned char ucMoveType;              /*运动控制标识，00H：无效 04H：循迹避障控制 05H：刹车控制 08H：靠边停车 09H：一键返航/倒车
                                            0AH：一键返航/掉头 0BH：辅助紧急制动*/
  MoveCaltDataST stMoveCaltData;         /*运动解算信息数据*/
  ChasCmdST stVC2ChasCmd;                /*底盘控制数据*/
  MoveEvaluateDataST stMoveEvaluateData; /*运动控制评价*/
  unsigned char reserve;                 /*保留字段*/

  NetEndST stNetEnd;
} PNC2PadMoveDataMsgST; /*决策控制单元向远程操控终端反馈的运动相关信息*/

typedef struct
{
  NetHeaderST stNetHeader;

  ChasDataST stChasData; /*底盘信息数据表*/

  NetEndST stNetEnd;
} PNC2PadChasDataMsgST; /*决策控制单元向远程操控终端反馈的底盘数据报文*/

typedef struct
{
  NetHeaderST stNetHeader;

  unsigned char ucPathType;             /*路径类型，01H：导航路径 02H：预测轨迹*/
  unsigned char ucMultiFrameFlag;       /*多包标识，00H：单包 01H：首包 02H：中间包 03H：结束包*/
  unsigned short int usFrameNum;        /*数据总包数*/
  unsigned int uiTotalByteNum;          /*数据总字节数*/
  unsigned short int usFrameNo;         /*包序号，单包填0，多包从1开始*/
  unsigned short int usFrameDataBytes;  /*当前包内路径数据字节数*/
  GlobalPositionST stGlobalPostion[20]; /*车辆位置*/

  NetEndST stNetEnd;
} PNC2PadRouteAndLocalPathMsgST; /*决策控制单元向远程操控终端反馈的路径信息上报报文*/

typedef struct
{
  NetHeaderST stNetHeader;

  unsigned char ucPathType;            /*路径类型，01H：导航路径 02H：预测轨迹*/
  unsigned char ucMultiFrameFlag;      /*多包标识，00H：单包 01H：首包 02H：中间包 03H：结束包*/
  unsigned short int usFrameNum;       /*数据总包数*/
  unsigned int uiTotalByteNum;         /*数据总字节数*/
  unsigned short int usFrameNo;        /*包序号，单包填0，多包从1开始*/
  unsigned short int usFrameDataBytes; /*当前包内路径数据字节数*/
  localPositionST stGlobalPostion[20]; /*车辆位置*/

  NetEndST stNetEnd;
} PNC2PadRouteAndGlobalPathMsgST; /*决策控制单元向远程操控终端反馈的路径信息上报报文*/

typedef struct
{
  int preferred_lane_id;     /*参考车道ID*/
  int lane_ids[6];           /*本路段车道ID*/
  int continued_lane_ids[6]; /*下一路段车道ID*/
  bool change_able_flag[6];  /*当前路可换道标志*/
} RouteSectionST;            /*路段定义表*/

typedef struct
{
  NetHeaderST stNetHeader;

  unsigned char ucPathType;            /*路径类型，01H：导航路径 02H：预测轨迹*/
  unsigned char ucMultiFrameFlag;      /*多包标识，00H：单包 01H：首包 02H：中间包 03H：结束包*/
  unsigned short int usFrameNum;       /*数据总包数*/
  unsigned int uiTotalByteNum;         /*数据总字节数*/
  unsigned short int usFrameNo;        /*包序号，单包填0，多包从1开始*/
  unsigned short int usFrameDataBytes; /*当前包内路径数据字节数*/
  RouteSectionST stRouteSection[10];   /*路段上报结果*/

  NetEndST stNetEnd;
} PNC2PadRouteSetMsgST; /*决策控制单元向远程操控终端反馈的全局路径规划结果上报报文*/

// typedef struct
// {
//   NetHeaderST stNetHeader;

//   adas_pnc_msgs::msg::ProcessInfoTipST stProcessInfoTip; /*流程提示信息*/

//   NetEndST stNetEnd;
// } ADAS2PadProcessInfoST;
/*ADAS向远程操控终端反馈的流程提示信息上报报文*/

typedef struct
{
  unsigned char ucEquip;
  unsigned char ucCmd;
  unsigned char ucDataLen; /*数据字节长度*/
  unsigned char ucData[100];
} CommandST; /*命令体中目前设置最多承载100个字节*/

typedef struct
{
  NetHeaderST stNetHeader;

  // 控制类型	1	07H：运动控制
  // 油门开度	4	float浮点数，单位%
  // 刹车开度	4	float浮点数，单位%
  // 转向角度	4	float浮点数，单位°
  // 转角速度	4	float浮点数，单位°/s
  // 保留字段	15	0
  unsigned char ucCmdType;
  float fThrottlePerct;
  float fBrakePercet;
  float fSteerAngleDeg;
  float fSteerAngleSpeedDeg;
  unsigned char ucData[15];
  NetEndST stNetEnd;

} RemoteControlToPadChassisMotionCommandST; /*遥控器发送至决策控制单元pad的运动命令,20220927,qixianyu*/

typedef struct
{
  NetHeaderST stNetHeader;
  unsigned char ucCmdType;
  unsigned char ucEngine;
  unsigned char ucEmergencyStop;
  unsigned char ucParking;
  unsigned char ucDriveMode;
  unsigned char ucGear;
  unsigned char ucDoubleflash;
  unsigned char ucLeftLight;
  unsigned char ucRightLight;
  unsigned char ucHighBeam;
  unsigned char ucLowBeam;
  unsigned char ucHorn;
  unsigned char ucData[20];
  NetEndST stNetEnd;

} RemoteControlToPadChassisControlCommandST; /*遥控器发送至决策控制单元pad的事件命令,20220927,qixianyu*/

typedef struct
{
  NetHeaderST stNetHeader;
  double fTimeStampS;        // 单位s，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间
  double fTimeStampNs;       // 纳秒
  uint8 usErrorState;        // 底盘故障状态
  uint16 usErrorCode;        // 底盘故障编码
  float us_total_kilometres; // 里程
  uint8 ucEngineState;       // 发动机状态   转速大于500：启动1  转速小于500：关闭0
  bool usParkingBrake;       // 驻车制动
  uint8 usGearLocation;
  bool usLeftLight;
  bool usRightLight;
  float usOilPercent;
  float usVehicleSpeed;
  float usEngineSpeed;
  uint8 usDrivingMode;
  bool bRecTask;
  uint8 res[6];
  // float brake_percet;
  // float vehicle_speed;
  // float steer_wheel_angle_deg;
  // float steer_wheel_angle_speed;
  // unsigned char ucParking;
  // unsigned char ucDriveMode;
  // unsigned char ucGearLevel;
  // unsigned char ucDoubleflash;
  // unsigned char ucLeftLight;
  // unsigned char ucRightLight;
  // unsigned char steer_mode;
  // unsigned char parking_mode;
  // unsigned char ucHighBeam;
  // unsigned char ucLowBeam;
  // unsigned char ucAutoDrivingSwitchState;
  // unsigned char parking_mode_reserve;
  // unsigned char ucHorn;
  // float air_pressure;
  // int16 engine_speed;
  // int16 transmission_torque;
  // unsigned char ucEmergencySwitchState;
  // unsigned char ucGearTransmissionState;
  NetEndST stNetEnd;

} PadChassisToRemoteControlReportST; /*控制单元pad发送到遥控器的状态,20220927,qixianyu*/

// 传递多边形障碍物 20220929 qixianyu
//  520 -35 = 485 为可以存下的顶点个数
typedef struct
{
  NetHeaderST stNetHeader;
  unsigned char ucFrameMultiFlag;  /*多包标识*/
  unsigned short usFrameNum;       /*数据包数*/
  unsigned short usFrameNo;        /*装订包序号*/
  unsigned short usObsNo;          /*障碍物个数*/
  unsigned char ucObsIndexNo[485]; /*障碍物顶点个数*/
  NetEndST stNetEnd;

} ObsPolyFirstPackageST;

// 520 -36 = 484 /2 = 242 个xy坐标 242/2 = 121个顶点
typedef struct
{
  NetHeaderST stNetHeader;
  unsigned short ucFrameMultiFlag; /*多包标识*/
  unsigned short usFrameNum;       /*数据包数*/
  unsigned short usFrameNo;        /*装订包序号*/
  unsigned short usCurObsVertexNo; /*当前包的顶点个数,肯定为偶数个顶点*/
  short usObsVertex[242];
  NetEndST stNetEnd;
} ObsPolyMiddleANdLastPackageST;

typedef struct
{
  NetHeaderST stNetHeader;

  unsigned char ucPathType;            /*保留字节，凑偶数字节*/
  unsigned char ucFrameMultiFlag;      /*多包标识*/
  unsigned short int usFrameNum;       /*数据包数*/
  unsigned int uiTotalByteNum;         /*路径数据总字节数*/
  unsigned short int usFrameNo;        /*包序号*/
  unsigned short int usFrameDataBytes; /*当前包内装订字节数*/
  GlobalPositionST stPathPoint[20];    /*路径数据*/
  NetEndST stNetEnd;
} PNC2PADLocalPathST; /*路径装订报文*/

typedef struct
{
  NetHeaderST stNetHeader;
  CtrlMsgContent cmd;
  NetEndST stNetEnd;
} PAD2PNC_ST; /*路径装订报文*/

#pragma pack()

#endif // ALLTYPES_H
