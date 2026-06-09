#ifndef ROS_NODE_HPP_
#define ROS_NODE_HPP_

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <ros/package.h>
#include <fstream>
#include "driver_msgs/ChassisCmd.h"
#include "driver_msgs/DriveCmd.h"
#include "driver_msgs/GearCmd.h"
#include "driver_msgs/ModeCmd.h"
#include "driver_msgs/LightHornWiperCmd.h"
#include "driver_msgs/ParkingBrakeCmd.h"
#include "driver_msgs/PowerCmd.h"
#include "driver_msgs/SteeringWheelCmd.h"
#include "driver_msgs/ChassisReport.h"
#include "localization_msgs/Localization.h"
#include "driver_msgs/MotionStartCmd.h"
#include "driver_msgs/ResetCmd.h"
#include "planning_msgs/TrajectoryPointArray.h"
#include "planning_msgs/TrajectoryPoint.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "lanelet_map_msgs/Way.h"
#include "lanelet_map_msgs/LaneletMap.h"
#include "taskPoints_msgs/taskPoints.h"
#include "heartbeat_msgs/Heartbeat.h"
#include "perception_msgs/PerceptionObstacles.h"
#include "perception_msgs/PerceptionObstacle.h"
#include "perception_msgs/PredictionObstacles.h"
#include "perception_msgs/PredictionObstacle.h"
#include "driver_msgs/LightHornWiperReport.h"
#include "driver_msgs/PowerCmd.h"
#include "driver_msgs/EstopCmd.h"
#include "platoon_msgs/PlatoonOpt.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"
#include "platoon_common/platoon_common.h"

#include <std_msgs/Int32.h>




#include "yaml-cpp/yaml.h"



// #include <std_srvs/Trigger.h>
// #include <rosgraph_msgs/Log.h>

#include "pad/alltypes.h"

#endif

#include <memory>
#include <string>
#include <QThread>
// #include <QStringListModel>

enum class VehicleType
{
  ZHI_TONG = 0,
  ZHITO_TRUCK = 1,
  TANK_500 = 2,
  SZC = 3,
  ZW = 4,
  X6000 =5,
  XB =6
};

//enum PlatoonType {
//  NONE =0, //
//  BUILD, //构建
//  JOIN,  //入队
//  LEAVE, //出队
//  DISSOLVE, //解散
//  COLUMN,   //纵队
//  DIAMOND,   //菱形
//  RUNNING
//};

class QRosNode : public QThread
{
  Q_OBJECT
public:
  QRosNode(int argc, char **argv);
  virtual ~QRosNode();
  bool init();
  void run();

  // ros call back
  void localizationTopicCallback(const localization_msgs::Localization::ConstPtr &msg);   // 接收定位消息
  void chassisTopicCallback(const driver_msgs::ChassisReport::ConstPtr &msg);             // 接收底盘消息
  void trajectoryTopicCallback(const planning_msgs::TrajectoryPointArray::ConstPtr &msg); // 接受局部轨迹消息
  void GlobalPath84callback(const lanelet_map_msgs::Way::ConstPtr &msg);                  // 接受全局路径消息
  void Dilixinxicallback(const perception_msgs::PredictionObstacles::ConstPtr &msg);      // 接受地理信息消息

  void heartbeatTopicCallback(const heartbeat_msgs::Heartbeat::ConstPtr &msg);
  void obstacleTopicCallback(const perception_msgs::PredictionObstacles::ConstPtr &msg);

  void send(const ros::TimerEvent &e);

  void LightHornWiperTopicCallback(const driver_msgs::LightHornWiperReport::ConstPtr &msg);

  void platoonmembersCallback(const platoon_msgs::PlatoonMember::ConstPtr &msg);
  void platoonmembersselfCallback(const platoon_msgs::PlatoonMember::ConstPtr &msg);
  void platoonmissionCallback(const platoon_msgs::PlatoonMission::ConstPtr &msg);
  void platoonmissionselfCallback(const platoon_msgs::PlatoonMission::ConstPtr &msg);

signals:
  // void sendLocalizationValue(double time, double x, double y, double theta);
  void rosShutdown();
  // void emitTopicData(QString);
  void emitXW5651(GNSS_IMU_ST msg);                        // 发布定位数据
  void emitVehicleChassisState(VehicleChassisStateST msg); // 发布车辆状态
  void emitTrajectory(std::vector<GlobalPositionST> listPoint);
  void emitGlobalPath84(std::vector<localPositionST> listPoint);
  void emitDilixinxi(DilixinxiST listPoint);
  void emitObstacle(std::vector<ObstacleST> listPoint);
  void emitHeartBeat(HeartBeatST heart_beat);

  void emitVehicleMotionState(VehicleMotionStateST msg);
  void emitReferencePathFeedback(QList<SmoothPathPointST> msg);
  void emitControlError(ControlErrorST msg);
  void emitLocalPath(QList<SmoothPathPointST> mgs);

  void emitLightHornWiperState(ChassisLightHornWiper msg);

  void emitplatoonmembers(const platoon_msgs::PlatoonMember::ConstPtr& msg);
  void emitplatoonmission(const platoon_msgs::PlatoonMission::ConstPtr& msg);

private slots:

  /*主线控*/
  void onSendDriveModeCmd(uint8 drive_mode);
  void onSendTargetGearCmd(uint8 gear_cmd);
  void onsendTargetThrottleAndBrakePct(float throttle_pct, float brake_pct); // change
  void onsendTaskdist(uint8 taskdistclicked);
  void onSendTargetSteeringAngleAndAngleSpeedCmd(float target_angle, float angle_speed);
  void onSendParkBrakeCmd(bool cmd);
  void onSendEngineCmd(bool cmd);
  void onSendLowVoltageCmd(bool cmd);
  void onSendHighVoltageCmd(bool cmd);
  void onSendEmcyBrakeCmd(bool cmd);
  void onSendVehicleResetCmd();
  void onSendMotionStart(uint8 cmd);

  /*远程遥控驾驶*/
  void onSendRemoteDrive(RemoteDriveST data);

  void ontaskpoints(std::vector<TaskPointST> task_points_, uint8 tasktype); // 报文赋值通过ros节点发送
  void onsendTaskPointGet(double lon, double lat, uint8 taskpointgetclicked);
  void onSendTaskPoints(taskPoints_msgs::taskPoints task_list);
  //  void onSendReferencePathCheckCmd();
  //  void onSendControlMode(uint8 mode);

  /*灯笛雨刮*/
  void onSendLeftLightCmd(bool cmd);
  void onSendRightLightCmd(bool cmd);
  void onSendEmcyFlasherCmd(bool cmd);
  void onSendLowBeamCmd(bool cmd);
  void onSendHighBeamCmd(bool cmd);
  void onSendHonkCmd(bool cmd);
  void onSendBrakeLightCmd(bool cmd);
  void onSendFrontFoggyLightCmd(bool cmd);
  void onSendRearFoggyLightCmd(bool cmd);
  void onSendPositionLightCmd(bool cmd);
  void onSendReverseLightCmd(bool cmd);
  void onSendWiperCmd(bool cmd);
  void onsendHeadLightCmd(bool cmd);

  // platoon
  void OnSendPlatoonBuildCmd();
  void OnSendPlatoonDissloveCmd();
  void OnSendPlatoonJoinCmd(int carnum);
  void OnSendPlatoonLeaveCmd(int carnum);
  void OnSendPlatoonColumnCmd();
  void OnSendPlatoonDiamondCmd();
  //轨迹录制槽函数
  void OnSendRecordCmd(int recordcmd);
  //倒车命令槽函数
  void OnSendBackWordCmd(int backwordcmd);
//  void OnSendPlatoonMotionStartCmd();
//  void OnSendPlatoonMotionStopCmd();

private:
  int init_argc;
  char **init_argv;
  std::shared_ptr<ros::NodeHandle> nh_;

  std::ofstream f_gps_data_out;

  // sub
  ros::Subscriber localization_sub_;
  ros::Subscriber chassis_report_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber global_path_wgs84_sub;
  ros::Subscriber dilixinxi_sub;
  ros::Subscriber obstacle_sub_;
  ros::Subscriber heartbeat_sub_;
  ros::Subscriber light_horn_wiper_sub_;
  ros::Subscriber platoonmembers_sub_;
  ros::Subscriber platoonmembersself_sub_;
  ros::Subscriber platoonmission_sub_;
  ros::Subscriber platoonmissionself_sub_;

  ros::Publisher mode_cmd_pub_;
  ros::Publisher light_horn_wiper_cmd_pub_;
  ros::Publisher motion_start_pub_;
  ros::Publisher emgency_cmd_pub_;

  // publisher remote
  ros::Publisher remote_drive_cmd_pub_;
  ros::Publisher remote_gear_cmd_pub_;
  ros::Publisher remote_power_cmd_pub_;
  ros::Publisher remote_parking_brake_cmd_pub_;
  ros::Publisher remote_steeringwheel_cmd_pub_;
  ros::Publisher remote_reset_cmd_pub_;

  // publisher auto
  ros::Publisher auto_drive_cmd_pub_;
  ros::Publisher auto_gear_cmd_pub_;
  ros::Publisher auto_power_cmd_pub_;
  ros::Publisher auto_parking_brake_cmd_pub_;
  ros::Publisher auto_steeringwheel_cmd_pub_;
  ros::Publisher auto_reset_cmd_pub_;

  ros::Publisher taskPoint_pub_;
  ros::Publisher heartbeat_pub_;

  // time
  ::ros::Timer send_timer_;

  GNSS_IMU_ST gnss_imu_qt_msg_;
  driver_msgs::LightHornWiperCmd light_horn_wiper_cmd_;
  YAML::Node config_;
  YAML::Node config_global_;

  int global_path_jump_point_size_ = 1;

  /*远程遥控驾驶指令*/
  driver_msgs::DriveCmd remote_cmd_drive_cmd_;
  driver_msgs::GearCmd remote_cmd_gear_cmd_;
  driver_msgs::ModeCmd remote_cmd_mode_cmd_;
  driver_msgs::ParkingBrakeCmd remote_cmd_parkingbrake_cmd_;
  driver_msgs::SteeringWheelCmd remote_cmd_steering_wheelcmd_;
  driver_msgs::PowerCmd remote_cmd_power_cmd_;

  /*无人驾驶指令*/
  driver_msgs::DriveCmd auto_cmd_drive_cmd_;
  driver_msgs::GearCmd auto_cmd_gear_cmd_;
  driver_msgs::ModeCmd auto_cmd_mode_cmd_;
  driver_msgs::ParkingBrakeCmd auto_cmd_parkingbrake_cmd_;
  driver_msgs::SteeringWheelCmd auto_cmd_steering_wheelcmd_;
  driver_msgs::PowerCmd auto_cmd_power_cmd_;

  // 全局指令
  driver_msgs::EstopCmd estop_cmd_;
  driver_msgs::ModeCmd mode_cmd_;

  uint8_t last_gear_location_ = 0;

  VehicleType vehicle_type_;


  //platoon
 ros::Publisher platoon_cmd_pub_;
 platoon_msgs::PlatoonOpt platoonOpt_cmd_;
 platoon_msgs::PlatoonMember platoonmembers;
 //发送轨迹录制
 ros::Publisher record_cmd_pub_;
 //发送倒车命令
 ros::Publisher backword_cmd_pub_;
};

#endif
