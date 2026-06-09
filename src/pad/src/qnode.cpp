#include <ros/ros.h>
#include "pad/qnode.h"
#include <string>
#include <memory>
#include <iostream>

QRosNode::QRosNode(int argc, char **argv) : init_argc(argc),
                                            init_argv(argv)
{
  init();
}

QRosNode::~QRosNode()
{
  if (ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }

  wait();
}

bool QRosNode::init()
{
  ros::init(init_argc, init_argv, "pad_node");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  nh_ = std::make_shared<ros::NodeHandle>();

  std::string pkg_dir = ros::package::getPath("launch_node");
  std::string config_file_path = pkg_dir + std::string("/param/pad/pad.yaml");

  std::string home = getenv("HOME");
  std::string config_global_file_path = std::string(home + "/vehicle_platform.yaml");

  // vehicle_type: "sanzhou"  # 车辆类型  geometry_c  sanzhou tank500 zhito zw zhitong

  config_ = YAML::LoadFile(config_file_path);
  config_global_ = YAML::LoadFile(config_global_file_path);

  std::string vehicle_type = config_global_["vehicle_type"].as<std::string>();

  if ("zhitong" == vehicle_type) //
  {
    vehicle_type_ = VehicleType::ZHI_TONG;
  }
  else if ("zhito" == vehicle_type)
  {
    vehicle_type_ = VehicleType::ZHITO_TRUCK;
  }
  else if ("tank500" == vehicle_type)
  {
    vehicle_type_ = VehicleType::TANK_500;
  }
  else if ("sanzhou" == vehicle_type)
  {
    // szc
    vehicle_type_ = VehicleType::SZC;
  }
  else if ("zw" == vehicle_type)
  {
    vehicle_type_ = VehicleType::ZW;
  }
  else if ("x6000" == vehicle_type)
  {
    vehicle_type_ = VehicleType::X6000;
  }
  else if ("xb" == vehicle_type)
  {
    vehicle_type_ = VehicleType::XB;
  }
  else
  {
    throw std::runtime_error{"dont has this vehicle type"};
  }

  global_path_jump_point_size_ = config_["global_path_jump_point_size"].as<int>();

  // Add your ros communications here.

  chassis_report_sub_ = nh_->subscribe("/chassis", 1, &QRosNode::chassisTopicCallback, this);
  localization_sub_ = nh_->subscribe("/odomData", 1, &QRosNode::localizationTopicCallback, this);
  trajectory_sub_ = nh_->subscribe("/trajectory", 1, &QRosNode::trajectoryTopicCallback, this);
  global_path_wgs84_sub = nh_->subscribe("/global_gps_orign_way_pub", 1, &QRosNode::GlobalPath84callback, this);
  dilixinxi_sub = nh_->subscribe("/scene_output", 1, &QRosNode::Dilixinxicallback, this); // 地理信息话题名称
  obstacle_sub_ = nh_->subscribe("/all_output", 1, &QRosNode::obstacleTopicCallback, this);
  heartbeat_sub_ = nh_->subscribe("/heartbeat", 30, &QRosNode::heartbeatTopicCallback, this);
  light_horn_wiper_sub_ = nh_->subscribe("/chassis_light_horn_wiper", 1, &QRosNode::LightHornWiperTopicCallback, this);
  platoonmembers_sub_ = nh_->subscribe("/PlatoonMember", 10, &QRosNode::platoonmembersCallback, this);
  platoonmembersself_sub_ = nh_->subscribe("/PlatoonMember_self", 10, &QRosNode::platoonmembersselfCallback, this);
  platoonmission_sub_ = nh_->subscribe("/PlatoonMission", 5, &QRosNode::platoonmissionCallback, this);
  platoonmissionself_sub_ = nh_->subscribe("/PlatoonMission_self", 5, &QRosNode::platoonmissionselfCallback, this);

  // global pub
  mode_cmd_pub_ = nh_->advertise<driver_msgs::ModeCmd>("/vehicle_cmd_gate_mode_cmd", 1);
  light_horn_wiper_cmd_pub_ = nh_->advertise<driver_msgs::LightHornWiperCmd>("/chassis_light_horn_wiper_cmd", 1);
  motion_start_pub_ = nh_->advertise<driver_msgs::MotionStartCmd>("/chassis_motion_start_cmd", 1);
  emgency_cmd_pub_ = nh_->advertise<driver_msgs::EstopCmd>("/estop_cmd", 1);

  // remote pub
  remote_drive_cmd_pub_ = nh_->advertise<driver_msgs::DriveCmd>("/remote_chassis_drive_cmd", 1);
  remote_gear_cmd_pub_ = nh_->advertise<driver_msgs::GearCmd>("/remote_chassis_gear_cmd", 1);
  remote_parking_brake_cmd_pub_ = nh_->advertise<driver_msgs::ParkingBrakeCmd>("/remote_chassis_parking_brake_cmd", 1);
  remote_steeringwheel_cmd_pub_ = nh_->advertise<driver_msgs::SteeringWheelCmd>("/remote_chassis_steeringwheel_cmd", 1);
  remote_reset_cmd_pub_ = nh_->advertise<driver_msgs::ResetCmd>("/remote_chassis_reset_cmd", 1);
  remote_power_cmd_pub_ = nh_->advertise<driver_msgs::PowerCmd>("/remote_chassis_power_cmd", 1);

  // auto pub
  auto_drive_cmd_pub_ = nh_->advertise<driver_msgs::DriveCmd>("/auto_chassis_drive_cmd", 1);
  auto_gear_cmd_pub_ = nh_->advertise<driver_msgs::GearCmd>("/auto_chassis_gear_cmd", 1);
  auto_parking_brake_cmd_pub_ = nh_->advertise<driver_msgs::ParkingBrakeCmd>("/auto_chassis_parking_brake_cmd", 1);
  auto_steeringwheel_cmd_pub_ = nh_->advertise<driver_msgs::SteeringWheelCmd>("/auto_chassis_steeringwheel_cmd", 1);
  auto_reset_cmd_pub_ = nh_->advertise<driver_msgs::ResetCmd>("/auto_chassis_reset_cmd", 1);
  auto_power_cmd_pub_ = nh_->advertise<driver_msgs::PowerCmd>("/auto_chassis_power_cmd", 1);

  // platoon

//  platoon_cmd_pub_ = nh_->advertise<std_msgs::Int32>("/platoon_opt", 10);
  platoon_cmd_pub_ = nh_->advertise<platoon_msgs::PlatoonOpt>("platoon_opt", 10);
  //轨迹录制
  record_cmd_pub_ = nh_->advertise<std_msgs::Int32>("/record_flag", 5);
  //倒车信号
  backword_cmd_pub_ = nh_->advertise<std_msgs::Int32>("shift_forward", 5);
  // other
  taskPoint_pub_ = nh_->advertise<taskPoints_msgs::taskPoints>("/dstPoint", 1);
  heartbeat_pub_ = nh_->advertise<heartbeat_msgs::Heartbeat>("/heartbeat", 1);

  send_timer_ = nh_->createTimer(::ros::Duration(0.5), &QRosNode::send, this);

  // sleep(15); //wait 7 seconds before starting that whole log is not shown at startup
  start(); // start a Qthread, which calls run()

  memset(&light_horn_wiper_cmd_, 0, sizeof(light_horn_wiper_cmd_));
  std::cout << "Successfully initialized node." << std::endl;

  return true;
}

void QRosNode::heartbeatTopicCallback(const heartbeat_msgs::Heartbeat::ConstPtr &msg)
{
  // std::cout << " get heart beat  flag  " << (int)(msg->flag) << " heart beat" << (int)(msg->heart_beat) << std::endl;
  HeartBeatST heart_beat;
  heart_beat.heart_beat = msg->heart_beat;
  heart_beat.flag = msg->flag;
  emit emitHeartBeat(heart_beat);
}

void QRosNode::chassisTopicCallback(const driver_msgs::ChassisReport::ConstPtr &msg)
{
  // std::cout << "chassisTopicCallback  start" << std::endl;
  // fflush(stdout);
  VehicleChassisStateST state;
  state.adu_mode = msg->adu_mode;
  state.auto_to_manual_tips = msg->auto_to_manual_tips;
  state.bcm_mode = msg->bcm_mode;
  state.brake_light = msg->brake_light;
  state.brake_pedal = msg->brake_pedal;
  state.brake_pedal_manual = msg->brake_pedal_manual;
  state.current_acceleration = msg->current_acceleration;
  state.current_engine_speed = msg->current_engine_speed;
  state.current_engine_temperature = msg->current_engine_temperature;
  state.current_engine_torque = msg->current_engine_torque;
  state.current_velocity = msg->current_velocity;
  state.driving_mode = msg->driving_mode;
  state.emergency_light = msg->emergency_light;
  state.emergency_stop = msg->emergency_stop;
  state.epb_mode = msg->epb_mode;
  state.epb_state = msg->epb_state;
  state.eps_mode = msg->eps_mode;
  state.front_foggy_light = msg->front_foggy_light;
  state.front_wheel_angle = msg->front_wheel_angle;
  state.gear_location = msg->gear_location;
  state.high_light = msg->high_light;
  state.high_voltage_signal = msg->high_voltage_signal;
  state.horn = msg->horn;
  state.left_light = msg->left_light;
  state.low_light = msg->low_light;
  state.low_voltage_signal = msg->low_voltage_signal;
  state.parking_brake = msg->parking_brake;
  state.parking_brake_switch = msg->parking_brake_switch;
  state.position_light = msg->position_light;
  state.rear_foggy_light = msg->rear_foggy_light;
  state.rear_wheel_angle = msg->rear_wheel_angle;
  state.remaining_electricity = msg->remaining_electricity;
  state.remaining_oil = msg->remaining_oil;
  state.reverse_light = msg->reverse_light;
  state.right_light = msg->right_light;
  state.steering_wheel_angle = msg->steering_wheel_angle;
  state.steering_wheel_angle_speed = msg->steering_wheel_angle_speed;
  state.steering_wheel_hand_moment_signal = msg->steering_wheel_hand_moment_signal;
  state.steer_mode = msg->steer_mode;
  state.throttle_pedal = msg->throttle_pedal;
  state.throttle_pedal_manual = msg->throttle_pedal_manual;
  state.total_kilometres = msg->total_kilometres;
  state.wheel_speed_fl = msg->wheel_speed_fl;
  state.wheel_speed_fr = msg->wheel_speed_fr;
  state.wheel_speed_rl = msg->wheel_speed_rl;
  state.wheel_speed_rr = msg->wheel_speed_rr;
  state.wiper = msg->wiper;
  state.xbr_active_mode = msg->xbr_active_mode;
  state.xbr_mode = msg->xbr_mode;
  state.xbr_system_state = msg->xbr_system_state;
  state.auto_switch = msg->auto_switch;
  state.steer_intervene = msg->steer_intervene;
  // std::cout << msg->brake_intervene << std::endl;
  state.brake_intervene = msg->brake_intervene;
  state.estop_intervene = msg->estop_intervene;
  state.timeout_status = msg->timeout_status;
  state.remote_button_status = msg->remote_button_status;
  state.is_ready = msg->ready;
  state.mode_flag = msg->mode_flag;
  emit emitVehicleChassisState(state);
  // std::cout << "chassisTopicCallback  end" << std::endl;
  // fflush(stdout);

  ChassisLightHornWiper light_state;
  light_state.left_light = msg->left_light;
  light_state.right_light = msg->right_light;
  light_state.high_light = msg->high_light;
  light_state.low_light = msg->low_light;
  light_state.brake_light = msg->brake_light;
  light_state.emergency_light = msg->emergency_light;
  light_state.front_foggy_light = msg->front_foggy_light;
  light_state.rear_foggy_light = msg->rear_foggy_light;
  light_state.position_light = msg->position_light;
  light_state.reverse_light = msg->reverse_light;
  light_state.horn = msg->horn;
  light_state.wiper = msg->wiper;
  light_state.head_light = msg->head_light;

  emit emitLightHornWiperState(light_state);
}

void QRosNode::run()
{
  // QThread function
  ros::Rate loop_rate(20); // too fast loop rate crashes the GUI
  std::cout << "Node is running." << std::endl;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QRosNode::localizationTopicCallback(const localization_msgs::Localization::ConstPtr &msg)
{

  //  std::cout <<"callback odom data"<<std::endl;
  // std::cout << "localizationTopicCallback  start" << std::endl;
  // fflush(stdout);
  memset(&gnss_imu_qt_msg_, 0, sizeof(GNSS_IMU_ST));

  gnss_imu_qt_msg_.is_valid = msg->is_valid;
  gnss_imu_qt_msg_.gps_week = msg->original_ins.gps_week;
  gnss_imu_qt_msg_.gps_time = msg->original_ins.gps_time;

  gnss_imu_qt_msg_.latitude = msg->original_ins.latitude;
  gnss_imu_qt_msg_.longitude = msg->original_ins.longitude;
  gnss_imu_qt_msg_.altitude = msg->original_ins.altitude;
  gnss_imu_qt_msg_.ellip_height = msg->original_ins.ellip_height;

  gnss_imu_qt_msg_.east_speed = msg->original_ins.east_speed;
  gnss_imu_qt_msg_.north_speed = msg->original_ins.north_speed;
  gnss_imu_qt_msg_.up_speed = msg->original_ins.sky_speed;
  gnss_imu_qt_msg_.ground_speed = msg->original_ins.ground_speed;
  gnss_imu_qt_msg_.angle_heading = msg->original_ins.yaw;
  gnss_imu_qt_msg_.angle_pitch = msg->original_ins.pitch;
  gnss_imu_qt_msg_.angle_roll = msg->original_ins.roll;

  gnss_imu_qt_msg_.acc_x = msg->original_ins.acc_x;
  gnss_imu_qt_msg_.acc_y = msg->original_ins.acc_y;
  gnss_imu_qt_msg_.acc_z = msg->original_ins.acc_z;
  gnss_imu_qt_msg_.gyro_x = msg->original_ins.angular_x;
  gnss_imu_qt_msg_.gyro_y = msg->original_ins.angular_y;
  gnss_imu_qt_msg_.gyro_z = msg->original_ins.angular_z;

  // TODO: utm
  // gnss_imu_qt_msg_.utm_x = msg->original_ins.utm_x;
  // gnss_imu_qt_msg_.utm_y = msg->original_ins.utm_y;
  // gnss_imu_qt_msg_.utm_z = msg->original_ins.utm_z;

  // 定位消息中需要补充卫星星数
  gnss_imu_qt_msg_.main_star_num = msg->original_ins.satellite_num;
  gnss_imu_qt_msg_.aux_star_num = msg->original_ins.satellite_num;
  gnss_imu_qt_msg_.main_star_num_searching = msg->original_ins.satellite_num_sats;
  gnss_imu_qt_msg_.aux_star_num_searching = msg->original_ins.satellite_num_sats_1;

  gnss_imu_qt_msg_.nav_uncertainty = msg->original_ins.nav_uncertainty;
  gnss_imu_qt_msg_.satellite_status = msg->original_ins.satellite_status;
  gnss_imu_qt_msg_.system_state = msg->original_ins.system_state;

  emit emitXW5651(gnss_imu_qt_msg_);
  // std::cout << "localizationTopicCallback  end" << std::endl;
  // fflush(stdout);
}

void QRosNode::LightHornWiperTopicCallback(const driver_msgs::LightHornWiperReport::ConstPtr &msg)
{
  ChassisLightHornWiper state;
  state.left_light = msg->left_light;
  state.right_light = msg->right_light;
  state.high_light = msg->high_light;
  state.low_light = msg->low_light;
  state.brake_light = msg->brake_light;
  state.emergency_light = msg->emergency_light;
  state.front_foggy_light = msg->front_foggy_light;
  state.rear_foggy_light = msg->rear_foggy_light;
  state.position_light = msg->position_light;
  state.reverse_light = msg->reverse_light;
  state.horn = msg->horn;
  state.wiper = msg->wiper;
  state.head_light = msg->head_light;

  emit emitLightHornWiperState(state);
}

void QRosNode::trajectoryTopicCallback(const planning_msgs::TrajectoryPointArray::ConstPtr &msg)
{
  // std::cout << "trajectoryTopicCallback  start" << std::endl;
  // fflush(stdout);

  std::vector<GlobalPositionST> listPoint;
  listPoint.clear();
  GlobalPositionST stPoint;
  //  memset(&stPoint, 0, sizeof(GlobalPositionST));
  for (size_t i = 0; i < msg->points.size(); i++)
  {
    stPoint.emCoordinate = 3; /*UTM坐标系*/
    stPoint.bIsSouth = 0;     /*北半球*/
    stPoint.ucUTMZoneID = 5;  // m_stPNC2PadHeartMsg.stGlobalPostion.ucUTMZoneID;
    stPoint.fX = msg->points.at(i).x;
    stPoint.fY = msg->points.at(i).y;
    stPoint.fZ = msg->points.at(i).z;
    //  std::cout<<stPoint.fX<<std::endl;
    listPoint.push_back(stPoint);
  }
  // std::cout<<"list daxiao"<<listPoint.size()<<std::endl;

  emit emitTrajectory(listPoint);
  // std::cout << "trajectoryTopicCallback  end" << std::endl;
  // fflush(stdout);

  // emit emitTrajectory(listPoint);
}

void QRosNode::GlobalPath84callback(const lanelet_map_msgs::Way::ConstPtr &msg)
{
  // std::cout << "GlobalPath84callback  start" << std::endl;
  // fflush(stdout);

  if (msg->points.empty())
  {
    std::cout << "global path empty " << std::endl;
    return;
  }
  std::vector<localPositionST> listPoint;
  listPoint.clear();
  localPositionST stPoint;
  memset(&stPoint, 0, sizeof(localPositionST));
  int index = 0;
  for (size_t i = 0; i < msg->points.size(); i = i + global_path_jump_point_size_)
  {
    stPoint.lon = msg->points.at(i).point.x;
    stPoint.lat = msg->points.at(i).point.y;
    //  std::cout<<stPoint.fX<<std::endl;
    listPoint.push_back(stPoint);
    index = i;
  }

  if (index != (msg->points.size() - 1))
  {
    stPoint.lon = msg->points.at((msg->points.size() - 1)).point.x;
    stPoint.lat = msg->points.at((msg->points.size() - 1)).point.y;
    listPoint.push_back(stPoint);
  }
  // std::cout<<"list daxiao"<<listPoint.size()<<std::endl;
  emit emitGlobalPath84(listPoint);

  // std::cout << "GlobalPath84callback  end" << std::endl;
  // fflush(stdout);
}

void QRosNode::Dilixinxicallback(const perception_msgs::PredictionObstacles::ConstPtr &msg)
{

  //  std::cout << "Dilixinxicallback " << std::endl;
  /*
  DilixinxiST listPoint;
  listPoint.fX = msg->position.x;
  listPoint.fY = msg->position.y;
  listPoint.fZ = msg->position.z;
  listPoint.heading = msg->heading;
  listPoint.type = msg->type;
  emit emitDilixinxi(listPoint);
  */
  // std::cout << "Dilixinxicallback  start" << std::endl;
  // fflush(stdout);

  std::vector<ObstacleST> listPoint;
  listPoint.clear();
  ObstacleST stPoint;
  for (int i = 0; i < msg->prediction_obstacles.size(); i++)
  {
    // std::cout << "obs  points size = " << msg->prediction_obstacles[i].perception_obstacle.polygon.points.size() << std::endl;
    // fflush(stdout);
    stPoint.fX_1 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(1).x;
    stPoint.fY_1 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(1).y;
    stPoint.fZ_1 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(1).z;
    stPoint.fX_2 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(2).x;
    stPoint.fY_2 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(2).y;
    stPoint.fZ_2 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(2).z;
    stPoint.fX_3 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(3).x;
    stPoint.fY_3 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(3).y;
    stPoint.fZ_3 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(3).z;
    stPoint.fX_4 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(4).x;
    stPoint.fY_4 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(4).y;
    stPoint.fZ_4 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(4).z;
    stPoint.type = msg->prediction_obstacles[i].perception_obstacle.type;
    stPoint.id = msg->prediction_obstacles[i].perception_obstacle.id;

    listPoint.push_back(stPoint);
  }
  emit emitObstacle(listPoint);
  // std::cout << "Dilixinxicallback  end" << std::endl;
  // fflush(stdout);
}

/*
void QRosNode::heartBeatTopicCallback(const heartbeat_msgs::Heartbeat msgs::ConstPtr &msg)
{
  // 0 reference_line
  // 1 planner;
  // 2 controller;

}
*/
void QRosNode::obstacleTopicCallback(const perception_msgs::PredictionObstacles::ConstPtr &msg)
{
  // std::cout << "obstacleTopicCallback  start" << std::endl;
  // fflush(stdout);
  std::vector<ObstacleST> listPoint;
  listPoint.clear();
  ObstacleST stPoint;
  for (int i = 0; i < msg->prediction_obstacles.size(); i++)
  {
    // std::cout << "obs  points size = " << msg->prediction_obstacles[i].perception_obstacle.polygon.points.size() << std::endl;
    // fflush(stdout);
    stPoint.fX_1 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(1).x;
    stPoint.fY_1 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(1).y;
    stPoint.fZ_1 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(1).z;
    stPoint.fX_2 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(2).x;
    stPoint.fY_2 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(2).y;
    stPoint.fZ_2 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(2).z;
    stPoint.fX_3 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(3).x;
    stPoint.fY_3 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(3).y;
    stPoint.fZ_3 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(3).z;
    stPoint.fX_4 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(4).x;
    stPoint.fY_4 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(4).y;
    stPoint.fZ_4 = msg->prediction_obstacles[i].perception_obstacle.polygon.points.at(4).z;
    stPoint.type = msg->prediction_obstacles[i].perception_obstacle.type;
    stPoint.id = msg->prediction_obstacles[i].perception_obstacle.id;

    listPoint.push_back(stPoint);
  }
  emit emitObstacle(listPoint);
  // std::cout << "obstacleTopicCallback  end" << std::endl;
  // fflush(stdout);
}

void QRosNode::onSendDriveModeCmd(uint8 drive_mode)
{

  // 按钮定义：0 人工 1无人 2遥控 3人工反向 4 单横向 5 单纵向

  // 驾驶模式   0:人工, 1:无人, 2:遥控

  if (0 == drive_mode)
  {
    mode_cmd_.driving_mode = 0;
    mode_cmd_.mode_flag = 0;
    mode_cmd_.adu_mode = 0;
    mode_cmd_.bcm_mode = 0;
    mode_cmd_.epb_mode = 0;
    mode_cmd_.eps_mode = 0;
    mode_cmd_.xbr_mode = 0;
    mode_cmd_.gear_mode = 0;
    mode_cmd_.bcm_mode = 0;
  }
  else if (1 == drive_mode)
  {
    mode_cmd_.driving_mode = 1;
    mode_cmd_.mode_flag = 0;
    mode_cmd_.adu_mode = 1;
    mode_cmd_.bcm_mode = 1;
    mode_cmd_.epb_mode = 1;
    mode_cmd_.eps_mode = 1;
    mode_cmd_.xbr_mode = 1;
    mode_cmd_.gear_mode = 1;
  }
  else if (2 == drive_mode)
  {
    mode_cmd_.driving_mode = 1;
    mode_cmd_.mode_flag = 1;
    mode_cmd_.adu_mode = 1;
    mode_cmd_.bcm_mode = 1;
    mode_cmd_.epb_mode = 1;
    mode_cmd_.eps_mode = 1;
    mode_cmd_.xbr_mode = 1;
    mode_cmd_.gear_mode = 1;
  }
  else if (3 == drive_mode)
  {
    mode_cmd_.driving_mode = 1;
    mode_cmd_.mode_flag = 2;
    mode_cmd_.adu_mode = 1;
    mode_cmd_.bcm_mode = 1;
    mode_cmd_.epb_mode = 1;
    mode_cmd_.eps_mode = 1;
    mode_cmd_.xbr_mode = 1;
    mode_cmd_.gear_mode = 1;
  }
  else if (4 == drive_mode)
  {
    mode_cmd_.driving_mode = 4;
    mode_cmd_.mode_flag = 0;
    mode_cmd_.adu_mode = 0;
    mode_cmd_.bcm_mode = 0;
    mode_cmd_.epb_mode = 0;
    mode_cmd_.eps_mode = 1;
    mode_cmd_.xbr_mode = 0;
    mode_cmd_.gear_mode = 1;
  }

  else if (5 == drive_mode)
  {
    mode_cmd_.driving_mode = 5;
    mode_cmd_.mode_flag = 0;
    mode_cmd_.adu_mode = 1;
    mode_cmd_.bcm_mode = 1;
    mode_cmd_.epb_mode = 1;
    mode_cmd_.eps_mode = 0;
    mode_cmd_.xbr_mode = 1;
    mode_cmd_.gear_mode = 1;
  }
  else
  {
    return;
  }

  mode_cmd_pub_.publish(mode_cmd_);
}

// gear_cmd_pub_ = nh_->advertise<driver_msgs::GearCmd>("/chassis_gear_cmd", 1);

void QRosNode::onSendEmcyBrakeCmd(bool cmd)
{
  estop_cmd_.estop = cmd;
  emgency_cmd_pub_.publish(estop_cmd_);
}

void QRosNode::onSendTargetGearCmd(uint8 gear_cmd)
{
  // std::cout << "gear cmd " << gear_cmd << std::endl;
  driver_msgs::GearCmd cmd;
  if (0 == gear_cmd) // 0 n 1 d 2 p 7 r
  {
    cmd.gear_location = gear_cmd;
  }
  else if (1 == gear_cmd)
  {
    cmd.gear_location = gear_cmd;
  }
  else if (2 == gear_cmd)
  {
    cmd.gear_location = gear_cmd;
  }
  else if (7 == gear_cmd)
  {
    cmd.gear_location = gear_cmd;
  }
  else
  {
    return;
  }
  if (mode_cmd_.driving_mode == 1)
  {
    if (mode_cmd_.mode_flag == 0)
    {
      auto_gear_cmd_pub_.publish(cmd);
    }
    else if (mode_cmd_.mode_flag == 1)
    {
      remote_gear_cmd_pub_.publish(cmd);
    }
    else if (mode_cmd_.mode_flag == 2)
    {
      remote_gear_cmd_pub_.publish(cmd);
    }
    else
    {
      // do nothing
    }
  }
}

void QRosNode::onsendTargetThrottleAndBrakePct(float throttle, float brake_pct)
{
  driver_msgs::DriveCmd cmd;
  if (vehicle_type_ == VehicleType::ZHITO_TRUCK)
  {
    cmd.throttle_pedal = throttle;
    double brake_pedal = brake_pct * (-6.0) / 100.0;
    cmd.acc_target = brake_pedal;
  }
  else if (vehicle_type_ == VehicleType::ZHI_TONG)
  {
    cmd.throttle_pedal = throttle;
    cmd.brake_pedal = brake_pct;
  }
  else if (vehicle_type_ == VehicleType::TANK_500)
  {
    cmd.throttle_pedal = throttle;
    cmd.brake_pedal = brake_pct;
  }
  else if (vehicle_type_ == VehicleType::SZC)
  {
    cmd.throttle_pedal = throttle;
    cmd.brake_pedal = brake_pct;
  }
  else if (vehicle_type_ == VehicleType::ZW)
  {
    cmd.throttle_pedal = throttle;
    cmd.brake_pedal = brake_pct;
  }
  else if (vehicle_type_ == VehicleType::XB)
  {
    cmd.throttle_pedal = throttle;
    cmd.brake_pedal = brake_pct;
  }
  else if (vehicle_type_ == VehicleType::X6000)
  {
    cmd.throttle_pedal = throttle;
    double brake_pedal = brake_pct * (-6.0) / 100.0;
    cmd.acc_target = brake_pedal;
  }
  else
  {
    return;
  }

  if (mode_cmd_.driving_mode == 1)
  {
    if (mode_cmd_.mode_flag == 0)
    {
      auto_drive_cmd_pub_.publish(cmd);
    }
    else if (mode_cmd_.mode_flag == 1)
    {
      remote_drive_cmd_pub_.publish(cmd);
    }
    else if (mode_cmd_.mode_flag == 2)
    {
      remote_drive_cmd_pub_.publish(cmd);
    }
    else
    {
      // do nothing
    }
  }
}

void QRosNode::onsendTaskdist(uint8 taskdistclicked)
{
  taskPoints_msgs::taskPoints task_list_pub_;
  taskPoints_msgs::TaskNode node;

  std::vector<TaskPointST> task_points;
  task_points.clear();

  std::string pkg_dir = ros::package::getPath("launch_node") + "/data/";
  std::string task_points_file = pkg_dir + config_["task_points_file"].as<std::string>();
  std::ifstream filename(task_points_file);
  if (!filename)
  {
    std::cout << " file open error: " << task_points_file << std::endl;
    return;
  }

  std::string oneLine;
  // int first = 0;
  while (getline(filename, oneLine))
  {
    // std::cout << " oneline " << oneLine << std::endl;

    TaskPointST point;
    replace(oneLine.begin(), oneLine.end(), ';', ' '); // 将txt数据中分隔符；替换为空格

    // std::cout << " oneline " << oneLine << std::endl;
    double lat = 0;
    double lon = 0;
    double alt = 0;
    uint16 type = 0;
    uint16 id = 0;

    std::istringstream streamOneLine(oneLine);
    streamOneLine >> id;
    streamOneLine >> lon;
    streamOneLine >> lat;
    streamOneLine >> alt;
    streamOneLine >> type;

    // std::cout << "  11111 " << lat << " " << lon << " " <<(int) type << " " << id << std::endl;

    point.id = id;
    point.lon = lon;
    point.lat = lat;
    point.type = type;
    task_points.push_back(point);
  }
  filename.close();

  if (taskdistclicked == 0x00)
  {
    task_list_pub_.is_configured = true;
    task_list_pub_.cmd_start_plan = false;
    std::cout << "任务下发" << std::endl;
  }
  else if (taskdistclicked == 0x01)
  {
    task_list_pub_.is_configured = true;
    task_list_pub_.cmd_start_plan = true;
    std::cout << "任务启动" << std::endl;
  }
  else if (taskdistclicked == 0x02)
  {
    task_list_pub_.is_configured = true;
    task_list_pub_.cmd_start_plan = false;
    std::cout << "任务暂停" << std::endl;
  }
  else if (taskdistclicked == 0x03)
  {
    task_list_pub_.is_configured = false;
    task_list_pub_.cmd_start_plan = false;
    std::cout << "任务停止" << std::endl;
  }
  task_list_pub_.task_mode = 0;
  task_list_pub_.patrol_mode = 0;
  task_list_pub_.patrol_number = 0;

  for (int i = 0; i < task_points.size(); i++)
  {
    node.type = task_points.at(i).type;
    node.id = task_points.at(i).id;
    node.longitude = task_points.at(i).lon;
    node.latitude = task_points.at(i).lat;
    std::cout << " node.type  " << (int)node.type << " node.id  " << node.id << " node.longitude  " << node.longitude
              << " node.latitude  " << node.latitude << std::endl;
    // uint16_t a = '11';
    // std::cout << (int)a <<std::endl;
    task_list_pub_.node.push_back(node);
  }
  taskPoint_pub_.publish(task_list_pub_);
}

void QRosNode::onsendTaskPointGet(double lon, double lat, uint8 taskpointgetclicked)
{
  taskpointgetclicked++;
  std::string recode_data_dir = ros::package::getPath("taskpointdataget");
  std::string recode_data_store_file = recode_data_dir + std::string("/taskpointData.txt");
  f_gps_data_out = std::ofstream(recode_data_store_file);
  f_gps_data_out << std::fixed;
  f_gps_data_out << "point"
                 << "lon"
                 << "lat" << std::endl;
  f_gps_data_out << taskpointgetclicked << " " << lon << " " << lat << std::endl;
  f_gps_data_out.flush();
  f_gps_data_out.close();
}

void QRosNode::onSendTargetSteeringAngleAndAngleSpeedCmd(float target_angle, float angle_speed)
{
  driver_msgs::SteeringWheelCmd cmd;
  cmd.steering_wheel_angle = target_angle;
  cmd.steering_wheel_angle_speed = angle_speed;

  if (mode_cmd_.driving_mode == 1)
  {
    if (mode_cmd_.mode_flag == 0)
    {
      auto_steeringwheel_cmd_pub_.publish(cmd);
    }
    else if (mode_cmd_.mode_flag == 1)
    {
      remote_steeringwheel_cmd_pub_.publish(cmd);
    }
    else if (mode_cmd_.mode_flag == 2)
    {
      remote_steeringwheel_cmd_pub_.publish(cmd);
    }
    else
    {
      // do nothing
    }
  }
}

void QRosNode::onSendParkBrakeCmd(bool cmd)
{
  driver_msgs::ParkingBrakeCmd cmd_msg;
  cmd_msg.parking_brake = cmd;
  if (mode_cmd_.driving_mode == 1)
  {
    if (mode_cmd_.mode_flag == 0)
    {
      auto_parking_brake_cmd_pub_.publish(cmd_msg);
    }
    else if (mode_cmd_.mode_flag == 1)
    {
      remote_parking_brake_cmd_pub_.publish(cmd_msg);
    }
    else if (mode_cmd_.mode_flag == 2)
    {
      remote_parking_brake_cmd_pub_.publish(cmd_msg);
    }
    else
    {
      // do nothing
    }
  }
}

void QRosNode::onSendLeftLightCmd(bool cmd)
{
  light_horn_wiper_cmd_.left_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}
void QRosNode::onSendRightLightCmd(bool cmd)
{
  light_horn_wiper_cmd_.right_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}
void QRosNode::onSendEmcyFlasherCmd(bool cmd)
{
  light_horn_wiper_cmd_.emergency_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}

void QRosNode::onSendLowBeamCmd(bool cmd)
{
  light_horn_wiper_cmd_.low_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}
void QRosNode::onSendHighBeamCmd(bool cmd)
{
  light_horn_wiper_cmd_.high_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}

void QRosNode::onSendHonkCmd(bool cmd)
{
  light_horn_wiper_cmd_.horn = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}

// TODO:
void QRosNode::onSendMotionStart(uint8 cmd)
{
  driver_msgs::MotionStartCmd motion_cmd;
  motion_cmd.motion_start = cmd;
  motion_start_pub_.publish(motion_cmd);
}

void QRosNode::onSendRemoteDrive(RemoteDriveST data) // 遥控驾驶下发data中包含急停，但无msg消息
{
  // 驾驶模式判断，如果和上一次不同，则更新驾驶模式并发送

  std::cout << "get remote drive data" << std::endl;

  // if (data.drive_mode != remote_cmd_mode_cmd_.driving_mode)
  // {
  std::cout << "mode rec " << (int)data.drive_mode << std::endl;
  std::cout << "gear rec " << (int)data.gear_location << std::endl;
  std::cout << "throttle rec " << data.throttle_percent << std::endl;
  std::cout << "brake rec " << data.brake_percent << std::endl;

  if (data.drive_mode == 0) // 人控
  {
    memset(&remote_cmd_mode_cmd_, 0, sizeof(remote_cmd_mode_cmd_));
  }

  //   # 驾驶模式反馈   0:人工, 1:无人
  // uint8 driving_mode
  // uint8 mode_flag # 无人模式下的具体细分:  0 无人 1远程遥控 2反向人工驾驶

  else if (1 == data.drive_mode) // 无人
  {
    remote_cmd_mode_cmd_.driving_mode = 1;
    remote_cmd_mode_cmd_.mode_flag = 0;

    remote_cmd_mode_cmd_.adu_mode = 1;
    remote_cmd_mode_cmd_.bcm_mode = 1;
    remote_cmd_mode_cmd_.epb_mode = 1;
    remote_cmd_mode_cmd_.eps_mode = 1;
    remote_cmd_mode_cmd_.xbr_mode = 1;
    remote_cmd_mode_cmd_.gear_mode = 1;
  }
  else if (2 == data.drive_mode) // 云控
  {
    remote_cmd_mode_cmd_.driving_mode = 1;
    remote_cmd_mode_cmd_.mode_flag = 1;

    remote_cmd_mode_cmd_.adu_mode = 1;
    remote_cmd_mode_cmd_.bcm_mode = 1;
    remote_cmd_mode_cmd_.epb_mode = 1;
    remote_cmd_mode_cmd_.eps_mode = 1;
    remote_cmd_mode_cmd_.xbr_mode = 1;
    remote_cmd_mode_cmd_.gear_mode = 1;
  }
  else
  {
    // do nothing
  }
  mode_cmd_pub_.publish(remote_cmd_mode_cmd_);
  // }

  // 紧急停止或者正常发送纵向指令
  if (data.emergencystop)
  {
    // if (vehicle_type_ == VehicleType::ZHITO_TRUCK)
    // {
    //   remote_cmd_drive_cmd_.throttle_pedal = 0;
    //   remote_cmd_drive_cmd_.brake_pedal = -6; // 仅用于轻卡
    //   remote_drive_cmd_pub_.publish(remote_cmd_drive_cmd_);
    // }

    // if (vehicle_type_ == VehicleType::ZHITO_TRUCK)
    // {
    //   remote_cmd_drive_cmd_.throttle_pedal = 0;
    //   remote_cmd_drive_cmd_.brake_pedal = -6; // 仅用于轻卡
    //   remote_drive_cmd_pub_.publish(remote_cmd_drive_cmd_);

    //   cmd.throttle_pedal = throttle;
    //   double brake_pedal = brake_pct * (-6.0) / 100.0;
    //   cmd.acc_target = brake_pedal;
    // }
    // else if (vehicle_type_ == VehicleType::ZHI_TONG)
    // {
    //   cmd.throttle_pedal = throttle;
    //   cmd.brake_pedal = brake_pct;
    // }
    // else if (vehicle_type_ == VehicleType::TANK_500)
    // {
    //   cmd.throttle_pedal = throttle;
    //   cmd.brake_pedal = brake_pct;
    // }
    // else if (vehicle_type_ == VehicleType::SZC)
    // {
    //   cmd.throttle_pedal = throttle;
    //   cmd.brake_pedal = brake_pct;
    // }
    // else if (vehicle_type_ == VehicleType::ZW)
    // {
    //   cmd.throttle_pedal = throttle;
    //   cmd.brake_pedal = brake_pct;
    // }
    // else if (vehicle_type_ == VehicleType::X6000)
    // {
    //   cmd.throttle_pedal = throttle;
    //   double brake_pedal = brake_pct * (-6.0) / 100.0;
    //   cmd.acc_target = brake_pedal;
    // }
  }
  else
  {
    remote_cmd_drive_cmd_.throttle_pedal = data.throttle_percent;
    // 映射关系
    //  100 -> -6 ?  x  -> y    => 100/x = -6/y  => y = -6.0 * x / 100.0;
    if (vehicle_type_ == VehicleType::ZHITO_TRUCK)
    {
      double brake_pedal = data.brake_percent * (-6.0) / 100.0;
      remote_cmd_drive_cmd_.acc_target = brake_pedal;
      remote_drive_cmd_pub_.publish(remote_cmd_drive_cmd_);
    }
    else if (vehicle_type_ == VehicleType::ZHI_TONG)
    {
      remote_cmd_drive_cmd_.throttle_pedal = data.throttle_percent > 10?10:data.throttle_percent;
      remote_cmd_drive_cmd_.brake_pedal = data.brake_percent;
      
      remote_drive_cmd_pub_.publish(remote_cmd_drive_cmd_);
    }
  }

  // 档位
  // if (data.gear_location != last_gear_location_)
  // {

  if (data.gear_location == 0)
  {
    remote_cmd_gear_cmd_.gear_location = 0;
  }
  else if (data.gear_location == 1)
  {
    remote_cmd_gear_cmd_.gear_location = 7;
  }
  else if (data.gear_location == 2)
  {
    remote_cmd_gear_cmd_.gear_location = 1;
  }
  else
  {
    //"无效档位"
  }
  last_gear_location_ = data.gear_location;
  remote_gear_cmd_pub_.publish(remote_cmd_gear_cmd_);
  // }

  // if (data.parking_stop != remote_cmd_parkingbrake_cmd_.parking_brake)
  // {
  std::cout << "parking_brake rec " << (int)data.parking_stop << std::endl;

  remote_cmd_parkingbrake_cmd_.parking_brake = data.parking_stop;
  remote_parking_brake_cmd_pub_.publish(remote_cmd_parkingbrake_cmd_);
  // }

  // 横向控制
  std::cout << "steer rec " << data.steer_angle << std::endl;

  remote_cmd_steering_wheelcmd_.steering_wheel_angle = data.steer_angle * 1.5;
  remote_cmd_steering_wheelcmd_.steering_wheel_angle_speed = 150; // 由于驾驶模拟器读取不到，所以当前发送一个默认值
  remote_steeringwheel_cmd_pub_.publish(remote_cmd_steering_wheelcmd_);

  // 发动机启动指令，TODO
  // ;
}

void QRosNode::onSendVehicleResetCmd()
{
  driver_msgs::ResetCmd cmd;
  cmd.reset = true;

  if (mode_cmd_.driving_mode == 1)
  {
    if (mode_cmd_.mode_flag == 0)
    {
      auto_reset_cmd_pub_.publish(cmd);
    }
    else if (mode_cmd_.mode_flag == 1)
    {
      remote_reset_cmd_pub_.publish(cmd);
    }
    else if (mode_cmd_.mode_flag == 2)
    {
      remote_reset_cmd_pub_.publish(cmd);
    }
    else
    {
      // do nothing
    }
  }
}

void QRosNode::onSendTaskPoints(taskPoints_msgs::taskPoints task_list) // 报文赋值通过ros节点发送
{
  taskPoints_msgs::taskPoints task_list_pub_;
  task_list_pub_ = task_list;
  taskPoint_pub_.publish(task_list_pub_);
  // std::cout<<"111"<<std::endl;
}

void QRosNode::onSendEngineCmd(bool cmd)
{

  if (mode_cmd_.driving_mode == 1)
  {
    if (mode_cmd_.mode_flag == 0)
    {
      auto_cmd_power_cmd_.engine_on_off = cmd;
      auto_power_cmd_pub_.publish(auto_cmd_power_cmd_);
    }
    else if (mode_cmd_.mode_flag == 1)
    {
      remote_cmd_power_cmd_.engine_on_off = cmd;
      remote_power_cmd_pub_.publish(remote_cmd_power_cmd_);
    }
    else if (mode_cmd_.mode_flag == 2)
    {
      remote_cmd_power_cmd_.engine_on_off = cmd;
      remote_power_cmd_pub_.publish(remote_cmd_power_cmd_);
    }
    else
    {
      // do nothing
    }
  }
}

void QRosNode::onSendLowVoltageCmd(bool cmd)
{
  if (mode_cmd_.driving_mode == 1)
  {
    if (mode_cmd_.mode_flag == 0)
    {
      auto_cmd_power_cmd_.low_voltage_signal = cmd;
      auto_power_cmd_pub_.publish(auto_cmd_power_cmd_);
    }
    else if (mode_cmd_.mode_flag == 1)
    {
      remote_cmd_power_cmd_.low_voltage_signal = cmd;
      remote_power_cmd_pub_.publish(remote_cmd_power_cmd_);
    }
    else if (mode_cmd_.mode_flag == 2)
    {
      remote_cmd_power_cmd_.low_voltage_signal = cmd;
      remote_power_cmd_pub_.publish(remote_cmd_power_cmd_);
    }
    else
    {
      // do nothing
    }
  }
}
void QRosNode::onSendHighVoltageCmd(bool cmd)
{
  if (mode_cmd_.driving_mode == 1)
  {
    if (mode_cmd_.mode_flag == 0)
    {
      auto_cmd_power_cmd_.high_voltage_signal = cmd;
      auto_power_cmd_pub_.publish(auto_cmd_power_cmd_);
    }
    else if (mode_cmd_.mode_flag == 1)
    {
      remote_cmd_power_cmd_.high_voltage_signal = cmd;
      remote_power_cmd_pub_.publish(remote_cmd_power_cmd_);
    }
    else if (mode_cmd_.mode_flag == 2)
    {
      remote_cmd_power_cmd_.high_voltage_signal = cmd;
      remote_power_cmd_pub_.publish(remote_cmd_power_cmd_);
    }
    else
    {
      // do nothing
    }
  }
}

// void QRosNode::ontaskpoints(std::vector<TaskPointST> task_points_) // 报文赋值通过ros节点发送
// {
// taskPoints_msgs::taskPoints task_list_pub_;
// taskPoints_msgs::TaskNode node;
// int sizenum = task_points_.size();
// // task_list_pub_.=task_points_.;

// std::cout << " ontaskpoints " << std::endl;
// task_list_pub_.is_configured = true;
// task_list_pub_.cmd_start_plan = true;
// task_list_pub_.task_mode = 0;
// task_list_pub_.patrol_mode = 0;
// task_list_pub_.patrol_number = 0;
// for (int i = 0; i < task_points_.size(); i++)
// {
//   node.type = task_points_.at(i).type;
//   node.id = task_points_.at(i).id;
//   node.longitude = task_points_.at(i).lon;
//   node.latitude = task_points_.at(i).lat;
//   task_list_pub_.node.push_back(node);
// }
// taskPoint_pub_.publish(task_list_pub_);
//   taskPoints_msgs::taskPoints task_list_pub_;
//   taskPoints_msgs::TaskNode node;
//   int sizenum = task_points_.size();
//   // task_list_pub_.=task_points_.;

//   std::cout << " ontaskpoints " << std::endl;
//   if(tasktype == 0x00){
//   task_list_pub_.is_configured = true;
//   task_list_pub_.cmd_start_plan = false;
//   }
//   else if(tasktype == 0x01)
//   {
//   task_list_pub_.is_configured = true;
//   task_list_pub_.cmd_start_plan = true;
//   }
//   else if(tasktype == 0x02)
//   {
//   task_list_pub_.is_configured = true;
//   task_list_pub_.cmd_start_plan = false;
//   }
//   else if(tasktype == 0x03)
//   {
//   task_list_pub_.is_configured = false;
//   task_list_pub_.cmd_start_plan = false;
//   }
//   task_list_pub_.task_mode = 0;
//   task_list_pub_.patrol_mode = 0;
//   task_list_pub_.patrol_number = 0;
//   for (int i = 0; i < task_points_.size(); i++)
//   {
//     node.type = task_points_.at(i).type;
//     node.id = task_points_.at(i).id;
//     node.longitude = task_points_.at(i).lon;
//     node.latitude = task_points_.at(i).lat;
//     task_list_pub_.node.push_back(node);
//   }
//   taskPoint_pub_.publish(task_list_pub_);
// }//

void QRosNode::ontaskpoints(std::vector<TaskPointST> task_points_, uint8 tasktype) // 报文赋值通过ros节点发送
{
  taskPoints_msgs::taskPoints task_list_pub_;
  taskPoints_msgs::TaskNode node;
  int sizenum = task_points_.size();
  // task_list_pub_.=task_points_.;

  std::cout << " ontaskpoints sssssssssssssssssssssssss" << std::endl;
  if (tasktype == 0x00)
  {
    task_list_pub_.is_configured = true;
    task_list_pub_.cmd_start_plan = false;
  }
  else if (tasktype == 0x01)
  {
    task_list_pub_.is_configured = true;
    task_list_pub_.cmd_start_plan = true;
  }
  else if (tasktype == 0x02)
  {
    task_list_pub_.is_configured = true;
    task_list_pub_.cmd_start_plan = false;
  }
  else if (tasktype == 0x03)
  {
    task_list_pub_.is_configured = false;
    task_list_pub_.cmd_start_plan = false;
  }
  task_list_pub_.task_mode = 0;
  task_list_pub_.patrol_mode = 0;
  task_list_pub_.patrol_number = 0;

  std::string pkg_dir = ros::package::getPath("launch_node") + "/data/";
  std::string task_points_file = pkg_dir + config_["task_points_file"].as<std::string>();
  std::ifstream filename(task_points_file);

  f_gps_data_out = std::ofstream(task_points_file);
  if (!f_gps_data_out)
  {
    std::cout << " file open error: " << task_points_file << std::endl;
    return;
  }
  for (int i = 0; i < task_points_.size(); i++)
  {
    node.type = task_points_.at(i).type;
    node.id = task_points_.at(i).id;
    node.longitude = task_points_.at(i).lon;
    node.latitude = task_points_.at(i).lat;
    task_list_pub_.node.push_back(node);
    f_gps_data_out << node.id << ";" << std::setprecision(7) << std::fixed << node.longitude << ";" << node.latitude << ";"
                   << "0.0"
                   << ";" << static_cast<int>(node.type) << std::endl;
  }
  f_gps_data_out.flush();
  f_gps_data_out.close();
  taskPoint_pub_.publish(task_list_pub_);
}

void QRosNode::send(const ros::TimerEvent &e)
{
  //  std::cout << "ssssss"<< std::endl;
  heartbeat_msgs::Heartbeat msgs;
  static uint8 heartbeat_cout = 0;
  msgs.heart_beat = heartbeat_cout;
  msgs.flag = 101;
  heartbeat_pub_.publish(msgs);
  if (heartbeat_cout != 255)
  {
    heartbeat_cout++;
  }
  else
  {
    heartbeat_cout = 0;
  }
}
void QRosNode::onSendBrakeLightCmd(bool cmd)
{
  light_horn_wiper_cmd_.brake_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}

void QRosNode::onSendFrontFoggyLightCmd(bool cmd)
{
  light_horn_wiper_cmd_.front_foggy_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}
void QRosNode::onSendRearFoggyLightCmd(bool cmd)
{
  light_horn_wiper_cmd_.rear_foggy_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}
void QRosNode::onSendPositionLightCmd(bool cmd)
{
  light_horn_wiper_cmd_.position_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}
void QRosNode::onSendReverseLightCmd(bool cmd)
{
  light_horn_wiper_cmd_.reverse_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}
void QRosNode::onSendWiperCmd(bool cmd)
{
  light_horn_wiper_cmd_.wiper = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}
void QRosNode::onsendHeadLightCmd(bool cmd)
{
  light_horn_wiper_cmd_.head_light = cmd;
  light_horn_wiper_cmd_pub_.publish(light_horn_wiper_cmd_);
}

//  platoon
//  enum PlatoonType {
//   NONE =0, //
//   BUILD, //构建
//   JOIN,  //入队
//   LEAVE, //出队
//   DISSOLVE, //解散
//   COLUMN,   //纵队
//   DIAMOND,   //菱形
//   RUNNING
void QRosNode::platoonmembersCallback(const platoon_msgs::PlatoonMember::ConstPtr &msg)
{
    emit emitplatoonmembers(msg);
}

void QRosNode::platoonmembersselfCallback(const platoon_msgs::PlatoonMember::ConstPtr &msg)
{
    emit emitplatoonmembers(msg);
}

void QRosNode::platoonmissionCallback(const platoon_msgs::PlatoonMission::ConstPtr &msg)
{
    emit emitplatoonmission(msg);
}

void QRosNode::platoonmissionselfCallback(const platoon_msgs::PlatoonMission::ConstPtr &msg)
{
    emit emitplatoonmission(msg);
}

void QRosNode::OnSendPlatoonBuildCmd()
{
  platoonOpt_cmd_.cmd = PlatoonType::BUILD;
  platoon_cmd_pub_.publish(platoonOpt_cmd_);
}

void QRosNode::OnSendPlatoonDissloveCmd()
{
  platoonOpt_cmd_.cmd = PlatoonType::DISSOLVE;
  platoon_cmd_pub_.publish(platoonOpt_cmd_);
}
void QRosNode::OnSendPlatoonJoinCmd(int carnum)
{
  platoonOpt_cmd_.num = carnum;
  platoonOpt_cmd_.cmd = PlatoonType::JOIN;
  platoon_cmd_pub_.publish(platoonOpt_cmd_);
}
void QRosNode::OnSendPlatoonLeaveCmd(int carnum)
{
  platoonOpt_cmd_.num = carnum;
  platoonOpt_cmd_.cmd = PlatoonType::LEAVE;
  platoon_cmd_pub_.publish(platoonOpt_cmd_);
}
void QRosNode::OnSendPlatoonColumnCmd()
{
  platoonOpt_cmd_.cmd = PlatoonType::COLUMN;
  platoon_cmd_pub_.publish(platoonOpt_cmd_);
}
void QRosNode::OnSendPlatoonDiamondCmd()
{
  platoonOpt_cmd_.cmd = PlatoonType::DIAMOND;
  platoon_cmd_pub_.publish(platoonOpt_cmd_);
}

//轨迹录制
void QRosNode::OnSendRecordCmd(int recordcmd)
{
    std_msgs::Int32 recordmsg;
    recordmsg.data = recordcmd;
    record_cmd_pub_.publish(recordmsg);
}

//倒车信号
void QRosNode::OnSendBackWordCmd(int backwordcmd)
{
    std_msgs::Int32 backwordcmd_;
    backwordcmd_.data = backwordcmd;
    backword_cmd_pub_.publish(backwordcmd_);
}
//void QRosNode::OnSendPlatoonMotionStartCmd()
//{
//  platoon_cmd_.data = PlatoonType::RUNNING;
//  platoon_cmd_pub_.publish(platoon_cmd_);
//}
//void QRosNode::OnSendPlatoonMotionStopCmd()
//{
//  todo
//}
