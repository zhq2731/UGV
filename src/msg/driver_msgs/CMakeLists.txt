cmake_minimum_required(VERSION 2.8.3)
project(driver_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    ChassisReport.msg
    ChassisCmd.msg       
    GearCmd.msg            
    PowerCmd.msg
    LightHornWiperCmd.msg
    SteeringWheelCmd.msg
    ChassisWheelCmd.msg
    ModeCmd.msg
    DriveCmd.msg 
    ParkingBrakeCmd.msg
    HeartBeat.msg
    ResetCmd.msg
    NonAutoModeReport.msg
    MotionStartCmd.msg
    ElectromagneticSilence.msg
    LightHornWiperReport.msg
    EstopCmd.msg
    ProcessTipCmd.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
