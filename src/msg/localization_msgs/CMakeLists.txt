cmake_minimum_required(VERSION 2.8.3)
project(localization_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  ins_msgs
  nav_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    Localization.msg
)
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    ins_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
    ins_msgs
    nav_msgs
)
