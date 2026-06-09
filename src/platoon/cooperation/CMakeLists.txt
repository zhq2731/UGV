cmake_minimum_required(VERSION 3.0.2)
project(cooperation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  roslint
  roscpp
  std_msgs
  visualization_msgs
  driver_msgs
  ins_msgs
  planning_msgs
  localization_msgs
  amathutils_lib
  display
  vehicle_info_util
  smoother
  lanelet_map_msgs
  utm
  heartbeat_msgs
  platoon_msgs
  platoon_common
  vehicle_info_util
  driver_msgs
  common
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${reference_line_smoother_INCLUDE_DIRS}
)

add_executable(cooperation
  src/cooperation.cpp
)

target_link_libraries(cooperation
  ${catkin_LIBRARIES}
  # ${EIGEN3_LIBRARIES}
)

install(
  TARGETS cooperation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
