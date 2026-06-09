cmake_minimum_required(VERSION 3.0.2)
project(reference_line_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  roslint
  roscpp
  std_msgs
  visualization_msgs
  autoware_msgs
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
  common
  logger
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

add_executable(reference_line_node
  src/reference_node.cpp src/formatting_reference.cpp
)

target_link_libraries(reference_line_node
  ${catkin_LIBRARIES}
  # ${EIGEN3_LIBRARIES}
)

install(
  TARGETS reference_line_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
