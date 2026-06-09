cmake_minimum_required(VERSION 3.0.2)
project(simulate)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  std_msgs
  visualization_msgs
  autoware_msgs
  driver_msgs
  localization_msgs
  ins_msgs
  planning_msgs
  amathutils_lib
  vehicle_info_util
  utm
  ray_msgs
  platoon_common
  common
  route_msgs
)

#find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  #${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulate
  src/simulate.cpp
)

target_link_libraries(simulate
  ${catkin_LIBRARIES}
  # ${EIGEN3_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS simulate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
