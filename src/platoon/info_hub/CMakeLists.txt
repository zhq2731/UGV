cmake_minimum_required(VERSION 3.0.2)
project(info_hub)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  roslint
  roscpp
  std_msgs
  platoon_msgs
  platoon_common
  amathutils_lib
  udp
  common
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(/usr/lib/x86_64-linux-gnu /usr/local/lib)

add_executable(info_hub
  src/info_hub.cpp src/main.cpp
)

target_link_libraries(info_hub
  ${catkin_LIBRARIES}
)

install(
  TARGETS info_hub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
