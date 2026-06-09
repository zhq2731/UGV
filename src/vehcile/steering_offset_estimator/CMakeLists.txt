cmake_minimum_required(VERSION 3.0.2)
project(steering_offset_estimator)

add_compile_options(-std=c++11)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  roslint
  roscpp
  std_msgs
  driver_msgs
  localization_msgs
  amathutils_lib
  vehicle_info_util
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

add_executable(${PROJECT_NAME}
  src/SteeringOffsetEstimator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp	
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)