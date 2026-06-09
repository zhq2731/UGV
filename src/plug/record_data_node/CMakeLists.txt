cmake_minimum_required(VERSION 3.0.2)
project(record_data_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  visualization_msgs
  ins_msgs
  roslib
  localization_msgs
  utm
  display
  driver_msgs
  vehicle_info_util
)
find_package(yaml-cpp)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  #${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(record_data_node
  src/RecordDataNode.cpp
)

target_link_libraries(record_data_node
  ${catkin_LIBRARIES} 
  utm 
  yaml-cpp
  # ${EIGEN3_LIBRARIES}  
)


install(TARGETS record_data_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
