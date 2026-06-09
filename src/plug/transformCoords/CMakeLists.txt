cmake_minimum_required(VERSION 3.0.2)
project(transformCoords)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  ins_msgs
  roslib
  localization_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  #${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/transformCoords.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}  
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
