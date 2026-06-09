cmake_minimum_required(VERSION 3.0.2)
project(data_compare)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  roslint
  roscpp
  std_msgs
  visualization_msgs
  amathutils_lib
  display
  smoother
  utm
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

add_executable(${PROJECT_NAME}_node
  src/data_compare.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  # ${EIGEN3_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
