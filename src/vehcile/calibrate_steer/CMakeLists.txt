cmake_minimum_required(VERSION 3.0.2)
project(calibrate_steer)

add_compile_options(-std=c++11)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
)
find_package(yaml-cpp)
find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(calibrate_steer
  src/calibrateSteer.cpp
)

target_link_libraries(calibrate_steer
  ${catkin_LIBRARIES}
)

add_dependencies(calibrate_steer ${catkin_EXPORTED_TARGETS})

install(
  TARGETS calibrate_steer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


