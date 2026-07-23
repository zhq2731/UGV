cmake_minimum_required(VERSION 3.0.2)

project(planner)

set(CMAKE_CXX_STANDARD 17)

catkin_package(
  CATKIN_DEPENDS
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  visualization_msgs
  autoware_msgs
  driver_msgs
  ins_msgs
  localization_msgs
  planning_msgs
  perception_msgs
  amathutils_lib
  display
  vehicle_info_util
  velocity_planner
  plan2control_msgs
  heartbeat_msgs
  utm
  lanelet_map_msgs
  platoon_msgs
  pid
  platoon_common
  common
  conflict_prediction_resolution
  planner_common
  route_msgs
  open_space_planner
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  include
  ${PROJECT_SOURCE_DIR}/lib
)

file(GLOB_RECURSE cpp_files src/*.cpp src/*.cc )

add_executable(planner ${cpp_files})

target_link_libraries(planner ${catkin_LIBRARIES} osqp)

add_dependencies(planner ${catkin_EXPORTED_TARGETS} )

install(
  TARGETS planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
