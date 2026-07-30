cmake_minimum_required(VERSION 3.0.2)
project(trajectory_follower)
set(CMAKE_CXX_STANDARD 17)

catkin_package(
  CATKIN_DEPENDS
)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  autoware_msgs
  driver_msgs
  ins_msgs
  planning_msgs
  localization_msgs
  vehicle_info_util
  roslib
  amathutils_lib
  third_party_controller
  display
  visualization_msgs
  heartbeat_msgs
  yaml-cpp
  platoon_msgs
  common
  platoon_common
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

#链接第三方库 lib
#link_directories(
 # /home/lb/eleven/autoDrive/src/trajectory_follower/lib
#)

#链接第三方库 lib
link_directories(
  include
  ${PROJECT_SOURCE_DIR}/lib
)

add_executable(trajectory_follower
  src/controller_node.cpp
  src/open_space_longitudinal_controller.cpp
  src/open_space_mpc_lateral_controller.cpp
  src/mpc_lateral_controller.cpp
  src/interpolate.cpp
  src/lowpass_filter.cpp
  src/mpc.cpp
  src/mpc_trajectory.cpp
  src/mpc_utils.cpp
  src/qp_solver/qp_solver_osqp.cpp
  src/qp_solver/qp_solver_unconstr_fast.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_interface.cpp
  src/osqp_interface.cpp
  src/csc_matrix_conv.cpp
  src/linear_interpolation.cpp
  src/spline_interpolation.cpp
  src/spline_interpolation_points_2d.cpp
  src/spherical_linear_interpolation.cpp
)

target_link_libraries(trajectory_follower
  ${catkin_LIBRARIES} osqp
  # ${EIGEN3_LIBRARIES}
  yaml-cpp
)

add_dependencies(trajectory_follower ${catkin_EXPORTED_TARGETS} )


install(
  TARGETS trajectory_follower 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
