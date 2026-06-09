cmake_minimum_required(VERSION 2.8.11)
project(velocity_planner) 
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  roslib
  driver_msgs
  perception_msgs
  planning_msgs
  amathutils_lib
  vehicle_info_util
  platoon_common
  platoon_msgs
  common
  planner_common
)
set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES velocity_planner
  CATKIN_DEPENDS
)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(yaml-cpp REQUIRED)

get_target_property(osqp_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${osqp_INCLUDE_DIR}
)


add_library(velocity_planner 
src/interpolate.cpp 
src/velocity_planner.cpp
src/optimizer.cpp
src/solver/qp_solver_osqp.cpp
src/solver/osqp_interface/csc_matrix_conv.cpp
src/solver/osqp_interface/osqp_interface.cpp
src/velocity_planner_flow.cpp
)


link_directories(
   include
  ${catkin_LIB_DIRS}       
)

add_dependencies(velocity_planner
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS velocity_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB_RECURSE ROSLINT_FILES
  LIST_DIRECTORIES false
  *.cpp *.h *.hpp
)

list(APPEND ROSLINT_CPP_OPTS "--extensions=cc,h,hpp,cpp,cu,cuh" "--filter=-build/c++11,-runtime/references")
roslint_cpp(${ROSLINT_FILES})

target_link_libraries(
    velocity_planner
    ${catkin_LIBRARIES}
	yaml-cpp
)

add_executable(velocity_planner_node app/run_velocity_planner.cpp)
target_link_libraries(velocity_planner_node 
velocity_planner
yaml-cpp)

target_link_libraries(velocity_planner_node PRIVATE osqp::osqp )
