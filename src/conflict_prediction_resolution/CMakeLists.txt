cmake_minimum_required(VERSION 3.0.2)
project(conflict_prediction_resolution)

add_compile_options(-std=c++14 -O2 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  driver_msgs
  geometry_msgs
  localization_msgs
  planning_msgs
  planner_common
  roscpp
  std_msgs
  tf
  velocity_planner
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
get_target_property(osqp_INCLUDE_DIR osqp::osqpstatic INTERFACE_INCLUDE_DIRECTORIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    conflict_prediction_resolution_core
    conflict_constraint_processor
  CATKIN_DEPENDS
    driver_msgs
    geometry_msgs
    localization_msgs
    planning_msgs
    planner_common
    roscpp
    std_msgs
    tf
    velocity_planner
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${osqp_INCLUDE_DIR}
)

add_library(conflict_prediction_resolution_core
  src/common.cpp
  src/coordination_core.cpp
)
target_link_libraries(conflict_prediction_resolution_core ${catkin_LIBRARIES})
add_dependencies(conflict_prediction_resolution_core ${catkin_EXPORTED_TARGETS})

add_library(conflict_constraint_processor
  src/conflict_constraint_processor.cpp
  src/conflict_velocity_optimizer.cpp
)
target_link_libraries(conflict_constraint_processor ${catkin_LIBRARIES} osqp::osqp)
add_dependencies(conflict_constraint_processor ${catkin_EXPORTED_TARGETS})

add_executable(conflict_resolver_node src/conflict_resolver_node.cpp)
target_link_libraries(conflict_resolver_node conflict_prediction_resolution_core ${catkin_LIBRARIES})
add_dependencies(conflict_resolver_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  conflict_prediction_resolution_core
  conflict_constraint_processor
  conflict_resolver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
