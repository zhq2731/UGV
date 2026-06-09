cmake_minimum_required(VERSION 3.0.2)
project(structured_road_conflict_sim)

add_compile_options(-std=c++14 -O2 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  driver_msgs
  geometry_msgs
  localization_msgs
  nav_msgs
  planning_msgs
  roscpp
  std_msgs
  tf
  tf2_ros
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    driver_msgs
    geometry_msgs
    localization_msgs
    nav_msgs
    planning_msgs
    roscpp
    std_msgs
    tf
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(structured_road_conflict_sim_core
  src/common.cpp
  src/coordination_core.cpp
  src/road_network.cpp
  src/trajectory_utils.cpp
  src/visualization_builder.cpp
)
target_link_libraries(structured_road_conflict_sim_core ${catkin_LIBRARIES})
add_dependencies(structured_road_conflict_sim_core ${catkin_EXPORTED_TARGETS})

add_executable(conflict_resolver_node src/conflict_resolver_node.cpp)
target_link_libraries(conflict_resolver_node structured_road_conflict_sim_core ${catkin_LIBRARIES})
add_dependencies(conflict_resolver_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  structured_road_conflict_sim_core
  conflict_resolver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
