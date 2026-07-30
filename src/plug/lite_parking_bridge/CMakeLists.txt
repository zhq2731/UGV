cmake_minimum_required(VERSION 3.0.2)
project(lite_parking_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  driver_msgs
  geometry_msgs
  localization_msgs
  nav_msgs
  planning_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(lite_parking_bridge_node src/lite_parking_bridge_node.cpp)
target_link_libraries(lite_parking_bridge_node ${catkin_LIBRARIES})
add_dependencies(lite_parking_bridge_node ${catkin_EXPORTED_TARGETS})

install(TARGETS lite_parking_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
