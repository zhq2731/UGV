cmake_minimum_required(VERSION 3.0.2)
project(task_scheduler)

add_compile_options(-std=c++17 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  route_msgs
  std_msgs
  tf
  utm
  visualization_msgs
)

find_package(pugixml REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp roslib route_msgs std_msgs tf utm visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(task_scheduler_node
  src/task_scheduler_node.cpp
)

add_dependencies(task_scheduler_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(task_scheduler_node
  ${catkin_LIBRARIES}
  pugixml
)

install(TARGETS task_scheduler_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
