cmake_minimum_required(VERSION 3.0.2)
project(launch_node)

add_compile_options(-std=c++11)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  driver_msgs
  pluginlib
  roscpp
  rospy
  rviz
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  CATKIN_DEPENDS
    driver_msgs
    pluginlib
    roscpp
    rospy
    rviz
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(launch_node_rviz_plugins
  src/two_vehicle_motion_panel.cpp
  include/launch_node/two_vehicle_motion_panel.h
)
add_dependencies(launch_node_rviz_plugins ${catkin_EXPORTED_TARGETS})
target_link_libraries(launch_node_rviz_plugins
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/marker_color_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  launch_node_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
