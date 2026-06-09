cmake_minimum_required(VERSION 3.0.2)
project(route)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  roslint
  roscpp
  std_msgs
  driver_msgs
  localization_msgs
  amathutils_lib
  utm 
  display
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pugixml REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/main.cpp  
  src/mapLoader.cpp
  src/osmFileParser.cpp
  src/route.cpp  
  src/topoGraph.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp	
  pugixml
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)