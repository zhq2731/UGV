cmake_minimum_required(VERSION 2.8.11)
project(display) # autoware math utility

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  roslint
  tf
  tf2
  visualization_msgs
  amathutils_lib
  vehicle_info_util
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES display
  CATKIN_DEPENDS
  autoware_msgs
  roscpp
  roslint
  tf
  tf2
  visualization_msgs
  amathutils_lib
  vehicle_info_util
)

include_directories(
  include/display
  ${catkin_INCLUDE_DIRS}
)

add_library(display
  src/display.cpp
)

target_link_libraries(display
  ${catkin_LIBRARIES}
)

add_dependencies(display
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS display
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