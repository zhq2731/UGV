cmake_minimum_required(VERSION 2.8.11)
project(smoother) # autoware math utility
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smoother
  CATKIN_DEPENDS
)

include_directories(
  include/smoother
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(smoother
  src/fem_pos_deviation_osqp_interface.cc
  src/fem_pos_deviation_smoother.cc
  src/fem_pos_deviation_sqp_osqp_interface.cc
)
target_link_libraries(smoother
  ${catkin_LIBRARIES} osqp
)

add_dependencies(smoother
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS smoother
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