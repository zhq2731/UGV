cmake_minimum_required(VERSION 2.8.11)
project(open_space_planner) # autoware math utility
set(CMAKE_CXX_STANDARD 17)

# 可选优化及正常规划接线，默认不增加 IPOPT/ADOL-C 依赖；运行开关另由 YAML 控制。
option(OPEN_SPACE_BUILD_LIOM "Build optional LIOM optimizer and checks" OFF)
set(OPEN_SPACE_LIBRARIES ${PROJECT_NAME})
if(OPEN_SPACE_BUILD_LIOM)
  find_path(LIOM_IPOPT_INCLUDE_DIR IpIpoptApplication.hpp PATH_SUFFIXES coin coin-or)
  find_library(LIOM_IPOPT_LIBRARY ipopt)
  find_path(LIOM_ADOLC_INCLUDE_DIR adolc/adolc.h)
  find_library(LIOM_ADOLC_LIBRARY adolc)
  if(NOT LIOM_IPOPT_INCLUDE_DIR OR NOT LIOM_IPOPT_LIBRARY OR
     NOT LIOM_ADOLC_INCLUDE_DIR OR NOT LIOM_ADOLC_LIBRARY)
    message(FATAL_ERROR "LIOM requires IPOPT and ADOL-C development libraries")
  endif()
  list(APPEND OPEN_SPACE_LIBRARIES open_space_liom)
endif()

find_package(Eigen3 REQUIRED)
# 同一 CMake 包提供头文件和库，避免 /usr/local 头文件与系统动态库版本混用。
find_package(yaml-cpp REQUIRED)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  roslib
  std_msgs
  geometry_msgs
  nav_msgs
  amathutils_lib
  common
  planning_msgs
  pid
  vehicle_info_util
  planner_common
  visualization_msgs
)


set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${OPEN_SPACE_LIBRARIES}
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

file(GLOB_RECURSE cpp_files src/*.cpp src/*.cc )
list(REMOVE_ITEM cpp_files ${CMAKE_CURRENT_SOURCE_DIR}/src/liom_optimizer.cpp)
list(REMOVE_ITEM cpp_files ${CMAKE_CURRENT_SOURCE_DIR}/src/liom_grid.cpp)

add_library(${PROJECT_NAME} ${cpp_files})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

if(OPEN_SPACE_BUILD_LIOM)
  add_library(open_space_liom src/liom_optimizer.cpp src/liom_grid.cpp)
  target_include_directories(open_space_liom PRIVATE ${LIOM_IPOPT_INCLUDE_DIR} ${LIOM_ADOLC_INCLUDE_DIR})
  target_compile_definitions(open_space_liom PRIVATE HAVE_CSTDDEF)
  target_link_libraries(open_space_liom ${LIOM_IPOPT_LIBRARY} ${LIOM_ADOLC_LIBRARY} ${catkin_LIBRARIES})
  add_dependencies(open_space_liom ${catkin_EXPORTED_TARGETS})
  target_compile_definitions(${PROJECT_NAME} PRIVATE OPEN_SPACE_WITH_LIOM)
  target_link_libraries(${PROJECT_NAME} open_space_liom)
  # 此检查需独立 ROS master，手动运行；不让普通 CTest 隐式连接现有车辆系统。
  add_executable(liom_planner_check test/liom_planner_check.cpp)
  target_link_libraries(liom_planner_check ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
  add_dependencies(liom_planner_check ${catkin_EXPORTED_TARGETS})
  add_executable(liom_segment_check test/liom_segment_check.cpp)
  target_link_libraries(liom_segment_check open_space_liom ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
  add_dependencies(liom_segment_check ${catkin_EXPORTED_TARGETS})
  add_executable(liom_visualization_demo test/liom_visualization_demo.cpp)
  target_link_libraries(liom_visualization_demo open_space_liom ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
  add_dependencies(liom_visualization_demo ${catkin_EXPORTED_TARGETS})
  add_executable(liom_grid_check test/liom_grid_check.cpp)
  target_link_libraries(liom_grid_check open_space_liom ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
  add_dependencies(liom_grid_check ${catkin_EXPORTED_TARGETS})
  add_executable(liom_search_check test/liom_search_check.cpp)
  target_link_libraries(liom_search_check open_space_liom ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
  add_dependencies(liom_search_check ${catkin_EXPORTED_TARGETS})
  if(CATKIN_ENABLE_TESTING)
    add_test(NAME liom_segment_check COMMAND liom_segment_check
      ${CMAKE_CURRENT_SOURCE_DIR}/param/openspace.yaml)
    add_test(NAME liom_grid_check COMMAND liom_grid_check
      ${CMAKE_CURRENT_SOURCE_DIR}/param/openspace.yaml)
    add_test(NAME liom_search_check COMMAND liom_search_check
      ${CMAKE_CURRENT_SOURCE_DIR}/param/openspace.yaml)
    set_tests_properties(liom_search_check PROPERTIES TIMEOUT 60)
  endif()
  install(TARGETS open_space_liom
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES LICENSE.LIOM LIOM_STAGE2.md LIOM_STAGE3.md LIOM_STAGE4.md LIOM_STAGE5.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

file(GLOB_RECURSE ROSLINT_FILES
  LIST_DIRECTORIES false
  *.cpp *.h *.hpp
)

list(APPEND ROSLINT_CPP_OPTS "--extensions=cc,h,hpp,cpp,cu,cuh" "--filter=-build/c++11,-runtime/references")
roslint_cpp(${ROSLINT_FILES})
