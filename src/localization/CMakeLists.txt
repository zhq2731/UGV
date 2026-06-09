cmake_minimum_required(VERSION 3.10)
project(localizer)
set(CMAKE_CXX_STANDARD 17)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
endif ()

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if(N GREATER 4)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=3)
        message("core for MP: 3")
    elseif(N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
        message("core for MP: 2")
    else()
        add_definitions(-DMP_PROC_NUM=1)
    endif()
else()
    add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  std_msgs
  tf
  localization_msgs
)


include_directories(${catkin_INCLUDE_DIRS} )
include_directories(${EIGEN_ROOT})
# add_message_files(
#     FILES
#     ConfigurationDown.msg
#     Ins.msg
#     Localization.msg
#     Replan.msg
#     TrajectoryPoint.msg
#     TrajectoryPointArray.msg
#     PowerCmd.msg
# )
  
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    nav_msgs
#    std_msgs
# )
  
include_directories(
  include
	${catkin_INCLUDE_DIRS}

)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        pcl_conversions
        pcl_ros)
        # message_generation)

add_library(${PROJECT_NAME}
    src/run_localization.cpp
    src/Localization.cpp
    src/Localization.hpp
#    src/visualpublish.cpp
#    src/visualpublish.h
)

add_executable(run_localization_node src/run_localization.cpp )
target_link_libraries(run_localization_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
