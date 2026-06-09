cmake_minimum_required(VERSION 3.8)
project(third_party_controller LANGUAGES CXX C)

add_definitions(-D__ROS__)

find_package(PkgConfig REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roslib
    rospy
    roscpp
    std_msgs
    driver_msgs
    planning_msgs
    localization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS 
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include 
                    ${EIGEN3_INCLUDE_DIR}
                    ${YAMLCPP_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/src/*.cc)                  

add_library(${PROJECT_NAME} SHARED ${SRCS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})



