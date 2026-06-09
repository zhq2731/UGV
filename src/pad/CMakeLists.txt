#################################
cmake_minimum_required(VERSION 3.2)
project(pad)

add_definitions(-std=c++11 -fPIC -Wall -Wextra)

##########
# CATKIN #
##########
find_package(catkin REQUIRED COMPONENTS
  roscpp
  driver_msgs
  ins_msgs
  localization_msgs
  perception_msgs
  taskPoints_msgs
  platoon_msgs
  roslib
  rospy
  platoon_common
)



catkin_package()

find_package(yaml-cpp)

##################
# QT ENVIRONMENT #
##################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# non-ROS:
# find_package(Qt5 COMPONENTS Core Gui Widgets)



find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED) #大写
# find_package(PCL 1.8 REQUIRED COMPONENTS common io sample_consensus) 
# find_package(autoware_auto_msgs REQUIRED)
# Note that ament_target_dependencies() in algo cannot export Boost::thread to downstream. As a result, qt_example_node, as a downstream of libalgo.so, needs to explicitly find_package() for Boost again so that qt_example_node is able to link to Boost::thread.
#find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
file(GLOB PRO_FORM_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB PRO_RESOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource/*.qrc)
file(GLOB_RECURSE PRO_INCLUDE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h )
file(GLOB_RECURSE PRO_SOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
#搜索头文件的位置
include_directories(
        include/
/usr/include
        ${catkin_INCLUDE_DIRS}
       ${CMAKE_CURRENT_BINARY_DIR}
       ${CMAKE_CURRENT_SOURCE_DIR}
      #${PCL_INCLUDE_DIRS}
)
#qt头文件moc文件resource文件处理
qt5_wrap_ui(QT_UI_HPP ${PRO_FORM_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOURCES_CPP ${PRO_RESOURCES_DIR})

add_executable(${PROJECT_NAME}_node
        ${PRO_SOURCES_DIR}
        ${QT_RESOURCES_CPP}
        ${PRO_INCLUDE_DIR}
        ${QT_MOC_HPP}
        ${PRO_FORM_DIR}
)




target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    Qt5::Network
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    yaml-cpp
  #  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############
# SECTIONS #
############

# message("==============" ${CMAKE_CURRENT_SOURCE_DIR})
# file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
# #file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

# message("==============" ${QT_FORMS})
# #QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})


# catkin_package(
#   CATKIN_DEPENDS
# )

# include_directories(
#     ./
#     ${Qt5Core_INCLUDE_DIRS}
#     ${Qt5Gui_INCLUDE_DIRS}
#     ${Qt5Widgets_INCLUDE_DIRS}
#     ${catkin_INCLUDE_DIRS}
#     )


# ###########
# # SOURCES #
# ###########

# set(QT_SOURCES
#     main.cpp
#     mainwindow.cpp
#     qnode.cpp
#     )

# #############
# # LIBRARIES #
# #############

# #cs_add_library(${PROJECT_NAME}
# #
# #        )

# ############
# # BINARIES #
# ############
# add_executable(auto_scoring_node ${QT_SOURCES}  ${QT_FORMS_HPP} ${QT_MOC_HPP} )


##########
# EXPORT #
##########
# cs_install()
# cs_export()

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )



