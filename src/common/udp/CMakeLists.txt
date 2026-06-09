cmake_minimum_required(VERSION 3.5)
project(udp)

set(PACKAGE_DEPENDENCIES
  roscpp
)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
  #
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include/)

add_library(${PROJECT_NAME} 
src/udpServer.cpp  src/udp_multicast.cpp)


add_executable(multicast_example src/test_udp.cpp)
target_link_libraries(multicast_example ${PROJECT_NAME} pthread)

# install lib
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install source headers.
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )



