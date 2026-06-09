cmake_minimum_required(VERSION 3.5)
project(casic_socketcan)

set(PACKAGE_DEPENDENCIES
  roscpp
)
#开fpic 20240729 yz
#set(cmake_cxx_flags "${cmake_cxx_flags} -fpic")
add_compile_options(-fpic)
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

add_library(${PROJECT_NAME} STATIC 
src/socket_can_common.cpp
src/socket_can_id.cpp
src/socket_can_receiver.cpp
src/socket_can_sender.cpp)
target_include_directories(${PROJECT_NAME}
PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include)

# add_executable(${PROJECT_NAME} )

# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install lib
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install source headers.
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
 )



