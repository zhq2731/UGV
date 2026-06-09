cmake_minimum_required(VERSION 2.8.11)
project(utm) 
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
)

find_package(GeographicLib)


message(STATUS "=== GeographicLib_INCLUDE_DIRS: ${GeographicLib_INCLUDE_DIRS}")
message(STATUS "=== GeographicLib_LIBRARIES: ${GeographicLib_LIBRARIES}")

include_directories(${GeographicLib_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utm
  CATKIN_DEPENDS
)


include_directories(
  include/utm/
  ${catkin_INCLUDE_DIRS}
)

add_library(utm src/UTM.cpp
)

target_link_libraries(utm
  ${catkin_LIBRARIES} 
  ${GeographicLib_LIBRARIES}
)

add_dependencies(utm ${catkin_EXPORTED_TARGETS}
)

install(TARGETS utm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/utm/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
