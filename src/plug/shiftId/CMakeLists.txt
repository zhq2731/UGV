cmake_minimum_required(VERSION 3.0.2)
project(shiftID)

#add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
find_package(pugixml)

include_directories(
  include
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/OsmFile.cpp
)

target_link_libraries(${PROJECT_NAME}
  -lpugixml
)
