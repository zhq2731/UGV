#!/usr/bin/env bash

UGV_WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
UGV_PUGIXML_PREFIX="${UGV_WORKSPACE_DIR}/.local-deps/pugixml/usr"

if [[ -f "${UGV_PUGIXML_PREFIX}/lib/x86_64-linux-gnu/cmake/pugixml/pugixml-config.cmake" ]]; then
  export CMAKE_PREFIX_PATH="${UGV_PUGIXML_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${UGV_PUGIXML_PREFIX}/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

catkin_make --pkg autoware_msgs driver_msgs ins_msgs localization_msgs perception_msgs planning_msgs lanelet_map_msgs plan2control_msgs ray_msgs sensor_driver_msgs taskPoints_msgs heartbeat_msgs platoon_msgs &&
  catkin_make &&
  catkin_make install



 

 
