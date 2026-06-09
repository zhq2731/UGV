cmake_minimum_required(VERSION 2.8.3)
project(perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    PerceptionObstacle.msg
    PerceptionObstacles.msg
    PredictionObstacle.msg
    PredictionObstacles.msg
    TrajectoryPoint.msg
)
generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
)
