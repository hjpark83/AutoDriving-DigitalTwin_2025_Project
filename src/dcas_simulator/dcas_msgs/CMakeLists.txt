cmake_minimum_required(VERSION 3.0.2)
project(dcas_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Object.msg
  ObjectArray.msg
  Lane.msg
  LaneArray.msg
  TrafficLight.msg
  TrafficLightArray.msg
  TrafficSign.msg
  TrafficSignArray.msg
  VehicleState.msg
  RadarDetection.msg
  RadarDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)


