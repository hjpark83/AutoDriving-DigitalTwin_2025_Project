cmake_minimum_required(VERSION 3.0.2)
project(dcas_lanelet_tutorial)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/lanelet_routing_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


