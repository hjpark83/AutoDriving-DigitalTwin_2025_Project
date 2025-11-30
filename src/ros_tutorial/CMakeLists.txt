cmake_minimum_required(VERSION 3.0.2)
project(ros_tutorial)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  visualization_msgs
)

add_message_files(
  FILES
  TutorialSineWaveInput.msg
  TutorialVehicleState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/sine_wave_set_node.py
  scripts/steering_sine_wave_generator_node.py
  scripts/kinematic_bicycle_model_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
