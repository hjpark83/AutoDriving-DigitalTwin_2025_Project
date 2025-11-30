# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dcas_msgs: 11 messages, 0 services")

set(MSG_I_FLAGS "-Idcas_msgs:/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dcas_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg" "geometry_msgs/Vector3:geometry_msgs/Point:geometry_msgs/Twist:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point32:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg" "geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Twist:dcas_msgs/Object:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point32:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg" "geometry_msgs/Point32:std_msgs/Header"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg" "dcas_msgs/Lane:std_msgs/Header:geometry_msgs/Point32"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:std_msgs/Header:dcas_msgs/TrafficLight:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg" "geometry_msgs/Point:dcas_msgs/TrafficSign:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg" "geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Twist:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg" "geometry_msgs/Vector3:geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg" NAME_WE)
add_custom_target(_dcas_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dcas_msgs" "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg" "geometry_msgs/Vector3:geometry_msgs/Point:dcas_msgs/RadarDetection:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_cpp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(dcas_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dcas_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dcas_msgs_generate_messages dcas_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_cpp _dcas_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dcas_msgs_gencpp)
add_dependencies(dcas_msgs_gencpp dcas_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dcas_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)
_generate_msg_eus(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(dcas_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dcas_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dcas_msgs_generate_messages dcas_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_eus _dcas_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dcas_msgs_geneus)
add_dependencies(dcas_msgs_geneus dcas_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dcas_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)
_generate_msg_lisp(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(dcas_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dcas_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dcas_msgs_generate_messages dcas_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_lisp _dcas_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dcas_msgs_genlisp)
add_dependencies(dcas_msgs_genlisp dcas_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dcas_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)
_generate_msg_nodejs(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(dcas_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dcas_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dcas_msgs_generate_messages dcas_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_nodejs _dcas_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dcas_msgs_gennodejs)
add_dependencies(dcas_msgs_gennodejs dcas_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dcas_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)
_generate_msg_py(dcas_msgs
  "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(dcas_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dcas_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dcas_msgs_generate_messages dcas_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Object.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/ObjectArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLight.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficLightArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSign.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/TrafficSignArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetection.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hjpark/automotive/digital_twin_sw/src/dcas_simulator/dcas_msgs/msg/RadarDetectionArray.msg" NAME_WE)
add_dependencies(dcas_msgs_generate_messages_py _dcas_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dcas_msgs_genpy)
add_dependencies(dcas_msgs_genpy dcas_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dcas_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dcas_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dcas_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dcas_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dcas_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dcas_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(dcas_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dcas_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dcas_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dcas_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dcas_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dcas_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(dcas_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dcas_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dcas_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dcas_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
