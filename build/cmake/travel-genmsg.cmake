# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "travel: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itravel:/home/euigon/ros_ws/travel/src/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(travel_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/euigon/ros_ws/travel/src/msg/node.msg" NAME_WE)
add_custom_target(_travel_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "travel" "/home/euigon/ros_ws/travel/src/msg/node.msg" "sensor_msgs/PointCloud2:sensor_msgs/PointField:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(travel
  "/home/euigon/ros_ws/travel/src/msg/node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/travel
)

### Generating Services

### Generating Module File
_generate_module_cpp(travel
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/travel
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(travel_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(travel_generate_messages travel_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/euigon/ros_ws/travel/src/msg/node.msg" NAME_WE)
add_dependencies(travel_generate_messages_cpp _travel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(travel_gencpp)
add_dependencies(travel_gencpp travel_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS travel_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(travel
  "/home/euigon/ros_ws/travel/src/msg/node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/travel
)

### Generating Services

### Generating Module File
_generate_module_eus(travel
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/travel
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(travel_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(travel_generate_messages travel_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/euigon/ros_ws/travel/src/msg/node.msg" NAME_WE)
add_dependencies(travel_generate_messages_eus _travel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(travel_geneus)
add_dependencies(travel_geneus travel_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS travel_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(travel
  "/home/euigon/ros_ws/travel/src/msg/node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/travel
)

### Generating Services

### Generating Module File
_generate_module_lisp(travel
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/travel
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(travel_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(travel_generate_messages travel_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/euigon/ros_ws/travel/src/msg/node.msg" NAME_WE)
add_dependencies(travel_generate_messages_lisp _travel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(travel_genlisp)
add_dependencies(travel_genlisp travel_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS travel_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(travel
  "/home/euigon/ros_ws/travel/src/msg/node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/travel
)

### Generating Services

### Generating Module File
_generate_module_nodejs(travel
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/travel
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(travel_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(travel_generate_messages travel_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/euigon/ros_ws/travel/src/msg/node.msg" NAME_WE)
add_dependencies(travel_generate_messages_nodejs _travel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(travel_gennodejs)
add_dependencies(travel_gennodejs travel_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS travel_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(travel
  "/home/euigon/ros_ws/travel/src/msg/node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/travel
)

### Generating Services

### Generating Module File
_generate_module_py(travel
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/travel
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(travel_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(travel_generate_messages travel_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/euigon/ros_ws/travel/src/msg/node.msg" NAME_WE)
add_dependencies(travel_generate_messages_py _travel_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(travel_genpy)
add_dependencies(travel_genpy travel_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS travel_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/travel)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/travel
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(travel_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(travel_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(travel_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/travel)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/travel
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(travel_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(travel_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(travel_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/travel)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/travel
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(travel_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(travel_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(travel_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/travel)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/travel
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(travel_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(travel_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(travel_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/travel)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/travel\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/travel
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(travel_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(travel_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(travel_generate_messages_py std_msgs_generate_messages_py)
endif()
