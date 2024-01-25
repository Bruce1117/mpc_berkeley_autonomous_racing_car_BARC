# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "barc: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ibarc:/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Ibarc:/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(barc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg" NAME_WE)
add_custom_target(_barc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barc" "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg" NAME_WE)
add_custom_target(_barc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barc" "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg" ""
)

get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg" NAME_WE)
add_custom_target(_barc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barc" "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barc
)
_generate_msg_cpp(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barc
)
_generate_msg_cpp(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barc
)

### Generating Services

### Generating Module File
_generate_module_cpp(barc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(barc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(barc_generate_messages barc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg" NAME_WE)
add_dependencies(barc_generate_messages_cpp _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg" NAME_WE)
add_dependencies(barc_generate_messages_cpp _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg" NAME_WE)
add_dependencies(barc_generate_messages_cpp _barc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barc_gencpp)
add_dependencies(barc_gencpp barc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barc
)
_generate_msg_eus(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barc
)
_generate_msg_eus(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barc
)

### Generating Services

### Generating Module File
_generate_module_eus(barc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(barc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(barc_generate_messages barc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg" NAME_WE)
add_dependencies(barc_generate_messages_eus _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg" NAME_WE)
add_dependencies(barc_generate_messages_eus _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg" NAME_WE)
add_dependencies(barc_generate_messages_eus _barc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barc_geneus)
add_dependencies(barc_geneus barc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barc
)
_generate_msg_lisp(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barc
)
_generate_msg_lisp(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barc
)

### Generating Services

### Generating Module File
_generate_module_lisp(barc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(barc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(barc_generate_messages barc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg" NAME_WE)
add_dependencies(barc_generate_messages_lisp _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg" NAME_WE)
add_dependencies(barc_generate_messages_lisp _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg" NAME_WE)
add_dependencies(barc_generate_messages_lisp _barc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barc_genlisp)
add_dependencies(barc_genlisp barc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barc
)
_generate_msg_nodejs(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barc
)
_generate_msg_nodejs(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barc
)

### Generating Services

### Generating Module File
_generate_module_nodejs(barc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(barc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(barc_generate_messages barc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg" NAME_WE)
add_dependencies(barc_generate_messages_nodejs _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg" NAME_WE)
add_dependencies(barc_generate_messages_nodejs _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg" NAME_WE)
add_dependencies(barc_generate_messages_nodejs _barc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barc_gennodejs)
add_dependencies(barc_gennodejs barc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barc
)
_generate_msg_py(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barc
)
_generate_msg_py(barc
  "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barc
)

### Generating Services

### Generating Module File
_generate_module_py(barc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(barc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(barc_generate_messages barc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/camera_out.msg" NAME_WE)
add_dependencies(barc_generate_messages_py _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/input.msg" NAME_WE)
add_dependencies(barc_generate_messages_py _barc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bruce/PycharmProjects/barc_ros_ws/src/barc/msg/State.msg" NAME_WE)
add_dependencies(barc_generate_messages_py _barc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barc_genpy)
add_dependencies(barc_genpy barc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(barc_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(barc_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(barc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(barc_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET barc_generate_messages_cpp)
  add_dependencies(barc_generate_messages_cpp barc_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(barc_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(barc_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(barc_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(barc_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET barc_generate_messages_eus)
  add_dependencies(barc_generate_messages_eus barc_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(barc_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(barc_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(barc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(barc_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET barc_generate_messages_lisp)
  add_dependencies(barc_generate_messages_lisp barc_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(barc_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(barc_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(barc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(barc_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET barc_generate_messages_nodejs)
  add_dependencies(barc_generate_messages_nodejs barc_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(barc_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(barc_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(barc_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(barc_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET barc_generate_messages_py)
  add_dependencies(barc_generate_messages_py barc_generate_messages_py)
endif()
