# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "nautonomous_sensor_imu: 1 messages, 0 services")

set(MSG_I_FLAGS "-Inautonomous_sensor_imu:/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(nautonomous_sensor_imu_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg" NAME_WE)
add_custom_target(_nautonomous_sensor_imu_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nautonomous_sensor_imu" "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg" "geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(nautonomous_sensor_imu
  "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nautonomous_sensor_imu
)

### Generating Services

### Generating Module File
_generate_module_cpp(nautonomous_sensor_imu
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nautonomous_sensor_imu
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(nautonomous_sensor_imu_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(nautonomous_sensor_imu_generate_messages nautonomous_sensor_imu_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg" NAME_WE)
add_dependencies(nautonomous_sensor_imu_generate_messages_cpp _nautonomous_sensor_imu_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautonomous_sensor_imu_gencpp)
add_dependencies(nautonomous_sensor_imu_gencpp nautonomous_sensor_imu_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautonomous_sensor_imu_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(nautonomous_sensor_imu
  "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nautonomous_sensor_imu
)

### Generating Services

### Generating Module File
_generate_module_eus(nautonomous_sensor_imu
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nautonomous_sensor_imu
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(nautonomous_sensor_imu_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(nautonomous_sensor_imu_generate_messages nautonomous_sensor_imu_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg" NAME_WE)
add_dependencies(nautonomous_sensor_imu_generate_messages_eus _nautonomous_sensor_imu_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautonomous_sensor_imu_geneus)
add_dependencies(nautonomous_sensor_imu_geneus nautonomous_sensor_imu_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautonomous_sensor_imu_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(nautonomous_sensor_imu
  "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nautonomous_sensor_imu
)

### Generating Services

### Generating Module File
_generate_module_lisp(nautonomous_sensor_imu
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nautonomous_sensor_imu
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(nautonomous_sensor_imu_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(nautonomous_sensor_imu_generate_messages nautonomous_sensor_imu_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg" NAME_WE)
add_dependencies(nautonomous_sensor_imu_generate_messages_lisp _nautonomous_sensor_imu_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautonomous_sensor_imu_genlisp)
add_dependencies(nautonomous_sensor_imu_genlisp nautonomous_sensor_imu_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautonomous_sensor_imu_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(nautonomous_sensor_imu
  "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nautonomous_sensor_imu
)

### Generating Services

### Generating Module File
_generate_module_nodejs(nautonomous_sensor_imu
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nautonomous_sensor_imu
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(nautonomous_sensor_imu_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(nautonomous_sensor_imu_generate_messages nautonomous_sensor_imu_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg" NAME_WE)
add_dependencies(nautonomous_sensor_imu_generate_messages_nodejs _nautonomous_sensor_imu_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautonomous_sensor_imu_gennodejs)
add_dependencies(nautonomous_sensor_imu_gennodejs nautonomous_sensor_imu_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautonomous_sensor_imu_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(nautonomous_sensor_imu
  "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautonomous_sensor_imu
)

### Generating Services

### Generating Module File
_generate_module_py(nautonomous_sensor_imu
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautonomous_sensor_imu
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(nautonomous_sensor_imu_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(nautonomous_sensor_imu_generate_messages nautonomous_sensor_imu_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/waternet/programmeren/nautonomous/WaternetNautonomousV2/src/nautonomous_sensor_imu_razor/msg/FilterOutput.msg" NAME_WE)
add_dependencies(nautonomous_sensor_imu_generate_messages_py _nautonomous_sensor_imu_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nautonomous_sensor_imu_genpy)
add_dependencies(nautonomous_sensor_imu_genpy nautonomous_sensor_imu_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nautonomous_sensor_imu_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nautonomous_sensor_imu)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nautonomous_sensor_imu
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(nautonomous_sensor_imu_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nautonomous_sensor_imu)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nautonomous_sensor_imu
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(nautonomous_sensor_imu_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nautonomous_sensor_imu)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nautonomous_sensor_imu
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(nautonomous_sensor_imu_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nautonomous_sensor_imu)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nautonomous_sensor_imu
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(nautonomous_sensor_imu_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautonomous_sensor_imu)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautonomous_sensor_imu\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nautonomous_sensor_imu
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(nautonomous_sensor_imu_generate_messages_py geometry_msgs_generate_messages_py)
endif()
