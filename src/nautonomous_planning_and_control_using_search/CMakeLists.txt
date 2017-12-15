cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_planning_and_control_using_search)

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )


find_package(catkin REQUIRED
roscpp
nautonomous_mpc_msgs
nautonomous_state_msgs
)

#find_package( ACADO REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)



include_directories(
	include
)

# add executable
# add_executable(mpc src/mpc.cpp)


# target link libraries
#target_link_libraries(mpc ${catkin_LIBRARIES} acado_auxiliary_functions acado_integrator acado_solver acado_qpoases_interface)

