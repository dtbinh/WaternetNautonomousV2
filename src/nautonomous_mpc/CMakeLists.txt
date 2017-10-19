cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_mpc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nautonomous_mpc_msgs
  nautonomous_state_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(casadi_LIBRARIES
  forces/current_solver/FORCESNLPsolver_casadi2forces.c
  forces/current_solver/FORCESNLPsolver_model_1.c
  forces/current_solver/FORCESNLPsolver_model_50.c
)

add_executable(mpc_ros src/mpc_ros_node.cpp)

target_link_libraries(mpc_ros
  ${catkin_LIBRARIES}
  casadi_LIBRARIES
  ${CMAKE_CURRENT_SOURCE_DIR}/forces/current_solver/FORCESNLPsolver/lib/libFORCESNLPsolver.so
)

