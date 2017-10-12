cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_webserver_logging)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  diagnostic_msgs
  nautonomous_logging_msgs
  nautonomous_webserver_msgs
  nautonomous_authentication_msgs
  rosbridge_library
  rosapi
  )

set(CMAKE_CXX_FLAGS "-std=c++11 -L/usr/local/lib ${CMAKE_CXX_FLAGS} -g")

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Declare a C++ executable
add_executable(logger_server
  src/logger_server.cpp
)

target_link_libraries(logger_server
  ${catkin_LIBRARIES}
  crypto
)
