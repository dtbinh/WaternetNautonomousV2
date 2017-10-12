cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_actuation_propulsion_sabertooth)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rostest
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -L/usr/local/lib ${CMAKE_CXX_FLAGS} -g")

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(serial_unix
  src/impl/unix.cpp
)

add_library(actuation_watchdog
  src/actuation_watchdog.cpp
)

add_library(sabertooth_packet
  src/sabertooth_packet.cpp
)

add_library(sabertooth_motor_driver
  src/sabertooth_motor_driver.cpp
)

add_library(sabertooth_propulsion
  src/sabertooth_propulsion.cpp
)

add_library(sabertooth_serial
  src/sabertooth_serial.cpp
)

add_library(serial
  src/serial.cpp
)

add_executable(sabertooth_propulsion_node src/sabertooth_propulsion_node.cpp)

target_link_libraries(sabertooth_serial ${catkin_LIBRARIES} serial serial_unix sabertooth_packet)
target_link_libraries(sabertooth_motor_driver ${catkin_LIBRARIES} sabertooth_packet)
target_link_libraries(sabertooth_propulsion ${catkin_LIBRARIES} actuation_watchdog sabertooth_motor_driver sabertooth_serial sabertooth_packet)
target_link_libraries(sabertooth_propulsion_node ${catkin_LIBRARIES} sabertooth_propulsion)

# if(CATKIN_ENABLE_TESTING)
#   catkin_add_gtest(sabertooth_test test/sabertooth.test src/test/sabertooth_test.cpp )
#   target_link_libraries(sabertooth_test ${catkin_LIBRARIES} sabertooth_propulsion)
# endif()
