cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_propulsion_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -L/usr/local/lib ${CMAKE_CXX_FLAGS} -g")

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(differential_test_publisher
  src/differential_test_publisher.cpp
)

add_executable(differential_test_publisher_node src/differential_test_publisher_node.cpp)

target_link_libraries(differential_test_publisher_node ${catkin_LIBRARIES} differential_test_publisher)