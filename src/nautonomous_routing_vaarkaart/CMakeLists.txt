cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_routing_vaarkaart)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nautonomous_routing_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  lib/vaarkaart
  lib/astar
  lib/graph
)
