cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_map_cropper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nautonomous_map_msgs
  nautonomous_configuration
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

