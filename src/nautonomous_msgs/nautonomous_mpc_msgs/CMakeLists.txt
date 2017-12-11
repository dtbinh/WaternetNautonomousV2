cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_mpc_msgs)

find_package(catkin REQUIRED
  message_generation
  nautonomous_state_msgs
  geometry_msgs
)

add_message_files(
  FILES
  StageVariable.msg
  WaypointList.msg
  Waypoint.msg
  CostsWN.msg
  CostsCC.msg
  Obstacle.msg
  Obstacles.msg
  Output.msg
  Route.msg
)

generate_messages(
  DEPENDENCIES
  nautonomous_state_msgs
)

catkin_package()
