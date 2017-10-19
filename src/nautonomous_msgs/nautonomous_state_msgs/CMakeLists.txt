cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_state_msgs)

find_package(catkin REQUIRED
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  State.msg
  Boat.msg
  BoatParam.msg
  BoatState.msg
  BoatCommand.msg
  Border.msg
  Borders.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()
