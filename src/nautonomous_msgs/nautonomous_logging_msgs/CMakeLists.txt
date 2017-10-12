cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_logging_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  LogPropulsionStatus.msg
  LogIMU.msg
  LogGPS.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
