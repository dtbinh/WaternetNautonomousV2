cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_mission_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs geometry_msgs message_generation std_msgs)

add_message_files(
	DIRECTORY msg
	FILES
	MissionStatus.msg
	OperationPlan.msg
)

add_action_files(
	DIRECTORY action 
	FILES MissionPlan.action
)

generate_messages(
	DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
