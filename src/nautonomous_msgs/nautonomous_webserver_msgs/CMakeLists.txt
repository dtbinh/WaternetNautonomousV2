cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_webserver_msgs)

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   message_generation
)

add_message_files(
   FILES
   NautonomousAuthentication.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
   ${catkin_INCLUDE_DIRS}
)

