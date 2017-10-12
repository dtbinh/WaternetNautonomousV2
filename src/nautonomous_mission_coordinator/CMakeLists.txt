cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_mission_coordinator)


find_package(catkin REQUIRED COMPONENTS roscpp roslib actionlib geometry_msgs message_generation tf nautonomous_map_msgs nautonomous_mission_msgs nautonomous_routing_msgs)


find_package(Boost REQUIRED COMPONENTS system thread) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 -L/usr/local/lib ${CMAKE_CXX_FLAGS} -g")

add_executable(mission_coordinator_node
  src/mission_coordinator_node.cpp
)

add_dependencies(
	mission_coordinator_node
	${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_library(move_base_client src/move_base_client.cpp)


add_dependencies(
	move_base_client
	${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_library(mission_coordinator_server src/mission_coordinator_server.cpp)


add_dependencies(
	mission_coordinator_server
	${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(move_base_client ${catkin_LIBRARIES} )
target_link_libraries(mission_coordinator_server ${catkin_LIBRARIES} move_base_client)
target_link_libraries(mission_coordinator_node ${catkin_LIBRARIES} move_base_client mission_coordinator_server)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS mission_coordinator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
