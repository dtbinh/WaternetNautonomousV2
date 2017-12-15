cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_planning_and_control_using_search)

find_package(catkin REQUIRED
roscpp
nautonomous_mpc_msgs
nautonomous_state_msgs
)

catkin_package(
)



include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# add executable
add_executable(path_planner src/Astar_tree.cpp)


# target link libraries
target_link_libraries(path_planner ${catkin_LIBRARIES} )

