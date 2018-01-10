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
add_executable(grid_planner src/Astar_grid.cpp)
add_executable(grid_planner_2 src/Astar_grid_2.cpp)
add_executable(grid_planner_3 src/Astar_grid_heap.cpp)
add_executable(grid_planner_4 src/Astar_grid_heuristics.cpp)
add_executable(tree_path_planner src/Astar_tree.cpp)
add_executable(tree_opt src/Astar_tree_search.cpp)
add_executable(tree_opt_closest_first src/Astar_tree_search_closest_first.cpp)


# target link libraries
target_link_libraries(grid_planner ${catkin_LIBRARIES} )
target_link_libraries(grid_planner_2 ${catkin_LIBRARIES} )
target_link_libraries(grid_planner_3 ${catkin_LIBRARIES} )
target_link_libraries(grid_planner_4 ${catkin_LIBRARIES} )
target_link_libraries(tree_path_planner ${catkin_LIBRARIES} )
target_link_libraries(tree_opt ${catkin_LIBRARIES} )
target_link_libraries(tree_opt_closest_first ${catkin_LIBRARIES} )

