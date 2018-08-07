cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_planning_and_control_using_search)

find_package(catkin REQUIRED
roscpp
nautonomous_mpc_msgs
nautonomous_state_msgs
tf
)

catkin_package(
)



include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


# add executable
add_executable(grid_planner src/Astar_grid.cpp)
add_executable(grid_planner_2 src/Astar_grid_with_motion_primitives.cpp)

add_executable(Jump_point_search src/Jump_point_search.cpp)

add_executable(rrt_planner src/RRT.cpp)

add_executable(visualization src/visualization.cpp)
add_executable(mission_control src/mission_control_sim.cpp)
add_executable(mission_control_slam src/mission_control_slam.cpp)
add_executable(mission_control_real src/mission_control_real.cpp)
add_executable(mission_control_gazebo src/mission_control_gazebo.cpp)
#add_executable(EKF src/ExtendedKalmanFilter_sim.cpp)




# target link libraries
target_link_libraries(grid_planner ${catkin_LIBRARIES} )
target_link_libraries(grid_planner_2 ${catkin_LIBRARIES} )

target_link_libraries(Jump_point_search ${catkin_LIBRARIES} )
target_link_libraries(rrt_planner ${catkin_LIBRARIES} )

target_link_libraries(visualization ${catkin_LIBRARIES} )
target_link_libraries(mission_control ${catkin_LIBRARIES} )
target_link_libraries(mission_control_slam ${catkin_LIBRARIES} )
target_link_libraries(mission_control_real ${catkin_LIBRARIES} )
target_link_libraries(mission_control_gazebo ${catkin_LIBRARIES} )
#target_link_libraries(EKF ${catkin_LIBRARIES} )
