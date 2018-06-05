cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_obstacle_detection)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  nautonomous_mpc_msgs
  nautonomous_state_msgs
  nautonomous_pose_msgs
)

find_package(OpenCV REQUIRED)


catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(Ellipse src/Ellipse.cpp)
add_executable(Laserscan src/Laserscan.cpp)
add_executable(map_modifier src/Map_modifier.cpp)
add_executable(Obstacle_tracker src/Obstacle_tracker.cpp)
add_executable(Obstacle_tracker_V2 src/Obstacle_tracker_V2.cpp)
add_executable(Obstacle_tracker_V3 src/Obstacle_tracker_V3.cpp)
add_executable(TestFile src/TestFile.cpp)
add_executable(TestFile_2 src/TestFile_2.cpp)
add_executable(visualization src/visualization.cpp)


target_link_libraries(Ellipse ${catkin_LIBRARIES})
target_link_libraries(Laserscan ${catkin_LIBRARIES})
target_link_libraries(Ellipse ${OpenCV_LIBRARIES})
target_link_libraries(map_modifier ${catkin_LIBRARIES} )
target_link_libraries(Obstacle_tracker ${catkin_LIBRARIES} )
target_link_libraries(Obstacle_tracker_V2 ${catkin_LIBRARIES} )
target_link_libraries(Obstacle_tracker_V3 ${catkin_LIBRARIES} )
target_link_libraries(TestFile ${catkin_LIBRARIES} )
target_link_libraries(TestFile_2 ${catkin_LIBRARIES} )
target_link_libraries(visualization ${catkin_LIBRARIES} )
