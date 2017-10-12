cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_obstacle_detection)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  mpc_ros_msgs
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


add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})
target_link_libraries(example ${OpenCV_LIBRARIES})
