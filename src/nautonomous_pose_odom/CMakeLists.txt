cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_pose_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gps_common
  nautonomous_pose_msgs
)

catkin_package(

)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(coordinate_odometry_node src/coordinate_odometry_node.cpp)

target_link_libraries(coordinate_odometry_node
  ${catkin_LIBRARIES}
)
