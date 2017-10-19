/*
 * Author: Antoine Seewer
 */

#include <ros/ros.h>
#include <mpc_ros/waypoints_navigation.h>
#include <mpc_ros/contouring_control.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_ros");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  int method;
  nh_private.param("method", method, mpc_ros::method::CONTOURING_CONTROL);

  if (method == mpc_ros::method::WAYPOINTS_NAVIGATION) {
    WaypointsNavigation mpc_ros(nh, nh_private);

    ROS_INFO("[%s] Initialized with waypoints navigation instance.", ros::this_node::getName().c_str());
    ros::spin();
  }
  else if (method == mpc_ros::method::CONTOURING_CONTROL) {
    ContouringControl mpc_ros(nh, nh_private);

    ROS_INFO("[%s] Initialized with contouring control instance.", ros::this_node::getName().c_str());
    ros::spin();
  }
  else {
    ROS_ERROR("[%s] Unknown method.", ros::this_node::getName().c_str());
  }
}
