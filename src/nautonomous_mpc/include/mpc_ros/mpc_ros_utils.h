/*
 * Author: Antoine Seewer
 */

#ifndef INCLUDE_MPC_ROS_MPC_ROS_UTILS_H_
#define INCLUDE_MPC_ROS_MPC_ROS_UTILS_H_

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

namespace mpc_ros {

// Convenience method for computing the norm of a 3D vector
double norm(const geometry_msgs::Vector3& v) {
  return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

// Convenience method for computing the distance between two 3D points
double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

} // namespace mpc_ros

#endif /* INCLUDE_MPC_ROS_MPC_ROS_UTILS_H_ */
