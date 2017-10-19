/*
 * Author: Antoine Seewer
 */

#ifndef INCLUDE_MPC_ROS_CONTOURING_CONTROL_UTILS_H_
#define INCLUDE_MPC_ROS_CONTOURING_CONTROL_UTILS_H_

#include <mpc_ros/mpc_ros_utils.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace contouring_control {

// Convenience method for computing the prior segments lengths
std::vector<double> computePriorSegmentsLengths(const std::vector<geometry_msgs::Point>& waypoints) {
  std::vector<double> prior_segments_lengths;

  prior_segments_lengths.push_back(0.0);
  for (int i = 0; i < waypoints.size() - 1; i++) {
    prior_segments_lengths.push_back(prior_segments_lengths.back() + mpc_ros::distance(waypoints[i], waypoints[i+1]));
  }

  return prior_segments_lengths;
}

// Convenience method for computing the progress on path
double computeProgress(const geometry_msgs::Point& position,
                       const geometry_msgs::Point& pA, const geometry_msgs::Point& pB,
                       const double& prior_segments_length, const double& path_length) {
  // Compute current segment parallel vector
  geometry_msgs::Vector3 t;
  t.x = pB.x - pA.x;
  t.y = pB.y - pA.y;
  double norm = mpc_ros::norm(t);
  if (norm != 0) {
    t.x /= norm;
    t.y /= norm;
  }

  // Compute current segment progress
  double segment_progress = t.x * (position.x - pA.x) + t.y * (position.y - pA.y);

  // Return unit progress ([0,1])
  return (prior_segments_length + segment_progress) / path_length;
}

} // namespace contouring_control

#endif /* INCLUDE_MPC_ROS_CONTOURING_CONTROL_UTILS_H_ */
