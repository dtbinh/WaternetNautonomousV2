/*
 * Author: Antoine Seewer
 */

#ifndef INCLUDE_MPC_ROS_BOAT_H_
#define INCLUDE_MPC_ROS_BOAT_H_

#include <ros/ros.h>
#include <nautonomous_state_msgs/Boat.h>
#include <mpc_ros/mpc_ros_defines.h>
#include <mpc_ros/contouring_control_utils.h>
#include <tf/transform_datatypes.h>

class Boat {
 public:
  Boat(const nautonomous_state_msgs::Boat& boat_msg);
  Boat(const ros::NodeHandle& nh);

  // Convenience method for computing the quadratic coefficients of the motor dynamics
  void computeMotorCoefficients();

  // Convenience methods for converting thrust to unit voltage input and vice versa
  double thrustToUnitVoltage(const double& thrust);
  double unitVoltageToThrust(const double& unit_voltage);

  // Convenience method for performing a simulation step on the boat
  void doStep(const double& dt, const int& mpc_method, const bool& verbose = false);

  // Convenience methods for updating the boat parameters
  void updateParameters(const nautonomous_state_msgs::BoatParam& param);

  // Convenience methods for updating the boat state
  void updateState(const nautonomous_state_msgs::BoatState& state);

  // Convenience methods for dealing with the boat waypoints
  void eraseFrontWaypoint();
  void clearWaypoints();

  // Set boat command
  void setCommand(const nautonomous_state_msgs::BoatCommand& command);

  // Get boat message
  nautonomous_state_msgs::Boat getBoatMsg() const { return boat_msg_; }

  // Convenience methods for getting the boat members
  int32_t id() const { return boat_msg_.id; }
  // Parameters
  double length() const { return boat_msg_.param.length; }
  double width() const { return boat_msg_.param.width; }
  double safetyRadius() const { return boat_msg_.param.safety_radius; }
  double m() const { return boat_msg_.param.m; }
  double Iz() const { return boat_msg_.param.I_z; }
  double l() const { return boat_msg_.param.l; }
  double Tfull() const { return boat_msg_.param.T_full; }
  double Thalf() const { return boat_msg_.param.T_half; }
  double beta() const { return boat_msg_.param.beta; }
  double Dx() const { return boat_msg_.param.D_x; }
  double Dy() const { return boat_msg_.param.D_y; }
  double Dtheta() const { return boat_msg_.param.D_theta; }
  // Quadratic coefficients of the motor dynamics
  double Ta() const { return a_; }
  double Tb() const { return b_; }
  // State
  nautonomous_state_msgs::BoatState state() const { return boat_msg_.state; }
  geometry_msgs::Point position() const { return boat_msg_.state.pose.position; }
  geometry_msgs::Quaternion orientation() const { return boat_msg_.state.pose.orientation; }
  geometry_msgs::Vector3 velocity() const { return boat_msg_.state.twist.linear; }
  geometry_msgs::Vector3 angularVelocity() const { return boat_msg_.state.twist.angular; }
  double progress() const { return boat_msg_.state.progress; }
  // Waypoints
  std::vector<geometry_msgs::Point> waypoints() const { return boat_msg_.waypoints; }
  // Prior segments lengths
  std::vector<double> priorSegmentsLengths() const { return prior_segments_lengths_; }
  // Command
  nautonomous_state_msgs::BoatCommand command() const { return boat_msg_.command; }

 private:
  // Boat message
  nautonomous_state_msgs::Boat boat_msg_;

  // Quadratic coefficients of the motor dynamics
  double a_;
  double b_;

  // Prior segments lengths (only for contouring control)
  std::vector<double> prior_segments_lengths_;
};

Boat::Boat(const nautonomous_state_msgs::Boat& boat_msg)
    : boat_msg_(boat_msg) {
  // Compute the quadratic coefficients of the motor dynamics
  computeMotorCoefficients();

  // Compute the prior segments lengths
  prior_segments_lengths_ = contouring_control::computePriorSegmentsLengths(boat_msg_.waypoints);
}

Boat::Boat(const ros::NodeHandle& nh) {
  boat_msg_.id = 0;

  // Load boat parameters
  nh.param("/boat/length", boat_msg_.param.length, 2.6);
  nh.param("/boat/width", boat_msg_.param.width, 2.0);
  nh.param("/boat/safety_radius", boat_msg_.param.safety_radius,
           sqrt(pow(boat_msg_.param.length / 2, 2) + pow(boat_msg_.param.width / 2, 2)) + 1.0);
  nh.param("/boat/m", boat_msg_.param.m, 200.0);
  nh.param("/boat/I_z", boat_msg_.param.I_z, 14.0);
  nh.param("/boat/l", boat_msg_.param.l, 0.8);
  nh.param("/boat/T_full", boat_msg_.param.T_full, 135.0);
  nh.param("/boat/T_half", boat_msg_.param.T_half, 50.0);
  nh.param("/boat/beta", boat_msg_.param.beta, 0.37);
  nh.param("/boat/D_x", boat_msg_.param.D_x, 38.0);
  nh.param("/boat/D_y", boat_msg_.param.D_y, 5280.0);
  nh.param("/boat/D_theta", boat_msg_.param.D_theta, 104.0);
  ROS_ASSERT(boat_msg_.param.safety_radius != 0);

  // Compute the quadratic coefficients of the motor dynamics
  computeMotorCoefficients();

  // Initialize boat state
  nh.param("/boat/start_x", boat_msg_.state.pose.position.x, 0.0);
  nh.param("/boat/start_y", boat_msg_.state.pose.position.y, 0.0);
  boat_msg_.state.pose.position.z = 0.0;
  double start_theta;
  nh.param("/boat/start_theta", start_theta, 0.0);
  boat_msg_.state.pose.orientation = tf::createQuaternionMsgFromYaw(start_theta);
  boat_msg_.state.twist.linear.x = 0.0;
  boat_msg_.state.twist.linear.y = 0.0;
  boat_msg_.state.twist.linear.z = 0.0;
  boat_msg_.state.twist.angular.x = 0.0;
  boat_msg_.state.twist.angular.y = 0.0;
  boat_msg_.state.twist.angular.z = 0.0;
  boat_msg_.state.progress = 0.0;

  // Initialize boat waypoints
  boat_msg_.waypoints.push_back(boat_msg_.state.pose.position); // first add initial position
  std::vector<double> waypoints_x, waypoints_y;
  nh.getParam("/boat/waypoints_x", waypoints_x);
  nh.getParam("/boat/waypoints_y", waypoints_y);
  ROS_ASSERT(waypoints_x.size() == waypoints_y.size());
  geometry_msgs::Point waypoint;
  for (int i = 0; i < waypoints_x.size(); i++) {
    waypoint.x = waypoints_x[i];
    waypoint.y = waypoints_y[i];
    waypoint.z = 0.0;
    boat_msg_.waypoints.push_back(waypoint);
  }

  // Compute the prior segments lengths
  prior_segments_lengths_ = contouring_control::computePriorSegmentsLengths(boat_msg_.waypoints);

  // Initialize boat command
  boat_msg_.command.u_l = 0.0;
  boat_msg_.command.u_r = 0.0;
}

void Boat::computeMotorCoefficients() {
  a_ = 2 * boat_msg_.param.T_full - 4 * boat_msg_.param.T_half;
  b_ = boat_msg_.param.T_full - a_;
}

double Boat::thrustToUnitVoltage(const double& thrust) {
  if (thrust >= 0) {
    return (-b_ + sqrt(pow(b_, 2) + 4 * a_ * thrust)) / (2 * a_);
  }
  else {
    return (b_ - sqrt(pow(b_, 2) - 4 * a_ * thrust / boat_msg_.param.beta)) / (2 * a_);
  }
}

double Boat::unitVoltageToThrust(const double& unit_voltage) {
  if (unit_voltage >= 0) {
    return a_ * pow(unit_voltage, 2) + b_ * unit_voltage;
  }
  else {
    return boat_msg_.param.beta * (-a_ * pow(unit_voltage, 2) + b_ * unit_voltage);
  }
}

void Boat::doStep(const double& dt, const int& mpc_method, const bool& verbose) {
  // Compute the thrusts from the unit voltage inputs
  double T_l = unitVoltageToThrust(boat_msg_.command.u_l);
  double T_r = unitVoltageToThrust(boat_msg_.command.u_r);

  // Simulate
  // won't give the same as MPC planner even if the boat parameters are identical since the discretization differs
  double theta = tf::getYaw(boat_msg_.state.pose.orientation);
  double u = boat_msg_.state.twist.linear.x;
  double v = boat_msg_.state.twist.linear.y;
  double omega = boat_msg_.state.twist.angular.z;
  boat_msg_.state.pose.position.x += (u * cos(theta) + v * sin(theta)) * dt;
  boat_msg_.state.pose.position.y += (u * sin(theta) - v * cos(theta)) * dt;
  boat_msg_.state.pose.orientation = tf::createQuaternionMsgFromYaw(theta - omega * dt);
  boat_msg_.state.twist.linear.x += ((T_l + T_r - boat_msg_.param.D_x * u) / boat_msg_.param.m + omega * v) * dt;
  boat_msg_.state.twist.linear.y += (-boat_msg_.param.D_y * v / boat_msg_.param.m - omega * u) * dt;
  boat_msg_.state.twist.angular.z += ((boat_msg_.param.l * (T_l - T_r) - boat_msg_.param.D_theta * omega)
      / boat_msg_.param.I_z) * dt;

  if (mpc_method == mpc_ros::method::CONTOURING_CONTROL) {
    // Compute progress on path
    boat_msg_.state.progress = contouring_control::computeProgress(boat_msg_.state.pose.position,
                                                                   boat_msg_.waypoints[0], boat_msg_.waypoints[1],
                                                                   prior_segments_lengths_.front(), prior_segments_lengths_.back());

    // Remove obsolete waypoints
    while (!prior_segments_lengths_.empty() &&
           boat_msg_.state.progress > prior_segments_lengths_[1] / prior_segments_lengths_.back()) {
      eraseFrontWaypoint();
      if (verbose) {
        ROS_INFO("[%s] Waypoint removed.", ros::this_node::getName().c_str());
      }
    }
  }
}

void Boat::updateParameters(const nautonomous_state_msgs::BoatParam& param) {
  boat_msg_.param = param;

  // Recompute the quadratic coefficients of the motor dynamics
  computeMotorCoefficients();
}

void Boat::updateState(const nautonomous_state_msgs::BoatState& state) {
  boat_msg_.state = state;
}

void Boat::eraseFrontWaypoint() {
  boat_msg_.waypoints.erase(boat_msg_.waypoints.begin());
  prior_segments_lengths_.erase(prior_segments_lengths_.begin());
}

void Boat::clearWaypoints() {
  boat_msg_.waypoints.clear();
  prior_segments_lengths_.clear();
}

void Boat::setCommand(const nautonomous_state_msgs::BoatCommand& command) {
  boat_msg_.command = command;
}

#endif /* INCLUDE_MPC_ROS_BOAT_H_ */
