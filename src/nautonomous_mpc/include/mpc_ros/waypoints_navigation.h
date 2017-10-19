/*
 * Author: Antoine Seewer
 */

#ifndef INCLUDE_MPC_ROS_WAYPOINTS_NAVIGATION_H_
#define INCLUDE_MPC_ROS_WAYPOINTS_NAVIGATION_H_

#include <mpc_ros/mpc_ros.h>
#include <nautonomous_mpc_msgs/CostsWN.h>

class WaypointsNavigation: public MpcRos {
 public:
  WaypointsNavigation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Convenience method for loading the costs for the objective function
  void loadCosts();

  // Convenience method for managing the waypoints
  void manageWaypoints();

  // Convenience method for setting all required solver parameters
  void setSolverParameters();

  // Convenience method for setting the initial condition on the vehicle states
  void setXinit(const nautonomous_state_msgs::BoatState& state);

  // Convenience method for setting the parameters
  void setAllParameters(const geometry_msgs::Point& goal,
                        const nautonomous_mpc_msgs::CostsWN& costs,
                        nautonomous_mpc_msgs::Obstacles obstacles);

  // Convenience methods for setting the initial guess
  void setX0(const nautonomous_state_msgs::BoatState& state); // from state and zero inputs
  void setX0(const FORCESNLPsolver_output& output); // from shifted solver output

  // Convenience method for publishing the solver output
  void publishSolverOutput();

 private:
  // Waypoints navigation costs
  nautonomous_mpc_msgs::CostsWN costs_;

  // Additional settings
  double waypoint_position_threshold_;
};

WaypointsNavigation::WaypointsNavigation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : MpcRos(nh, nh_private, 50, 8, 6, 42, 6) {
  // Load additional parameters
  nh_private_.param("waypoint_position_threshold", waypoint_position_threshold_, 0.5);

  // Load costs for the objective function
  loadCosts();

  // Remove start position from waypoints
  boat_.eraseFrontWaypoint();
}

void WaypointsNavigation::loadCosts() {
  nh_private_.param("costsWN/q_goal", costs_.q_goal, 0.0);
  nh_private_.param("costsWN/qN_goal", costs_.qN_goal, 500.0);
  nh_private_.param("costsWN/q_u", costs_.q_u, 0.1);
  nh_private_.param("costsWN/qN_u", costs_.qN_u, 0.1);
}

void WaypointsNavigation::manageWaypoints() {
  while ((boat_.waypoints().size() > 1 &&
             mpc_ros::distance(boat_.position(), boat_.waypoints().front()) < waypoint_position_threshold_) ||
         (boat_.waypoints().size() == 1 &&
             mpc_ros::distance(boat_.position(), boat_.waypoints().front()) < goal_position_threshold_ &&
             mpc_ros::norm(boat_.velocity()) < goal_velocity_threshold_)) {
    boat_.eraseFrontWaypoint();
    if (boat_.waypoints().empty()) {
      if (verbose_) {
        ROS_INFO("[%s] At goal!", ros::this_node::getName().c_str());
      }
      if (statistics_) {
        printStatistics();
      }
    }
    else {
      if (verbose_) {
        ROS_INFO("[%s] Waypoint reached. Aiming for next one.", ros::this_node::getName().c_str());
      }
    }
  }
}

void WaypointsNavigation::setSolverParameters() {
  setXinit(boat_.state()); // initial condition on vehicle states
  setAllParameters(boat_.waypoints().front(), costs_, obstacles_); // parameters
  if (exitflag_ == -1000) {
    setX0(boat_.state()); // first initial guess from initial state and zero inputs
  }
  else {
    setX0(output_); // initial guess from shifted solver output
  }
}

void WaypointsNavigation::setXinit(const nautonomous_state_msgs::BoatState& state) {
  int k = 0;
  params_.xinit[k++] = state.pose.position.x;
  params_.xinit[k++] = state.pose.position.y;
  params_.xinit[k++] = tf::getYaw(state.pose.orientation);
  params_.xinit[k++] = state.twist.linear.x;
  params_.xinit[k++] = state.twist.linear.y;
  params_.xinit[k++] = state.twist.angular.z;
  ROS_ASSERT(k == Nstates_);
}

void WaypointsNavigation::setAllParameters(const geometry_msgs::Point& goal,
                                           const nautonomous_mpc_msgs::CostsWN& costs,
                                           nautonomous_mpc_msgs::Obstacles obstacles) {
  // Check number of obstacles
  if (obstacles.obstacles.size() > Nobstacles_) {
    ROS_WARN("[%s] Too many obstacles! The current solver only supports up to %d obstacles and therefore only the first"
             "%d obstacles will be taken into account. Make sure they are sorted properly or consider generating a new solver.",
             ros::this_node::getName().c_str(), Nobstacles_, Nobstacles_);
  }

  // Set parameters
  int k = 0;
  int j_obstacles;
  for (int i = 0; i < N_; i++) {
    params_.all_parameters[k++] = boat_.m();
    params_.all_parameters[k++] = boat_.Iz();
    params_.all_parameters[k++] = boat_.l();
    params_.all_parameters[k++] = boat_.beta();
    params_.all_parameters[k++] = boat_.Dx();
    params_.all_parameters[k++] = boat_.Dy();
    params_.all_parameters[k++] = boat_.Dtheta();
    params_.all_parameters[k++] = controller_dt_;
    params_.all_parameters[k++] = goal.x;
    params_.all_parameters[k++] = goal.y;
    params_.all_parameters[k++] = (i == N_-1) ? costs.qN_goal : costs.q_goal;
    params_.all_parameters[k++] = (i == N_-1) ? costs.qN_u : costs.q_u;
    for (int j = 0; j < Nobstacles_; j++) {
      if (j < obstacles.obstacles.size()) {
        j_obstacles = j;
      }
      else {
        j_obstacles = obstacles.obstacles.size() - 1;
      }
      params_.all_parameters[k+j] = obstacles.obstacles[j_obstacles].state.pose.position.x;
      params_.all_parameters[k+Nobstacles_+j] = obstacles.obstacles[j_obstacles].state.pose.position.y;
      params_.all_parameters[k+2*Nobstacles_+j] = obstacles.obstacles[j_obstacles].major_semiaxis;
      params_.all_parameters[k+3*Nobstacles_+j] = obstacles.obstacles[j_obstacles].minor_semiaxis;
      params_.all_parameters[k+4*Nobstacles_+j] = tf::getYaw(obstacles.obstacles[j_obstacles].state.pose.orientation);
    }
    k += Nobstacles_ * Npar_obstacle_;
    predictObstaclesMovement(obstacles);
  }
  ROS_ASSERT(k == N_ * Npar_);
}

void WaypointsNavigation::setX0(const nautonomous_state_msgs::BoatState& state) {
  int k = 0;
  for (int i = 0; i < N_; i++) {
    params_.x0[k++] = 0.0;
    params_.x0[k++] = 0.0;
    params_.x0[k++] = state.pose.position.x;
    params_.x0[k++] = state.pose.position.y;
    params_.x0[k++] = tf::getYaw(state.pose.orientation);
    params_.x0[k++] = state.twist.linear.x;
    params_.x0[k++] = state.twist.linear.y;
    params_.x0[k++] = state.twist.angular.z;
  }
  ROS_ASSERT(k == N_ * Nvar_);
}

void WaypointsNavigation::setX0(const FORCESNLPsolver_output& output) {
  for (int i = 0; i < (N_-1) * Nvar_; i++) {
    params_.x0[i] = output.Z[i+Nvar_];
  }
  for (int i = (N_-1) * Nvar_; i < N_ * Nvar_; i++) {
    params_.x0[i] = output.Z[i];
  }
}

void WaypointsNavigation::publishSolverOutput() {
  nautonomous_mpc_msgs::Output solver_output;
  nautonomous_mpc_msgs::StageVariable z;
  int k = 0;
  for (int i = 0; i < N_; i++) {
    z.T_l = output_.Z[k++];
    z.T_r = output_.Z[k++];
    z.x = output_.Z[k++];
    z.y = output_.Z[k++];
    z.theta = output_.Z[k++];
    z.u = output_.Z[k++];
    z.v = output_.Z[k++];
    z.omega = output_.Z[k++];
    solver_output.Z.push_back(z);
  }
  ROS_ASSERT(k == N_ * Nvar_);
  solver_output.exitflag = exitflag_;

  pub_solver_output_.publish(solver_output);
}

#endif /* INCLUDE_MPC_ROS_WAYPOINTS_NAVIGATION_H_ */
