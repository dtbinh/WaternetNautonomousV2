/*
 * Author: Antoine Seewer
 */

#ifndef INCLUDE_MPC_ROS_CONTOURING_CONTROL_H_
#define INCLUDE_MPC_ROS_CONTOURING_CONTROL_H_

#include <mpc_ros/mpc_ros.h>
#include <mpc_ros/contouring_control_utils.h>
#include <nautonomous_mpc_msgs/CostsCC.h>

class ContouringControl: public MpcRos {
 public:
  ContouringControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Convenience method for loading the costs for the objective function
  void loadCosts();

  // Convenience method for managing the waypoints
  void manageWaypoints();

  // Convenience method for setting all required solver parameters
  void setSolverParameters();

  // Convenience method for setting the initial condition on the vehicle states
  void setXinit(const nautonomous_state_msgs::BoatState& state);

  // Convenience method for setting the parameters
  void setAllParameters(const std::vector<geometry_msgs::Point>& waypoints,
                        const nautonomous_mpc_msgs::CostsCC& costs,
                        nautonomous_mpc_msgs::Obstacles obstacles);

  // Convenience methods for setting the initial guess
  void setX0(const nautonomous_state_msgs::BoatState& state); // from state and zero inputs
  void setX0(const FORCESNLPsolver_output& output); // from shifted solver output

  // Convenience method for publishing the solver output
  void publishSolverOutput();

 private:
  // Waypoints navigation costs
  nautonomous_mpc_msgs::CostsCC costs_;

  // Additional settings
  const int Npoints_;
  const int Npar_point_;
};

ContouringControl::ContouringControl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : MpcRos(nh, nh_private, 50, 10, 7, 54, 6),
      Npoints_(5),
      Npar_point_(2) {
  // Load costs for the objective function
  loadCosts();
}

void ContouringControl::loadCosts() {
  nh_private_.param("costsCC/q_progress", costs_.q_progress, -100.0);
  nh_private_.param("costsCC/q_lag", costs_.q_lag, 300.0);
  nh_private_.param("costsCC/q_contouring", costs_.q_contouring, 5.0);
  nh_private_.param("costsCC/q_u", costs_.q_u, 0.01);
}

void ContouringControl::manageWaypoints() {
  // Check if at goal
  if (!boat_.waypoints().empty() &&
      mpc_ros::distance(boat_.position(), boat_.waypoints().back()) < goal_position_threshold_ &&
      mpc_ros::norm(boat_.velocity()) < goal_velocity_threshold_) {
    boat_.clearWaypoints();
    if (verbose_) {
      ROS_INFO("[%s] At goal!", ros::this_node::getName().c_str());
    }
    if (statistics_) {
      printStatistics();
    }
  }

  // Remove obsolete waypoints
  // beneficial to remove as many waypoints as possible even if the solver is able to deal with Npoints_ waypoints
  // (ease the "if_else()")
  while (boat_.waypoints().size() > 2 && boat_.progress() > boat_.priorSegmentsLengths()[1] / boat_.priorSegmentsLengths().back()) {
    boat_.eraseFrontWaypoint();
    if (verbose_) {
      ROS_INFO("[%s] Waypoint removed.", ros::this_node::getName().c_str());
    }
  }
}

void ContouringControl::setSolverParameters() {
  setXinit(boat_.state()); // initial condition on vehicle states
  setAllParameters(boat_.waypoints(), costs_, obstacles_); // parameters
  if (exitflag_ == -1000) {
    setX0(boat_.state()); // first initial guess from initial state and zero inputs
  }
  else {
    setX0(output_); // initial guess from shifted solver output
  }
}

void ContouringControl::setXinit(const nautonomous_state_msgs::BoatState& state) {
  int k = 0;
  params_.xinit[k++] = state.pose.position.x;
  params_.xinit[k++] = state.pose.position.y;
  params_.xinit[k++] = tf::getYaw(state.pose.orientation);
  params_.xinit[k++] = state.twist.linear.x;
  params_.xinit[k++] = state.twist.linear.y;
  params_.xinit[k++] = state.twist.angular.z;
  params_.xinit[k++] = state.progress;
  ROS_ASSERT(k == Nstates_);
}

void ContouringControl::setAllParameters(const std::vector<geometry_msgs::Point>& waypoints,
                                         const nautonomous_mpc_msgs::CostsCC& costs,
                                         nautonomous_mpc_msgs::Obstacles obstacles) {
  // Check number of obstacles
  if (obstacles.obstacles.size() > Nobstacles_) {
    ROS_WARN("[%s] Too many obstacles! The current solver only supports up to %d obstacles and therefore only the first"
             "%d obstacles will be taken into account. Make sure they are sorted properly or consider generating a new solver.",
             ros::this_node::getName().c_str(), Nobstacles_, Nobstacles_);
  }

  // Set parameters
  int k = 0;
  int j_points, j_obstacles;
  for (int i = 0; i < N_; i++) {
    params_.all_parameters[k++] = boat_.m();
    params_.all_parameters[k++] = boat_.Iz();
    params_.all_parameters[k++] = boat_.l();
    params_.all_parameters[k++] = boat_.beta();
    params_.all_parameters[k++] = boat_.Dx();
    params_.all_parameters[k++] = boat_.Dy();
    params_.all_parameters[k++] = boat_.Dtheta();
    params_.all_parameters[k++] = controller_dt_;
    for (int j = 0; j < Npoints_; j++) {
      if (j < waypoints.size()) {
        j_points = j;
      }
      else {
        j_points = waypoints.size() - 1;
      }
      params_.all_parameters[k+j] = waypoints[j_points].x;
      params_.all_parameters[k+Npoints_+j] = waypoints[j_points].y;
    }
    k += Npoints_ * Npar_point_;
    params_.all_parameters[k++] = boat_.priorSegmentsLengths().front();
    params_.all_parameters[k++] = boat_.priorSegmentsLengths().back(); // path length
    params_.all_parameters[k++] = costs.q_progress;
    params_.all_parameters[k++] = costs.q_lag;
    params_.all_parameters[k++] = costs.q_contouring;
    params_.all_parameters[k++] = costs.q_u;
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

void ContouringControl::setX0(const nautonomous_state_msgs::BoatState& state) {
  int k = 0;
  for (int i = 0; i < N_; i++) {
    params_.x0[k++] = 0.0;
    params_.x0[k++] = 0.0;
    params_.x0[k++] = 0.0;
    params_.x0[k++] = state.pose.position.x;
    params_.x0[k++] = state.pose.position.y;
    params_.x0[k++] = tf::getYaw(state.pose.orientation);
    params_.x0[k++] = state.twist.linear.x;
    params_.x0[k++] = state.twist.linear.y;
    params_.x0[k++] = state.twist.angular.z;
    params_.x0[k++] = state.progress;
  }
  ROS_ASSERT(k == N_ * Nvar_);
}

void ContouringControl::setX0(const FORCESNLPsolver_output& output) {
  for (int i = 0; i < (N_-1) * Nvar_; i++) {
    params_.x0[i] = output.Z[i+Nvar_];
  }
  for (int i = (N_-1) * Nvar_; i < N_ * Nvar_; i++) {
    params_.x0[i] = output.Z[i];
  }
}

void ContouringControl::publishSolverOutput() {
  nautonomous_mpc_msgs::Output solver_output;
  nautonomous_mpc_msgs::StageVariable z;
  int k = 0;
  for (int i = 0; i < N_; i++) {
    z.T_l = output_.Z[k++];
    z.T_r = output_.Z[k++];
    z.v_cc = output_.Z[k++];
    z.x = output_.Z[k++];
    z.y = output_.Z[k++];
    z.theta = output_.Z[k++];
    z.u = output_.Z[k++];
    z.v = output_.Z[k++];
    z.omega = output_.Z[k++];
    z.p_cc = output_.Z[k++];
    solver_output.Z.push_back(z);
  }
  ROS_ASSERT(k == N_ * Nvar_);
  solver_output.exitflag = exitflag_;

  pub_solver_output_.publish(solver_output);
}

#endif /* INCLUDE_MPC_ROS_CONTOURING_CONTROL_H_ */
