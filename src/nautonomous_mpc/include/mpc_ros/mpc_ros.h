/*
 * Author: Antoine Seewer
 */

#ifndef INCLUDE_MPC_ROS_MPC_ROS_H_
#define INCLUDE_MPC_ROS_MPC_ROS_H_

#include <ros/ros.h>
#include <mpc_ros/boat.h>
#include <mpc_ros/mpc_ros_defines.h>
#include <mpc_ros/mpc_ros_utils.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <nautonomous_mpc_msgs/Output.h>
#include <tf/transform_datatypes.h>
#include "../forces/current_solver/FORCESNLPsolver/include/FORCESNLPsolver.h"

#ifdef __cplusplus
extern "C" {
#endif
extern void FORCESNLPsolver_casadi2forces(FORCESNLPsolver_FLOAT *x, FORCESNLPsolver_FLOAT *y,
                                          FORCESNLPsolver_FLOAT *l, FORCESNLPsolver_FLOAT *p,
                                          FORCESNLPsolver_FLOAT *f, FORCESNLPsolver_FLOAT *nabla_f,
                                          FORCESNLPsolver_FLOAT *c, FORCESNLPsolver_FLOAT *nabla_c,
                                          FORCESNLPsolver_FLOAT *h, FORCESNLPsolver_FLOAT *nabla_h,
                                          FORCESNLPsolver_FLOAT *H, int stage);
FORCESNLPsolver_ExtFunc pt2Function = &FORCESNLPsolver_casadi2forces;
#ifdef __cplusplus
} // extern "C"
#endif

class MpcRos {
 public:
  MpcRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
         const int& N, const int& Nvar, const int& Nstates, const int& Npar, const int& Nobstacles);
  virtual ~MpcRos() {}

  // Convenience method for loading the costs for the objective function
  virtual void loadCosts() = 0;

  // Convenience method for loading the ghost obstacle
  void loadGhostObstacle();

  // Convenience method for initializing the obstacles
  void initializeObstacles(const nautonomous_mpc_msgs::Obstacle& obstacle);

  // Convenience method for managing the waypoints
  virtual void manageWaypoints() = 0;

  // Convenience method for predicting the obstacles movement
  void predictObstaclesMovement(nautonomous_mpc_msgs::Obstacles& obstacles);

  // Convenience method for setting all required solver parameters
  virtual void setSolverParameters() = 0;

  // Convenience method for publishing the boat command
  void publishBoatCommand();

  // Convenience method for publishing the solver output
  virtual void publishSolverOutput() = 0;

  // Convenience method for printing the overall solver statistics
  void printStatistics();

  // Subscriber callbacks
  void cbBoatState(const nautonomous_state_msgs::BoatState& boat_state_msg);
  void cbObstacles(const nautonomous_mpc_msgs::Obstacles& obstacles_msg);

  // Timer callbacks
  void cbStep(const ros::TimerEvent&);

 protected:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber sub_boat_state_;
  ros::Subscriber sub_obstacles_;

  // Publishers
  ros::Publisher pub_boat_command_;
  ros::Publisher pub_solver_output_;

  // Timers
  ros::Timer timer_controller_;

  // Boat
  Boat boat_;

  // Obstacles
  nautonomous_mpc_msgs::Obstacles obstacles_;
  nautonomous_mpc_msgs::Obstacle ghost_obstacle_;

  // FORCES
  FORCESNLPsolver_params params_;
  FORCESNLPsolver_output output_;
  FORCESNLPsolver_info info_;
  FILE *fp_ = NULL;
//  FORCESNLPsolver_ExtFunc pt2Function_ = &FORCESNLPsolver_casadi2forces;
  int exitflag_;

  // Statistics
  int c_ = 0; // solver call counter
  int it_min_ = std::numeric_limits<int>::max();
  int it_max_ = 0;
  int it_sum_ = 0;
  double solvetime_min_ = std::numeric_limits<double>::infinity();
  double solvetime_max_ = 0.0;
  double solvetime_sum_ = 0.0;

  // Settings
  const int N_;
  const int Nvar_;
  const int Nstates_;
  const int Npar_;
  const int Nobstacles_;
  const int Npar_obstacle_;
  double controller_dt_;
  double goal_position_threshold_;
  double goal_velocity_threshold_;
  double obstacle_uncertainty_growth_;
  bool simulation_;
  bool statistics_;
  bool visualization_;
  bool verbose_;
};

MpcRos::MpcRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
               const int& N, const int& Nvar, const int& Nstates, const int& Npar, const int& Nobstacles)
    : nh_(nh),
      nh_private_(nh_private),
      boat_(Boat(nh_)),
      N_(N),
      Nvar_(Nvar),
      Nstates_(Nstates),
      Npar_(Npar),
      Nobstacles_(Nobstacles),
      Npar_obstacle_(5),
      exitflag_(-1000) {
  // Load parameters
  nh_private_.param("controller_dt", controller_dt_, 0.1);
  nh_private_.param("goal_position_threshold", goal_position_threshold_, 0.1);
  nh_private_.param("goal_velocity_threshold", goal_velocity_threshold_, 0.1);
  nh_private_.param("obstacle_uncertainty_growth", obstacle_uncertainty_growth_, 0.05);
  nh_private_.param("simulation", simulation_, true);
  nh_private_.param("statistics", statistics_, false);
  nh_private_.param("visualization", visualization_, true);
  nh_private_.param("verbose", verbose_, true);

  // Load ghost obstacle and initialize obstacles
  loadGhostObstacle();
  initializeObstacles(ghost_obstacle_);

  // Subscribers
  sub_boat_state_ = nh_.subscribe("/boat_simulator/boat_state", 1, &MpcRos::cbBoatState, this);
  sub_obstacles_ = nh_.subscribe("/obstacles_simulator/obstacles", 1, &MpcRos::cbObstacles, this);

  // Publishers
  pub_boat_command_ = nh_private_.advertise<nautonomous_state_msgs::BoatCommand>("boat_command", 1, true);
  pub_solver_output_ = nh_private_.advertise<nautonomous_mpc_msgs::Output>("solver_output", 1, true);

  // Timers
  timer_controller_ = nh_.createTimer(ros::Duration(controller_dt_), &MpcRos::cbStep, this);
}

void MpcRos::loadGhostObstacle() {
  nh_private_.param("ghost_obstacle/x", ghost_obstacle_.state.pose.position.x, -49.5);
  nh_private_.param("ghost_obstacle/y", ghost_obstacle_.state.pose.position.y, 49.5);
  ghost_obstacle_.state.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  ghost_obstacle_.major_semiaxis = 0.1; // should not be zero! ("/0")
  ghost_obstacle_.minor_semiaxis = 0.1; // should not be zero! ("/0")
}

void MpcRos::initializeObstacles(const nautonomous_mpc_msgs::Obstacle& obstacle) {
  for (int i = 0; i < Nobstacles_; i++) {
    obstacles_.obstacles.push_back(obstacle);
  }

  if (verbose_) {
    ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "] Obstacles initialized with ghost obstacle:\n"
                    << ghost_obstacle_);
  }
}

void MpcRos::predictObstaclesMovement(nautonomous_mpc_msgs::Obstacles& obstacles) {
  for (nautonomous_mpc_msgs::Obstacle& obstacle : obstacles.obstacles) {
    if (mpc_ros::norm(obstacle.state.twist.linear) != 0 || mpc_ros::norm(obstacle.state.twist.angular) != 0) { //TODO: yes or no?
      double theta = tf::getYaw(obstacle.state.pose.orientation);
      obstacle.state.pose.position.x += (obstacle.state.twist.linear.x * cos(theta) + obstacle.state.twist.linear.y * sin(theta))
          * controller_dt_;
      obstacle.state.pose.position.y += (obstacle.state.twist.linear.x * sin(theta) - obstacle.state.twist.linear.y * cos(theta))
          * controller_dt_;
      theta += obstacle.state.twist.angular.z * controller_dt_;
      obstacle.state.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      obstacle.major_semiaxis += obstacle_uncertainty_growth_; //TODO: make proportional to velocity? linear, angular (separately?)?
      obstacle.minor_semiaxis += obstacle_uncertainty_growth_;
    }
  }
}

void MpcRos::publishBoatCommand() {
  nautonomous_state_msgs::BoatCommand boat_command;
  boat_command.u_l = boat_.thrustToUnitVoltage(output_.Z[0]);
  boat_command.u_r = boat_.thrustToUnitVoltage(output_.Z[1]);

  pub_boat_command_.publish(boat_command);
}

void MpcRos::printStatistics() {
  ROS_INFO("[%s] Overall solver statistics:\n"
           "number of solver calls: %d\n"
           "iterations:\tmin: %d\t\tmax: %d\taverage: %f\n"
           "solvetimes:\tmin: %f\tmax: %f\taverage: %f",
           ros::this_node::getName().c_str(),
           c_, it_min_, it_max_, (double ) it_sum_ / c_, solvetime_min_, solvetime_max_, solvetime_sum_ / c_);
}

void MpcRos::cbBoatState(const nautonomous_state_msgs::BoatState& boat_state_msg) {
  boat_.updateState(boat_state_msg);
}

void MpcRos::cbObstacles(const nautonomous_mpc_msgs::Obstacles& obstacles_msg) {
  obstacles_ = obstacles_msg;

  // Use ghost obstacle in case there is no obstacle
  if (obstacles_.obstacles.empty()) {
    obstacles_.obstacles.push_back(ghost_obstacle_);
    if (verbose_) {
      ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "] No obstacle received. Using ghost obstacle:\n"
                      << ghost_obstacle_);
    }
  }

  // Enlarge obstacles by safety radius
  for (nautonomous_mpc_msgs::Obstacle& obstacle : obstacles_.obstacles) {
    obstacle.major_semiaxis += boat_.safetyRadius();
    obstacle.minor_semiaxis += boat_.safetyRadius();
  }
}

void MpcRos::cbStep(const ros::TimerEvent&) {
  // Manage waypoints
  manageWaypoints();

  if (!boat_.waypoints().empty()) { //TODO: better with timer_.start()/stop()??... in cbBoatState()?...
    // Initialize FORCES parameters
    setSolverParameters();

    // Call FORCES solver
    exitflag_ = FORCESNLPsolver_solve(&params_, &output_, &info_, fp_, pt2Function);

    if (exitflag_ == 1 || exitflag_ == 0) {
      if (exitflag_ == 1) {
        if (verbose_) {
          ROS_INFO("[%s] FORCES took %d iterations and %f seconds to solve the problem.",
                   ros::this_node::getName().c_str(), info_.it, info_.solvetime);
        }
        if (info_.solvetime > controller_dt_) {
          ROS_WARN("[%s] FORCES solvetime exceeded controller dt.", ros::this_node::getName().c_str());
        }
      }
      else {
        ROS_WARN("[%s] FORCES reached maximum number of iterations (%d) in %f seconds. Proceeding with best solution found"
                 "so far.", ros::this_node::getName().c_str(), info_.it, info_.solvetime);
      }

      // Publish boat command
      publishBoatCommand();

      // Publish solver output (if needed)
      if (simulation_ || visualization_) {
        publishSolverOutput();
      }

      // Statistics
      if (statistics_) {
        c_++;
        if (info_.it < it_min_) {
          it_min_ = info_.it;
        }
        if (info_.it > it_max_) {
          it_max_ = info_.it;
        }
        it_sum_ += info_.it;
        if (info_.solvetime < solvetime_min_) {
          solvetime_min_ = info_.solvetime;
        }
        if (info_.solvetime > solvetime_max_) {
          solvetime_max_ = info_.solvetime;
        }
        solvetime_sum_ += info_.solvetime;
      }
    }
    else {
      ROS_ERROR("[%s] Some problem in FORCES solver. Exitflag %d.", ros::this_node::getName().c_str(), exitflag_);
    }
  }
}

#endif /* INCLUDE_MPC_ROS_MPC_ROS_H_ */
