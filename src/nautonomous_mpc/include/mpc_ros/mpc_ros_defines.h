/*
 * Author: Antoine Seewer
 */

#ifndef INCLUDE_MPC_ROS_MPC_ROS_DEFINES_H_
#define INCLUDE_MPC_ROS_MPC_ROS_DEFINES_H_

namespace mpc_ros {

namespace method {
static constexpr int WAYPOINTS_NAVIGATION = 0;
static constexpr int CONTOURING_CONTROL = 1;
}

namespace borders_mode {
static constexpr int ELLIPSES = 0;
static constexpr int SHIFTED_ELLIPSES = 1;
static constexpr int LINEAR_CONSTRAINTS = 2;
}

} // namespace mpc_ros

#endif /* INCLUDE_MPC_ROS_MPC_ROS_DEFINES_H_ */
