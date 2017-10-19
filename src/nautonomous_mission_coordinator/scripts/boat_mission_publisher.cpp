#include <ros/ros.h>
#include <mpc_ros_msgs/Output.h>
#include <nautonomous_state_msgs/boatState.h>

ros::Subscriber sub_solver_output_;
ros::Subscriber sub_odom_;

ros::Publisher pub_state_;

void cbSolverOutput(const mpc_ros_msgs::Output& output_msg);
{

	boat_state.progress = output_msg.Z[1].p_cc;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mission_Coordinator");
	ros::NodeHandle node_handle_;
	
	
	ros::spin();
}

