#include "ros/ros.h"
#include <nautonomous_mpc_msgs/StageVariable.h>

ros::Publisher current_state_pub;
ros::Subscriber next_state_sub;

void action_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	ros::Duration(0.3).sleep();

	current_state_pub.publish(*state_msg);
}

int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "mission_coordinator");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	current_state_pub = 	nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("current_state",10);

	next_state_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/MPC/next_state",10, action_cb);
	
	ros::spin();
}

