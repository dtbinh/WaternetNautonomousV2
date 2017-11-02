#include <ros/ros.h>
#include <ros/console.h>
//PCL specific includes
#include <math.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist command;
ros::Publisher command_pub;
ros::Subscriber command_sub ;

void  command_cb(const geometry_msgs::Twist::ConstPtr& command_msg)
{
	command = *command_msg;
	//command_pub.publish(command);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "System_Identification");

	ros::NodeHandle n;

	command_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	command_sub = n.subscribe<geometry_msgs::Twist>("/command_vel",10,command_cb);

	ros::Rate loop_rate(3);
	int count = 0;

	while (ros::ok())
	{
		command_pub.publish(command);
		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	ros::spin();
	return 0;
}

