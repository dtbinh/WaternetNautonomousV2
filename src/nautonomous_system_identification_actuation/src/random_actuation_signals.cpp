#include <ros/ros.h>
#include <ros/console.h>
//PCL specific includes
#include <math.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

// Namespaces
using namespace std;

ros::Publisher vel_pub;

geometry_msgs::Twist msg;

// Publishers
ros::Publisher marker_pub;
ros::Publisher message_pub;

// Initialization
int timescale = 5;
int accscale = 10;
int startuptime = 120;

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Random_signal_actuation");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	ros::Rate cycle(3);

	double Begin = ros::Time::now().toSec();
	double Now = ros::Time::now().toSec();
	double Elapsed_time = Now-Begin;

	while (ros::ok())
	{
		// Parameters
		Now = ros::Time::now().toSec();
		Elapsed_time = Now-Begin;

		vel_pub = nh_private.advertise<geometry_msgs::Twist>("cmd_vel",10);
	
		Now = ros::Time::now().toSec();
		Elapsed_time = Now-Begin;

		cout << "Elapsed time is: " << Elapsed_time << endl;
		
		if (Elapsed_time < startuptime)
		{
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0;
		}
		else if (Elapsed_time < (startuptime + timescale * 1))
		{
			msg.linear.x = 0.1 * accscale;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0;
		}
		else if (Elapsed_time < (startuptime + timescale * 2))
		{
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0.1 * accscale;
		}
		else if (Elapsed_time < (startuptime + timescale * 3))
		{
			msg.linear.x = 0.1 * accscale;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0;
		}
		else if (Elapsed_time < (startuptime + timescale * 4))
		{
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0.1 * accscale;
		}
		else if (Elapsed_time < (startuptime + timescale * 5))
		{
			msg.linear.x = 0.1 * accscale;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0.1 * accscale;
		}
		else
		{
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0;
		}

		if (Elapsed_time > startuptime)
		{
			vel_pub.publish(msg);
		}
		
		cycle.sleep();
	}
}
