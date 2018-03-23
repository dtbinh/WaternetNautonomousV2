#include <iostream>
#include <boost/array.hpp>

#include <boost/numeric/odeint.hpp>

#include <ros/ros.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <gazebo_msgs/ModelState.h>
#include <cmath>

#include <Eigenvalues>

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include <nautonomous_planning_and_control/Quaternion_conversion.h>

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;

ros::Subscriber start_sub;
ros::Subscriber control_sub;

ros::Publisher state_pub;

double Current_loop_time = 0;
double Time_of_last_call = 0;

double D_x = 130;
double D_y = 5280;
double D_t = 750;
double m = 250;
double l = 0.5;
double I_z = 750;

double x = 0;
double y = 0;
double th = 0;
double u = 0;
double v = 0;
double w = 0;

double f1 = 0;
double f2 = 0;

int checks = 0;
nautonomous_mpc_msgs::StageVariable start_state;
gazebo_msgs::ModelState next_state;
gazebo_msgs::ModelState obstacle_state;

bool start_state_send = false;

void start_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& start_msg )
{
	f1 = start_msg->T_l;
	f2 = start_msg->T_r;
	next_state.pose.position.x  = start_msg->x;
	next_state.pose.position.y  = start_msg->y;
	th = start_msg->theta;
	u  = start_msg->u;
	v  = start_msg->v;
	w  = start_msg->omega;
	start_state_send = true;
	std::cout << "Start received" << std::endl;
}

void start_positioning()
{
	// Define initial values:
	// ----------------------	
	next_state.model_name = "mybot";
	next_state.reference_frame = "ground_plane";
	next_state.pose.orientation.w = 1.0;

	next_state.pose.position.x += 0.1 * (u * cos(th) - v * sin(th));
	next_state.pose.position.y += 0.1 * (u * sin(th) + v * cos(th));
	next_state.pose.orientation = toQuaternion(0, 0, th);

	state_pub.publish(next_state);
	checks++;
}

void set_obstacle_positions()
{
	obstacle_state.model_name = "bus";
	obstacle_state.reference_frame = "ground_plane";

	obstacle_state.pose.position.x += 0.1 * cos(2.98);
	obstacle_state.pose.position.y += 0.1 * sin(2.98);
	obstacle_state.pose.orientation = toQuaternion(0,0,4.55);

	state_pub.publish(obstacle_state);
}

void Initialize()
{
	obstacle_state.pose.position.x = 10;
	obstacle_state.pose.position.y = -5;
}

void set_control_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& pos_msg)
{
	f1 = pos_msg->T_l;
	f2 = pos_msg->T_r;
	next_state.pose.position.x  = pos_msg->x;
	next_state.pose.position.y  = pos_msg->y;
	th = pos_msg->theta;
	u  = pos_msg->u;
	v  = pos_msg->v;
	w  = pos_msg->omega;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv,"Gazebo_control");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	start_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/set_gazebo_start",10,start_cb);
	control_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/MPC/set_gazebo_control",10,set_control_cb);

	state_pub = nh_private.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10);

	ros::Rate loop_rate(100);

	Initialize();

	while (ros::ok())
	{
		Current_loop_time = ros::Time::now().toSec();
		if ( Current_loop_time - Time_of_last_call > 0.1 && start_state_send)
		{
			start_positioning();
			Time_of_last_call = Current_loop_time;
			set_obstacle_positions();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
