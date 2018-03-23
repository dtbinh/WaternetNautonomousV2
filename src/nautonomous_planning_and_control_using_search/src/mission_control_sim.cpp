#include "ros/ros.h"
#include "vector"
#include "cmath"
#include "nautonomous_mpc_msgs/StageVariable.h"
#include "nautonomous_mpc_msgs/WaypointList.h"
#include "nautonomous_mpc_msgs/Waypoint.h"
#include "nautonomous_mpc_msgs/Route.h"
#include "nautonomous_mpc_msgs/Obstacle.h"
#include "nautonomous_mpc_msgs/Obstacles.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include <std_msgs/Float64.h>

float temp_x;
float temp_y;
float ellipse_value;

double ellipse_x1;
double ellipse_x2;

double ref_velocity;
double safety_margin;

ros::Publisher	current_state_pub;
ros::Publisher	start_pub;
ros::Publisher	goal_pub;
ros::Publisher	obstacle_pub;
ros::Publisher	obstacles_pub;
ros::Publisher	gazebo_pub;

ros::Subscriber	next_state_sub;
ros::Subscriber	route_sub;
ros::Subscriber	obstacles_sub;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable mpc_state;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable goal_state;
nautonomous_mpc_msgs::StageVariable temp_state;

nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacles obstacles;
nautonomous_mpc_msgs::Obstacle closest_obstacle;

nav_msgs::Path Full_path;

geometry_msgs::Point p;
geometry_msgs::Quaternion q;

std::vector<double> start_x, start_y, start_theta, u, v, omega, major_semiaxis, minor_semiaxis;

std::vector<float> waypoint_x;
std::vector<float> waypoint_y;

double Current_loop_time;
double Time_of_last_path_call;
double Time_of_last_mpc_call;
double Time_of_last_obstacle_call;

int waypoint_iterator = 0;

bool current_state_received = false;
bool path_received = false;

void toQuaternion(double pitch, double roll, double yaw)
{
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
}


void Initialization () // State 1
{
	ROS_INFO_STREAM("Start initialization, there are: " << start_x.size() << " obstacles" );

	current_state.x = waypoint_x[waypoint_iterator];
	current_state.y = waypoint_y[waypoint_iterator];

	for (int i = 0; i < start_x.size(); i++)
	{
		obstacle.major_semiaxis = major_semiaxis[i];
		obstacle.minor_semiaxis = minor_semiaxis[i];
		obstacle.state.pose.position.x = start_x[i];
		obstacle.state.pose.position.y = start_y[i];
		obstacle.state.twist.linear.x = u[i];
		obstacle.state.twist.linear.y = v[i];
		obstacle.state.pose.orientation.z = start_theta[i];
		obstacle.state.twist.angular.z = omega[i];

		obstacles.obstacles.push_back(obstacle);
	}

	obstacles_pub.publish(obstacles);

	start_state.x = waypoint_x[waypoint_iterator];
	start_state.y = waypoint_y[waypoint_iterator];
	goal_state.x = waypoint_x[waypoint_iterator + 1];
	goal_state.y = waypoint_y[waypoint_iterator + 1];

	gazebo_pub.publish(start_state);
}

void Call_Route_generator()
{
	goal_pub.publish(goal_state);
	ros::Duration(0.01).sleep();
	if(current_state_received)
	{
		start_pub.publish(current_state);
	}
	else
	{
		start_pub.publish(start_state);
	}
}

void Call_MPC_generator()
{
	if(path_received)
	{
		current_state_pub.publish(current_state);
	}
}

void action_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& action_msg)
{
	mpc_state = *action_msg;
	current_state = mpc_state;
	current_state_received = true;
}

void route_cb(const nav_msgs::Path::ConstPtr& route_msg)
{
	Full_path = *route_msg;
	std::cout << "Received route" <<std::endl;
	path_received = true;
}

void obstacle_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles = *obstacle_msg;
	obstacles_pub.publish(obstacles);
}

int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "mission_coordinator");	
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	ros::Duration(5).sleep();

	nh_private.getParam("reference/velocity", ref_velocity);
	nh_private.getParam("safety_margin", safety_margin);

	nh_private.getParam("waypoints_x", waypoint_x);
	ROS_ASSERT(waypoint_x.size() != 0);
	nh_private.getParam("waypoints_y", waypoint_y);
	ROS_ASSERT(waypoint_x.size() == waypoint_y.size());

	nh_private.getParam("obstacles/start_x", start_x);
	ROS_ASSERT(start_x.size() != 0);
	nh_private.getParam("obstacles/start_y", start_y);
	ROS_ASSERT(start_x.size() == start_y.size());
	nh_private.getParam("obstacles/start_theta", start_theta);
	ROS_ASSERT(start_x.size() == start_theta.size());
	nh_private.getParam("obstacles/u", u);
	ROS_ASSERT(start_x.size() == u.size());
	nh_private.getParam("obstacles/v", v);
	ROS_ASSERT(start_x.size() == v.size());
	nh_private.getParam("obstacles/omega", omega);
	ROS_ASSERT(start_x.size() == omega.size());
	nh_private.getParam("obstacles/major_semiaxis", major_semiaxis);
	ROS_ASSERT(start_x.size() == major_semiaxis.size());
	nh_private.getParam("obstacles/minor_semiaxis", minor_semiaxis);
	ROS_ASSERT(start_x.size() == minor_semiaxis.size());

	current_state_pub = 	nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("current_state",10);
	start_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("start_state",10);
	goal_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("goal_state",10);
	obstacle_pub = 		nh_private.advertise<nautonomous_mpc_msgs::Obstacle>("obstacle",10);
	obstacles_pub = 	nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",10);
	gazebo_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("set_gazebo_start",10);

	next_state_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/MPC/next_state",10, action_cb);
	route_sub = 		nh.subscribe<nav_msgs::Path>("/Local_planner/route", 10, route_cb);
	obstacles_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles", 10, obstacle_cb);

	ros::Rate loop_rate(100);

	ros::Duration(1).sleep();

	Initialization();

	while (ros::ok())
	{	
		Current_loop_time = ros::Time::now().toSec();
		if (Current_loop_time - Time_of_last_path_call > 2)
		{
			Time_of_last_path_call = Current_loop_time;
			Call_Route_generator();
		}		
		else if (Current_loop_time - Time_of_last_mpc_call > 1)
		{
			Time_of_last_mpc_call = Current_loop_time;
			Call_MPC_generator();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
