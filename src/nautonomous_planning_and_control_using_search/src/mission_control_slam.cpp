#include "ros/ros.h"
#include "vector"
#include "cmath"
#include "nautonomous_mpc_msgs/StageVariable.h"
#include "nautonomous_mpc_msgs/WaypointList.h"
#include "nautonomous_mpc_msgs/Waypoint.h"
#include "nautonomous_mpc_msgs/Route.h"
#include "nautonomous_mpc_msgs/Obstacle.h"
#include "nautonomous_mpc_msgs/Obstacles.h"
#include "nautonomous_planning_and_control_using_search/Quaternion_conversion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

float temp_x;
float temp_y;
float ellipse_value;
float angle_offset;

double ellipse_x1;
double ellipse_x2;

double ref_velocity;
double safety_margin;

ros::Publisher	current_state_pub;
ros::Publisher	start_pub;
ros::Publisher	goal_pub;
ros::Publisher	obstacle_pub;
ros::Publisher	obstacles_pub;
ros::Publisher	borders_pub;
ros::Publisher	gazebo_pub;
ros::Publisher	ekf_pub;

ros::Subscriber	next_state_sub;
ros::Subscriber	ekf_sub;
ros::Subscriber	route_sub;
ros::Subscriber	obstacles_sub;
ros::Subscriber	borders_sub;
ros::Subscriber	action_sub;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable goal_state;
nautonomous_mpc_msgs::StageVariable temp_state;

nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacles obstacles;
nautonomous_mpc_msgs::Obstacle closest_obstacle;
nautonomous_mpc_msgs::Obstacles borders;

geometry_msgs::PoseStamped pose_state;

nav_msgs::Path Full_path;

geometry_msgs::Point p;
geometry_msgs::Quaternion q;

geometry_msgs::Twist action;

tf::StampedTransform transform_world_grid;
tf::StampedTransform transform_lidar_grid;

std::vector<double> start_x, start_y, start_theta, u, v, omega, major_semiaxis, minor_semiaxis;

std::vector<float> waypoint_x;
std::vector<float> waypoint_y;
std::vector<float> bridges;

double Current_loop_time;
double Time_of_last_path_call;
double Time_of_last_mpc_call;
double Time_of_last_obstacle_call;
double const_angle_diff = 0.5;

int waypoint_iterator = 0;

bool current_state_received = false;
bool path_received = false;
bool borders_received = false;

void Initialization () // State 1
{
	ROS_INFO_STREAM("Start initialization, there are: " << start_x.size() << " obstacles" );

	current_state.x = waypoint_x[waypoint_iterator];
	current_state.y = waypoint_y[waypoint_iterator];

	for (int i = 0; i < start_x.size(); i++)
	{
		obstacle.major_semiaxis = major_semiaxis[i];
		obstacle.minor_semiaxis = minor_semiaxis[i];
		obstacle.pose.position.x = start_x[i];
		obstacle.pose.position.y = start_y[i];
		obstacle.twist.linear.x = u[i];
		obstacle.twist.linear.y = v[i];
		obstacle.pose.orientation.z = start_theta[i];
		obstacle.twist.angular.z = omega[i];

		obstacles.obstacles.push_back(obstacle);
	}

	obstacles_pub.publish(obstacles);

	start_state.x = waypoint_x[waypoint_iterator];
	start_state.y = waypoint_y[waypoint_iterator];
	goal_state.x = waypoint_x[waypoint_iterator+1];
	goal_state.y = waypoint_y[waypoint_iterator+1];

	ROS_INFO_STREAM("Start state set to (" << current_state.x << ", " << current_state.y << ")" );
	ROS_INFO_STREAM("Goal state set to (" << goal_state.x << ", " << goal_state.y << ")" );

	gazebo_pub.publish(start_state);
}

void Check_if_goal_is_close()
{
	if ((sqrt(pow(current_state.x - goal_state.x,2) + pow(current_state.y - goal_state.y,2)) < 5) && borders_received)
	{
		waypoint_iterator++;
		goal_state.x = waypoint_x[waypoint_iterator+1];
		goal_state.y = waypoint_y[waypoint_iterator+1];
		ROS_INFO_STREAM("Set waypoint to: " << waypoint_iterator);
	}
}

void Call_Route_generator()
{
	goal_pub.publish(goal_state);
	ros::Duration(0.01).sleep();
	if(current_state_received)
	{
		start_pub.publish(current_state);
		ROS_INFO_STREAM("Call planner");
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
		ROS_INFO_STREAM("Call mpc");
	}
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
	nautonomous_mpc_msgs::StageVariable pre_ekf_state;

	pose_state = *pose_msg;	
	pre_ekf_state.x = (pose_state.pose.position.x - transform_world_grid.getOrigin().x()) * cos(tf::getYaw(transform_world_grid.getRotation())) + (pose_state.pose.position.y - transform_world_grid.getOrigin().y()) * sin(tf::getYaw(transform_world_grid.getRotation()));
	pre_ekf_state.y = -(pose_state.pose.position.x - transform_world_grid.getOrigin().x()) * sin(tf::getYaw(transform_world_grid.getRotation())) + (pose_state.pose.position.y - transform_world_grid.getOrigin().y()) * cos(tf::getYaw(transform_world_grid.getRotation()));
	pre_ekf_state.theta = toEulerAngle(pose_state.pose.orientation) + angle_offset;
	pre_ekf_state.T_r = (action.linear.x + action.angular.z)/2.0;
	pre_ekf_state.T_l = (action.linear.x - action.angular.z)/2.0;

	ekf_pub.publish(pre_ekf_state);
}

void ekf_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& pose_msg)
{
	current_state = *pose_msg;	
	
	current_state_received = true;

	Check_if_goal_is_close();
}

void route_cb(const nav_msgs::Path::ConstPtr& route_msg)
{
	Full_path = *route_msg;
	std::cout << "Received route" <<std::endl;
	path_received = true;
}

void obstacle_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles.obstacles.clear();
	for (int i = 0; i < obstacle_msg->obstacles.size(); i++)
	{
		/*obstacle.pose.position.x = (obstacle_msg->obstacles.at(i).pose.position.x - transform_lidar_grid.getOrigin().x()) * cos(tf::getYaw(transform_lidar_grid.getRotation())) + (obstacle_msg->obstacles.at(i).pose.position.y - transform_lidar_grid.getOrigin().y()) * sin(tf::getYaw(transform_lidar_grid.getRotation()));
		obstacle.pose.position.y = -(obstacle_msg->obstacles.at(i).pose.position.x - transform_lidar_grid.getOrigin().x()) * sin(tf::getYaw(transform_lidar_grid.getRotation())) + (obstacle_msg->obstacles.at(i).pose.position.y - transform_lidar_grid.getOrigin().y()) * cos(tf::getYaw(transform_lidar_grid.getRotation()));
		obstacle.pose.orientation = toQuaternion(0,0,toEulerAngle(obstacle_msg->obstacles.at(i).pose.orientation) - tf::getYaw(transform_lidar_grid.getRotation()));*/
		obstacle.pose.position.x = obstacle_msg->obstacles.at(i).pose.position.x;
		obstacle.pose.position.y = obstacle_msg->obstacles.at(i).pose.position.y;
		obstacle.pose.orientation = obstacle_msg->obstacles.at(i).pose.orientation;
		obstacle.minor_semiaxis = obstacle_msg->obstacles.at(i).minor_semiaxis;
		obstacle.major_semiaxis = obstacle_msg->obstacles.at(i).major_semiaxis;
		obstacles.obstacles.push_back(obstacle);
	}

	obstacles_pub.publish(obstacles);
}

void borders_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& border_msg)
{
	borders = *border_msg;
	ROS_INFO_STREAM("Borders received");
	borders_pub.publish(border_msg);
	
	Initialization();

	borders_received = true;

	ros::Duration(1).sleep();
}

void action_cb(const geometry_msgs::Twist::ConstPtr& action_msg)
{
	action = *action_msg;
}

int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "mission_coordinator");	
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");


	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 		
	{
   		ros::console::notifyLoggerLevelsChanged();
        }

	ROS_INFO_STREAM("Startup started");



	tf::TransformListener listener;

	ROS_INFO_STREAM("Transform listener setup");

	nh_private.getParam("reference/velocity", ref_velocity);
	nh_private.getParam("safety_margin", safety_margin);
	nh_private.getParam("angle_offset", angle_offset);

	nh_private.getParam("waypoints_x", waypoint_x);
	ROS_ASSERT(waypoint_x.size() != 0);
	nh_private.getParam("waypoints_y", waypoint_y);
	ROS_ASSERT(waypoint_x.size() == waypoint_y.size());
	nh_private.getParam("bridges", bridges);
	ROS_ASSERT(waypoint_x.size() == bridges.size());

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

	ROS_INFO_STREAM("Parameters loaded");

	current_state_pub = 	nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("current_state",10);
	start_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("start_state",10);
	goal_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("goal_state",10);
	obstacle_pub = 		nh_private.advertise<nautonomous_mpc_msgs::Obstacle>("obstacle",10);
	obstacles_pub = 	nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",10);
	borders_pub =	 	nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("borders",10);
	gazebo_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("set_gazebo_start",10);
	ekf_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("start_ekf",10);

	next_state_sub = 	nh.subscribe<geometry_msgs::PoseStamped>("/predict_pose",10, pose_cb);
	ekf_sub = 		nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/Ekf/next_state",10, ekf_cb);
	route_sub = 		nh.subscribe<nav_msgs::Path>("/Local_planner/route", 10, route_cb);
	obstacles_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles", 10, obstacle_cb);
	borders_sub =	 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Map_modifier/borders", 10, borders_cb);
	action_sub = 		nh.subscribe<geometry_msgs::Twist>("cmd_vel",10,action_cb);

	ros::Rate loop_rate(100);

	ros::Duration(1).sleep();

	ROS_INFO_STREAM("Startup finished");

	while (ros::ok())
	{	
		listener.lookupTransform("/map", "/occupancy_grid", ros::Time(0), transform_world_grid);
		listener.lookupTransform("/lidar_link", "/occupancy_grid", ros::Time(0), transform_lidar_grid);

		Current_loop_time = ros::Time::now().toSec();
		if(borders_received && current_state_received)
		{
			if (Current_loop_time - Time_of_last_path_call > 2)
			{
				Time_of_last_path_call = Current_loop_time;
				Call_Route_generator();
			}
			if (Current_loop_time - Time_of_last_mpc_call > 1)
			{
				Time_of_last_mpc_call = Current_loop_time;
				Call_MPC_generator();
			}		
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
