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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
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
ros::Publisher	ref_pub;
ros::Publisher	obstacle_pub;
ros::Publisher	obstacles_pub;
ros::Publisher	borders_pub;
ros::Publisher	action_pub;

ros::Subscriber	next_state_sub;
ros::Subscriber	route_sub;
ros::Subscriber	obstacles_sub;
ros::Subscriber	borders_sub;
ros::Subscriber	action_sub;
ros::Subscriber	new_goal_sub;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable goal_state;
nautonomous_mpc_msgs::StageVariable temp_state;

nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacles obstacles;
nautonomous_mpc_msgs::Obstacle closest_obstacle;
nautonomous_mpc_msgs::Obstacles borders;

nautonomous_mpc_msgs::Waypoint waypoint;

geometry_msgs::PoseWithCovarianceStamped pose_state;

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
int waypoint_iterator_2 = 0;

bool current_state_received = false;
bool path_received = false;
bool borders_received = false;

double goal_x;
double goal_y;

void Initialization()
{
    goal_state.x = goal_x;
    goal_state.y = goal_y;
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
}

void Call_PID_generator()
{    
	if(path_received)
        {
                while (sqrt(pow(current_state.x - waypoint.stage.x,2) + pow(current_state.y - waypoint.stage.y,2)) < 5)
                {
                    waypoint_iterator_2++;
                    waypoint.stage.x = Full_path.poses[waypoint_iterator_2].pose.position.x;
                    waypoint.stage.y = Full_path.poses[waypoint_iterator_2].pose.position.y;
                }
                ref_pub.publish(waypoint);
		current_state_pub.publish(current_state);
                ROS_INFO_STREAM("Call pid");
	}
}

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{

    ROS_INFO_STREAM("Pose received");

        pose_state = *pose_msg;
        current_state.x = (pose_state.pose.pose.position.x - transform_world_grid.getOrigin().x()) * cos(tf::getYaw(transform_world_grid.getRotation())) + (pose_state.pose.pose.position.y - transform_world_grid.getOrigin().y()) * sin(tf::getYaw(transform_world_grid.getRotation()));
        current_state.y = -(pose_state.pose.pose.position.x - transform_world_grid.getOrigin().x()) * sin(tf::getYaw(transform_world_grid.getRotation())) + (pose_state.pose.pose.position.y - transform_world_grid.getOrigin().y()) * cos(tf::getYaw(transform_world_grid.getRotation()));
        current_state.theta = toEulerAngle(pose_state.pose.pose.orientation) + angle_offset;

        current_state_received = true;

        Check_if_goal_is_close();
}

void route_cb(const nav_msgs::Path::ConstPtr& route_msg)
{
	Full_path = *route_msg;

	waypoint_iterator_2 = 1;
        waypoint.stage.x = Full_path.poses[waypoint_iterator_2].pose.position.x;
        waypoint.stage.y = Full_path.poses[waypoint_iterator_2].pose.position.y;
	std::cout << "Received route" <<std::endl;
	path_received = true;
}

void obstacle_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
        ROS_INFO_STREAM("Obstacles received");
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

void action_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& action_msg)
{
	action.linear.x = action_msg->T_l;
	action.angular.z = -1*action_msg->T_r;
	action_pub.publish(action);
}

void new_goal_cb(const geometry_msgs::PointStamped::ConstPtr& new_goal_msg)
{
    goal_x = new_goal_msg->point.x;
    goal_y = new_goal_msg->point.y;

    Call_Route_generator();
    Time_of_last_path_call = Current_loop_time;
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

        nh_private.getParam("goal_x", goal_x);
        nh_private.getParam("goal_y", goal_y);

	current_state_pub = 	nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("current_state",10);
	start_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("start_state",10);
	goal_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("goal_state",10);
        ref_pub = 		nh_private.advertise<nautonomous_mpc_msgs::Waypoint>("reference_state",10);
	obstacle_pub = 		nh_private.advertise<nautonomous_mpc_msgs::Obstacle>("obstacle",10);
	obstacles_pub = 	nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",10);
	borders_pub =	 	nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("borders",10);
	action_pub =	 	nh_private.advertise<geometry_msgs::Twist>("/cmd_vel",10);

        next_state_sub = 	nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/state/pose/robot_pose_ekf/odom_combined",10, pose_cb);
	route_sub = 		nh.subscribe<nav_msgs::Path>("/Local_planner/route", 10, route_cb);
	obstacles_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles", 10, obstacle_cb);
	borders_sub =	 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Map_modifier/borders", 10, borders_cb);
	action_sub =	 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/PID/next_state", 10, action_cb);
        new_goal_sub =          nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, new_goal_cb);

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
                        if (Current_loop_time - Time_of_last_path_call > 20)
			{
				Time_of_last_path_call = Current_loop_time;
				Call_Route_generator();
			}
			if (Current_loop_time - Time_of_last_mpc_call > 1)
			{
				Time_of_last_mpc_call = Current_loop_time;
                                Call_PID_generator();
			}		
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
        ROS_INFO_STREAM("Shizzle finished");
	return 0;
}

