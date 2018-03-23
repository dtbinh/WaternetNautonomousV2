#include <ros/ros.h>
#include <cmath>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nav_msgs/Path.h>
#include <nautonomous_planning_and_control_using_search/Quaternion_conversion.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher marker_pub_1;
ros::Publisher marker_pub_2;

ros::Subscriber obstacle_sub;
ros::Subscriber position_sub;

nautonomous_mpc_msgs::Obstacles obstacles;
nav_msgs::Path route;

geometry_msgs::Point p;

visualization_msgs::Marker point;
visualization_msgs::Marker obstacle_marker;
visualization_msgs::MarkerArray obstacles_marker;

geometry_msgs::PoseStamped position_marker;

double ns = 0;

void obstacle_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles_marker.markers.clear();
	obstacles = *obstacle_msg;
	for (int i = 0; i < obstacles.obstacles.size(); i++)
	{
		std::cout << "Obstacle received" << std::endl;
		obstacle_marker.scale.x = obstacles.obstacles[i].major_semiaxis * 2;
		obstacle_marker.scale.y = obstacles.obstacles[i].minor_semiaxis * 2;
		obstacle_marker.pose.position.x = obstacles.obstacles[i].state.pose.position.x + cos(obstacles.obstacles[i].state.pose.orientation.z) * obstacles.obstacles[i].state.twist.linear.x;
		obstacle_marker.pose.position.y = obstacles.obstacles[i].state.pose.position.y + sin(obstacles.obstacles[i].state.pose.orientation.z) * obstacles.obstacles[i].state.twist.linear.x;
		obstacle_marker.pose.orientation = toQuaternion(0.0, 0.0, obstacles.obstacles[i].state.pose.orientation.z);
		obstacle_marker.ns = 65 + i;
		obstacles_marker.markers.push_back(obstacle_marker);
	}
	marker_pub_2.publish(obstacles_marker);
}

void position_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& position_msg)
{
	std::cout << "Pos found" << std::endl;
	position_marker.pose.position.x = position_msg->x;
	position_marker.pose.position.y = position_msg->y;
	position_marker.pose.position.z = 1.0;
	position_marker.pose.orientation = toQuaternion(0.0, 0.0, position_msg->theta);
	
	marker_pub_1.publish(position_marker);
}

int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "visualization");	
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	marker_pub_1 = 		nh_private.advertise<geometry_msgs::PoseStamped>("Position_marker", 10);
	marker_pub_2 = 		nh_private.advertise<visualization_msgs::MarkerArray>("Obstacle_marker",10);

	obstacle_sub = 		nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/mission_coordinator/obstacles",1,obstacle_cb);
	position_sub = 		nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/MPC/next_state",1,position_cb);

	position_marker.header.frame_id = "/map";
	position_marker.header.stamp = ros::Time::now();

	obstacle_marker.header.frame_id = "/map";
	obstacle_marker.header.stamp = ros::Time::now();
	
	obstacle_marker.action = visualization_msgs::Marker::ADD;
	obstacle_marker.pose.orientation.w = 1.0;
	obstacle_marker.id = 0;
	obstacle_marker.type = visualization_msgs::Marker::CYLINDER;

	obstacle_marker.scale.z = 0.5;
	obstacle_marker.color.r = 1.0;
	obstacle_marker.color.b = 1.0;
	obstacle_marker.color.a = 1.0;

	ros::spin();
}

