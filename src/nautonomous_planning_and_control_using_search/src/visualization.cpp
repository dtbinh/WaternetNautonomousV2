#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nav_msgs/Path.h>
#include <nautonomous_planning_and_control_using_search/Quaternion_conversion.h>

ros::Publisher marker_pub_1;
ros::Publisher marker_pub_2;

ros::Subscriber obstacle_sub;
ros::Subscriber position_sub;

nautonomous_mpc_msgs::Obstacle obstacle;
nav_msgs::Path route;

geometry_msgs::Point p;

visualization_msgs::Marker point;
visualization_msgs::Marker obstacle_marker;

double ns = 0;

void obstacle_cb(const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg)
{
	obstacle = *obstacle_msg;
	obstacle_marker.scale.x = obstacle.major_semiaxis * 2;
	obstacle_marker.scale.y = obstacle.minor_semiaxis * 2;
	obstacle_marker.pose.position.x = obstacle.state.pose.position.x + obstacle.state.twist.linear.x;
	obstacle_marker.pose.position.y = obstacle.state.pose.position.y;
	marker_pub_2.publish(obstacle_marker);
}

void position_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& position_msg)
{
	std::cout << "Pos found" << std::endl;
	point.pose.position.x = position_msg->x;
	point.pose.position.y = position_msg->y;
	point.pose.position.z = 1.0;
	
	obstacle_marker.pose.position.x = obstacle_marker.pose.position.x + cos(obstacle.state.pose.position.z) * obstacle.state.twist.linear.x;
	obstacle_marker.pose.position.y = obstacle_marker.pose.position.y + sin(obstacle.state.pose.position.z) * obstacle.state.twist.linear.x;

	marker_pub_1.publish(point);
	marker_pub_2.publish(obstacle_marker);
}

int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "visualization");	
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	marker_pub_1 = 		nh_private.advertise<visualization_msgs::Marker>("Position_marker", 10);
	marker_pub_2 = 		nh_private.advertise<visualization_msgs::Marker>("Obstacle_marker",10);

	obstacle_sub = 		nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",1,obstacle_cb);
	position_sub = 		nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/MPC/next_state",1,position_cb);

	point.header.frame_id = "/map";
	point.header.stamp = ros::Time::now();
	point.ns = "waypoints";
	point.action = visualization_msgs::Marker::ADD;
	point.pose.orientation.w = 1.0;
    	point.id = 0;
	point.type = visualization_msgs::Marker::CYLINDER;

	point.scale.x = 1;
	point.scale.y = 1;
	point.scale.z = 1;
	point.color.g = 1.0;
	point.color.a = 1.0;

	obstacle_marker.header.frame_id = "/map";
	obstacle_marker.header.stamp = ros::Time::now();
	obstacle_marker.ns = "obstacle";
	
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

