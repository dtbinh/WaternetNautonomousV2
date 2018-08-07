#include <ros/ros.h>
#include <cmath>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/Waypoint.h>
#include <nav_msgs/Path.h>
#include <nautonomous_planning_and_control_using_search/Quaternion_conversion.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher marker_pub;
ros::Publisher marker_pub_1;
ros::Publisher marker_pub_2;
ros::Publisher marker_pub_3;
ros::Publisher marker_pub_4;
ros::Publisher marker_pub_5;

ros::Subscriber obstacle_sub;
ros::Subscriber position_sub;
ros::Subscriber route_sub;
ros::Subscriber route2_sub;
ros::Subscriber goal_sub;
ros::Subscriber ref_sub;

nautonomous_mpc_msgs::Obstacles obstacles;
nav_msgs::Path route;

geometry_msgs::Point p;

visualization_msgs::Marker line_strip;
visualization_msgs::Marker point;
visualization_msgs::Marker obstacle_marker;
visualization_msgs::Marker route_marker;
visualization_msgs::Marker goal_marker;
visualization_msgs::MarkerArray obstacles_marker;

geometry_msgs::PoseStamped position_marker;

double ns = 0;

float Xobst, Yobst, THobst,Aobst,Bobst;

void obstacle_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles_marker.markers.clear();
	obstacles = *obstacle_msg;
	for (int i = 0; i < obstacles.obstacles.size(); i++)
	{
                std::cout << "Obstacle received at" << std::endl;
                std::cout << obstacles.obstacles[i] << std::endl;

                Xobst = obstacles.obstacles[i].pose.position.x + cos(obstacles.obstacles[i].pose.orientation.z) * obstacles.obstacles[i].twist.linear.x;
                Yobst = obstacles.obstacles[i].pose.position.y + sin(obstacles.obstacles[i].pose.orientation.z) * obstacles.obstacles[i].twist.linear.x;
                THobst = toEulerAngle(obstacles.obstacles[i].pose.orientation);
                Aobst = obstacles.obstacles[i].major_semiaxis;
                Bobst = obstacles.obstacles[i].minor_semiaxis;

                obstacle_marker.scale.x = Aobst;
                obstacle_marker.scale.y = Bobst;
                obstacle_marker.pose.position.x = Xobst;
                obstacle_marker.pose.position.y = Yobst;
                obstacle_marker.pose.orientation = toQuaternion(0, 0, THobst);
		obstacle_marker.ns = 65 + i;
		obstacles_marker.markers.push_back(obstacle_marker);

                /*obstacle_marker.scale.x = Aobst*4;
                obstacle_marker.scale.y = Bobst*2;
                obstacle_marker.pose.position.x = Xobst + 3 * cos(THobst) * Aobst;
                obstacle_marker.pose.position.y = Yobst + 3 * sin(THobst) * Aobst;
                obstacle_marker.pose.orientation = toQuaternion(0, 0, THobst);
		obstacle_marker.ns = 70 + i;
                obstacles_marker.markers.push_back(obstacle_marker);*/
	}
	marker_pub_2.publish(obstacles_marker);
}

void goal_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& position_msg)
{
	std::cout << "Pos found" << std::endl;

        goal_marker.scale.x = 5;
        goal_marker.scale.y = 5;
        goal_marker.pose.position.x = position_msg->x;
        goal_marker.pose.position.y = position_msg->y;
        goal_marker.ns = "Goal";

        marker_pub_5.publish(goal_marker);
}

void position_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& position_msg)
{
        std::cout << "Pos found" << std::endl;
        position_marker.pose.position.x = position_msg->x;
        position_marker.pose.position.y = position_msg->y;
        position_marker.pose.position.z = 1.0;
        position_marker.pose.orientation = toQuaternion(0.0, 0.0, position_msg->theta);

        marker_pub_1.publish(position_marker);

        p.x = position_msg->x;
        p.y = position_msg->y;

        line_strip.points.push_back(p);

        marker_pub.publish(line_strip);
}

void route_cb(const nav_msgs::Path::ConstPtr& route_msg)
{
	route = *route_msg;
	route_marker.points.clear();
        route_marker.color.b = 1.0;
        route_marker.color.g = 0.0;
	route_marker.color.r = 0.0;
	route_marker.color.a = 1.0;
	for (int i = 0; i < route.poses.size(); i++)
	{
		p.x = route.poses[i].pose.position.x;
      		p.y = route.poses[i].pose.position.y;
	
      		route_marker.points.push_back(p);

    		marker_pub_3.publish(route_marker);
	}
}

void non_smooth_route_cb(const nav_msgs::Path::ConstPtr& route_msg)
{
	route = *route_msg;
	route_marker.points.clear();
        route_marker.color.b = 0.0;
        route_marker.color.g = 1.0;
	route_marker.color.r = 0.0;
	route_marker.color.a = 1.0;
	for (int i = 0; i < route.poses.size(); i++)
	{
		p.x = route.poses[i].pose.position.x;
      		p.y = route.poses[i].pose.position.y;
	
      		route_marker.points.push_back(p);

    		marker_pub_4.publish(route_marker);
	}
}

void ref_cb(const nautonomous_mpc_msgs::Waypoint::ConstPtr& ref_msg)
{
        std::cout << "Pos found" << std::endl;

        goal_marker.scale.x = 5;
        goal_marker.scale.y = 5;
        goal_marker.pose.position.x = ref_msg->stage.x;
        goal_marker.pose.position.y = ref_msg->stage.y;
        goal_marker.ns = "Goal";

        marker_pub_5.publish(goal_marker);
}

int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "visualization");	
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

        marker_pub = 		nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_pub_1 = 		nh_private.advertise<geometry_msgs::PoseStamped>("Position_marker", 10);
	marker_pub_2 = 		nh_private.advertise<visualization_msgs::MarkerArray>("Obstacle_marker",10);
	marker_pub_3 = 		nh_private.advertise<visualization_msgs::Marker>("Route_marker",10);
	marker_pub_4 = 		nh_private.advertise<visualization_msgs::Marker>("Route_2_marker",10);
        marker_pub_5 = 		nh_private.advertise<visualization_msgs::Marker>("Goal_marker",10);

	obstacle_sub = 		nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/mission_coordinator/obstacles",1,obstacle_cb);
	position_sub = 		nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/MPC/next_state",1,position_cb);
        goal_sub = 		nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/goal_state",1,goal_cb);
        ref_sub = 		nh.subscribe<nautonomous_mpc_msgs::Waypoint>("/mission_coordinator/reference_state",1,ref_cb);
        route_sub = 		nh.subscribe<nav_msgs::Path>("/Local_planner/non_smooth_route",1,non_smooth_route_cb);
	route2_sub = 		nh.subscribe<nav_msgs::Path>("/Local_planner/route",1,route_cb);

        line_strip.header.frame_id = "/occupancy_grid";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.scale.x = 0.2;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;

	position_marker.header.frame_id = "/occupancy_grid";
	position_marker.header.stamp = ros::Time::now();

	obstacle_marker.header.frame_id = "/occupancy_grid";
	obstacle_marker.header.stamp = ros::Time::now();
	
	obstacle_marker.action = visualization_msgs::Marker::ADD;
	obstacle_marker.pose.orientation.w = 1.0;
	obstacle_marker.id = 0;
	obstacle_marker.type = visualization_msgs::Marker::CYLINDER;

	obstacle_marker.scale.z = 0.5;
        obstacle_marker.color.r = 0.0;
	obstacle_marker.color.b = 1.0;
	obstacle_marker.color.a = 1.0;

        goal_marker.header.frame_id = "/occupancy_grid";
        goal_marker.header.stamp = ros::Time::now();

        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.id = 0;
        goal_marker.type = visualization_msgs::Marker::CYLINDER;

        goal_marker.scale.z = 0.5;
        goal_marker.color.b = 1.0;
        goal_marker.color.a = 1.0;


	route_marker.header.frame_id = "/occupancy_grid";
	route_marker.header.stamp  = ros::Time::now();
	route_marker.ns = "route";
	route_marker.action = visualization_msgs::Marker::ADD;
	route_marker.pose.orientation.w = 1.0;
    	route_marker.id = 0;
	route_marker.type = visualization_msgs::Marker::LINE_STRIP;
	
	route_marker.scale.x = 1.0;

	ros::spin();
}

