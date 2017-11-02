#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <std_msgs/Int8.h>
#include <cmath>

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable reference_state;
nautonomous_mpc_msgs::StageVariable temp_state;
nautonomous_mpc_msgs::StageVariable starting_state;
nautonomous_mpc_msgs::StageVariable final_state;

nautonomous_mpc_msgs::Obstacle obstacle;

std_msgs::Int8 iter;

geometry_msgs::Point p;
geometry_msgs::Point waypoint;

ros::Subscriber position_sub;
ros::Subscriber next_waypoint_sub;
	
ros::Publisher ref_pub;
ros::Publisher pos_pub;
ros::Publisher marker_pub;
ros::Publisher obstacle_pub;
ros::Publisher start_pub;
ros::Publisher start_next_waypoint;

visualization_msgs::Marker line_strip;
visualization_msgs::Marker point;

bool first_step = true;
bool waypoint_received = false;
bool destination_reached = false;

void pos_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& twist_msg )
{
	current_state = *twist_msg;
	pos_pub.publish(current_state);
	std::cout << "Next step made" << std::endl;

      	p.x = current_state.x;
      	p.y = current_state.y;

      	line_strip.points.push_back(p);

    	marker_pub.publish(line_strip);
	marker_pub.publish(point);
}

void next_waypoint_cb( const geometry_msgs::Point::ConstPtr& waypoint_msg)
{
	waypoint = *waypoint_msg;
	std::cout << "Next waypoint received" << std::endl;
	waypoint_received = true;
}

int main(int argc, char **argv)
{
 
	ros::init(argc, argv, "mission_coordinator");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	point.header.frame_id = line_strip.header.frame_id = "/my_frame";
	point.header.stamp = line_strip.header.stamp = ros::Time::now();
	point.ns = line_strip.ns = "points_and_lines";
	point.action = line_strip.action = visualization_msgs::Marker::ADD;
	point.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    	point.id = 0;
	line_strip.id = 1;
    	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	point.type = visualization_msgs::Marker::POINTS;

	point.scale.x = 0.5;
	point.scale.y = 0.5;
	point.color.g = 1.0;
	point.color.a = 1.0;

	line_strip.scale.x = 0.1;
	line_strip.color.r = 1.0;
	line_strip.color.a = 1.0;


	position_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/Position",10,pos_cb);
	next_waypoint_sub = nh.subscribe<geometry_msgs::Point>("Route_generator/next_waypoint",10,next_waypoint_cb);
	
	ref_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("/Ref",10);
	pos_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("/GPS",10);
	marker_pub = nh_private.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
	obstacle_pub = nh_private.advertise<nautonomous_mpc_msgs::Obstacle>("/obstacle", 10);
	start_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("/start", 10);
	start_next_waypoint = nh_private.advertise<std_msgs::Int8>("/ask_for_next_waypoint", 10);

	ros::Rate loop_rate(10);
	
	/* Generate a route */
	ros::Rate poll_rate(100);
	while(obstacle_pub.getNumSubscribers() == 0){poll_rate.sleep();}
	while(start_pub.getNumSubscribers() == 0){poll_rate.sleep();}
	
	obstacle.major_semiaxis = 3;
	obstacle.minor_semiaxis = 5;

	obstacle_pub.publish(obstacle);
	
	starting_state.x = -20;
	starting_state.y = 0;

	start_pub.publish(starting_state);
	
	iter.data = 1;
		
	ros::Duration(5).sleep();
	
	start_next_waypoint.publish(iter);

	ros::Duration(5).sleep();

	/* Initial position and reference */	
	reference_state.x = waypoint.x;
	reference_state.y = waypoint.y;
	temp_state.x = starting_state.x;
	temp_state.y = starting_state.y;
	temp_state.theta = starting_state.theta;
	final_state.x = 20;
	final_state.y = 0;

	std::cout << "First waypoint: (" << reference_state.x << ", "<< reference_state.y << ")" << std::endl;

	p.x = reference_state.x;
      	p.y = reference_state.y;

      	point.points.push_back(p);

    	marker_pub.publish(point);

	p.x = reference_state.x;
      	p.y = reference_state.y;

      	point.points.push_back(p);

    	marker_pub.publish(point);

	while(ref_pub.getNumSubscribers() == 0){poll_rate.sleep();}
	while(pos_pub.getNumSubscribers() == 0){poll_rate.sleep();}

	std::cout << "Start program" << std::endl;
	int count = 0;
	while (ros::ok() && not(destination_reached))
	{
		if (first_step)
		{
			ref_pub.publish(reference_state);
			first_step = false;
			pos_pub.publish(temp_state);
			std::cout << "First step made" << std::endl;
		}
		else
		{
			if(((current_state.x - reference_state.x) * (current_state.x - reference_state.x) + (current_state.y - reference_state.y) * (current_state.y - reference_state.y) < 1))
			{
				iter.data = iter.data + 1;
				if (iter.data < 11)
				{
					start_next_waypoint.publish(iter);
					ros::Duration(1).sleep();
				}
				else if (iter.data == 11)
				{
					waypoint.x = final_state.x;
					waypoint.y = final_state.y;
				}
				else
				{
					std::cout << "Final destination reached" <<std::endl;
					destination_reached = true;
				}
				reference_state.x = waypoint.x;
				reference_state.y = waypoint.y;
				p.x = reference_state.x;
			      	p.y = reference_state.y;
				point.points.push_back(p);

				std::cout << "Next waypoint: (" << reference_state.x << ", "<< reference_state.y << ")" << std::endl;
				ref_pub.publish(reference_state);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Program ended" << std::endl;	
	return 0;
}

