
#include "ros/ros.h"
#include "vector"
#include "cmath"
#include "nautonomous_mpc_msgs/StageVariable.h"
#include "nautonomous_mpc_msgs/Route.h"
#include "nautonomous_mpc_msgs/Obstacle.h"
#include "nautonomous_mpc_msgs/Obstacles.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

int Stage = 1;
int Next_stage = 1;
int length;

float temp_x;
float temp_y;
float ellipse_value;

int m = 250;
int d11 = 130;
int d22 = 5280;
int d33 = 750;
double l = 0.5;
int Iz = 750;
int F1 = 175;
double F2 = 0.6;
double F3 = 0;
double F4 = 0;
int T1 = -150;
double T2 = 2.9;
double T3 = 0.1828;
double T4 = 27.1;
double FF1 = -0.0317;
double FF2 = 0.0525;
double FF3 = -0.0408;
double FF4 = 0;

bool Blocking_obstacle_found = false;
bool Obstacle_still_blocking = false;
bool Route_received = false;
bool Action_received = false;
bool Intermediate_point_reached = false;
bool Waypoint_reached = false;
bool Semi_waypoint_reached = false;

ros::Publisher current_state_pub;
ros::Publisher ref_pub;
ros::Publisher obstacle_pub;
ros::Publisher start_pub;
ros::Publisher marker_pub;
ros::Publisher marker_pub_2;
ros::Publisher action_pub;

ros::Subscriber next_state_sub;
ros::Subscriber waypoint_sub;
ros::Subscriber position_sub;
ros::Subscriber imu_sub;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable waypoint_state;
nautonomous_mpc_msgs::StageVariable starting_state;
nautonomous_mpc_msgs::StageVariable intermediate_state;
nautonomous_mpc_msgs::StageVariable received_state;

nautonomous_mpc_msgs::Obstacles obstacles;
nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacle closest_obstacle;

nautonomous_mpc_msgs::Route Intermediate_route;
nautonomous_mpc_msgs::Route Temp_route;

visualization_msgs::Marker line_strip;
visualization_msgs::Marker point;
visualization_msgs::Marker obstacle_marker;
visualization_msgs::MarkerArray obstacle_array;

nav_msgs::Odometry Position;

sensor_msgs::Imu Imu;

geometry_msgs::Point p;

geometry_msgs::Twist action;

std::vector<float> waypoint_x = {40, 60, 72, 40};
std::vector<float> waypoint_y = {60, 60, 80, 40};
int waypoint_stage = 0;
int intermediate_stage = 0;

void Initialization () // State 1
{
	current_state.x = 0;
	current_state.y = 0;

	obstacles.Nobstacles = 0;

	for (int i = 0; i < obstacles.Nobstacles; i++)
	{
		obstacle = obstacles.obstacles[i];
		obstacle_marker.scale.x = obstacle.major_semiaxis * 2;
		obstacle_marker.scale.y = obstacle.minor_semiaxis * 2;
		obstacle_marker.scale.z = 0.5;

		obstacle_marker.pose.position.x = obstacle.state.pose.position.x;
      		obstacle_marker.pose.position.y = obstacle.state.pose.position.y;
      		obstacle_marker.pose.position.z = -0.5;
		obstacle_marker.ns = i + 65;

		obstacle_array.markers.push_back(obstacle_marker);
	}
    	marker_pub_2.publish(obstacle_array);


	Next_stage = 2;
}

void Select_waypoint() // State 2
{
	starting_state = current_state;

	if (waypoint_stage == waypoint_x.size()){waypoint_stage = 0;}
	waypoint_state.x = waypoint_x[waypoint_stage];
	waypoint_state.y = waypoint_y[waypoint_stage];
	std::cout << "Waypoint: " << waypoint_state.x  <<","<< waypoint_state.y << std::endl;
	std::cout << "Current: " << current_state.x  <<","<< current_state.y << std::endl;
	Next_stage = 3;
}

void Determine_closest_blocking_obstacle() // State 3
{
	Blocking_obstacle_found = false;
	float dist = sqrt(pow(waypoint_state.x - starting_state.x,2) + pow(waypoint_state.y - starting_state.y,2));
	float theta = atan2(waypoint_state.y - starting_state.y,waypoint_state.x - starting_state.x);
	float max_dist = 1e10;
	
	for ( int j = 0; j < obstacles.Nobstacles; j++)
	{
		obstacle = obstacles.obstacles[j];
		std::cout << "Obstacle: " << obstacle << std::endl;
		temp_x = starting_state.x;
		temp_y = starting_state.y;
		for ( double i = 0; i < dist; i+=0.1)
		{
			temp_x+= 0.1 * cos(theta);
			temp_y+= 0.1 * sin(theta);
			ellipse_value = pow((obstacle.state.pose.position.x - temp_x)/obstacle.major_semiaxis,2) + pow((obstacle.state.pose.position.y - temp_y)/obstacle.minor_semiaxis,2);
			std::cout << "Ellipse value: " << ellipse_value << std::endl;
			if (ellipse_value < 1)
			{
				Blocking_obstacle_found = true;
				float dist_to_obstacle = i;
				if (dist_to_obstacle < max_dist)
				{
					max_dist = dist_to_obstacle;
					closest_obstacle = obstacle;
					std::cout << "Close obstacle found" << std::endl;
					i = dist;
				}
			}
		}
	}
	if (Blocking_obstacle_found){
		Next_stage = 4; 
		obstacle_pub.publish(closest_obstacle);
		ros::Duration(1).sleep();
	}
	else {
		Next_stage = 11;
		std::cout << "No obstacle found" <<std::endl;
	}
}

void Generate_route() // State 4
{
	Route_received = false;
	start_pub.publish(starting_state);
	Next_stage = 5;
}

void Wait_for_route() // State 5
{
	if (Route_received){Next_stage = 6;}
	else {Next_stage = 5;}
	Route_received = false;
}

int Determine_length_of_route() // State 6
{
	Obstacle_still_blocking = true;
	int i = 0;
	while  (Obstacle_still_blocking && i < Temp_route.waypoints.size())
	{
		Obstacle_still_blocking = false;
		temp_x = starting_state.x;
		temp_y = starting_state.y;
		float dist = sqrt(pow(waypoint_state.x - Temp_route.waypoints[i].x,2) + pow(waypoint_state.y - Temp_route.waypoints[i].y,2));
		float theta = atan2(waypoint_state.y - Temp_route.waypoints[i].y,waypoint_state.x - Temp_route.waypoints[i].x);
		for (double j = 0; j < dist; j+=0.1)
		{
			temp_x+= 0.1 * cos(theta);
			temp_y+= 0.1 * sin(theta);
			ellipse_value = pow((closest_obstacle.state.pose.position.x - temp_x)/closest_obstacle.major_semiaxis,2) + pow((closest_obstacle.state.pose.position.y - temp_y)/closest_obstacle.minor_semiaxis,2);
			if (ellipse_value < 1)
			{
				Obstacle_still_blocking = true;
				break;
			}
		}
		Intermediate_route.waypoints.push_back(Temp_route.waypoints[i]);
		i++;
	}
	Intermediate_route.waypoints.push_back(Temp_route.waypoints[i+1]);
	Intermediate_route.waypoints.push_back(Temp_route.waypoints[i+2]);
	Intermediate_route.waypoints.push_back(Temp_route.waypoints[i+3]);
	Intermediate_route.waypoints.push_back(Temp_route.waypoints[i+4]);
	Intermediate_route.waypoints.push_back(Temp_route.waypoints[i+5]);
	Next_stage = 7;
	std::cout << "Route: " << Intermediate_route << std::endl;
	length = intermediate_stage + i + 4; 
	intermediate_stage++;
}

void Initialize_MPC() // State 7
{
	Next_stage = 8;
	intermediate_stage+=5;
}

void Send_action_request() // State 8
{
	intermediate_state.x = Intermediate_route.waypoints[intermediate_stage].x;
	intermediate_state.y = Intermediate_route.waypoints[intermediate_stage].y;	
	std::cout << "Stage: " << intermediate_stage <<  " Reference: " << intermediate_state.x <<","<<intermediate_state.y << std::endl;
	std::cout << "Current: " << current_state.x <<","<<current_state.y << std::endl;
	ref_pub.publish(intermediate_state);
	current_state_pub.publish(current_state); 
	Next_stage = 9;
	
	p.x = intermediate_state.x;
      	p.y = intermediate_state.y;

      	point.points.push_back(p);

    	marker_pub.publish(point);
}

void Wait_for_action() // State 9
{
	if (Action_received){Next_stage = 10;}
	else {Next_stage = 9;}
	Action_received = false;
}

void Intermediate_point_reached_check() // State 10
{
	Waypoint_reached = false;
	Intermediate_point_reached = false;
	float dist_to_waypoint = sqrt(pow(waypoint_state.x - current_state.x,2) + pow(waypoint_state.y - current_state.y,2));
	float dist_to_intermediate = sqrt(pow(intermediate_state.x - current_state.x,2) + pow(intermediate_state.y - current_state.y,2));
	if (dist_to_waypoint < 5){Waypoint_reached = true;}
	if (dist_to_intermediate < 5){Intermediate_point_reached = true;}
	if (Waypoint_reached){
		Next_stage = 2;
		std::cout << "Waypoint reached" << std::endl;
		waypoint_stage++;
		intermediate_stage++;
	}
	else if (Intermediate_point_reached && Blocking_obstacle_found && intermediate_stage == length)
	{
		std::cout << "Intermediate final point reached" << std::endl;
		Next_stage = 2;
		intermediate_stage++;
	}
	else if(Intermediate_point_reached)
	{
		std::cout << "Intermediate point reached" << std::endl;
		Next_stage = 8;
		intermediate_stage++;
	}
	else
	{
		Next_stage = 8;
	}
}

void MPC_route_without_obstacle() // State 11
{
	float dist_to_waypoint = sqrt(pow(waypoint_state.x - current_state.x,2) + pow(waypoint_state.y - current_state.y,2));
	float theta = atan2(waypoint_state.y - current_state.y,waypoint_state.x - current_state.x);
	for (int i = 10; i < dist_to_waypoint; i+=10)
	{
		p.x = current_state.x + i * cos(theta);
		p.y = current_state.y + i * sin(theta);
		Intermediate_route.waypoints.push_back(p);
	}
	p.x = waypoint_state.x;
	p.y = waypoint_state.y;
	Intermediate_route.waypoints.push_back(p);
	std::cout << "Route: " << Intermediate_route << std::endl;
	Next_stage = 8;
}

void action_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& action_msg)
{
	received_state = *action_msg;
	float Torque = 	0.5 * (received_state.T_l - received_state.T_r);
	float Force = 	received_state.T_l + received_state.T_r;
	float uf = 1/F2 * (atanh((Force + F4)/F1) + F3);
	float FF = FF1 * pow(uf,3) + FF2 * pow(uf,2) + FF3 * uf + FF4;
	float ut = 1/T2 * (atanh((Torque + T4)/T1) + T3) + FF;

	std::cout << "Action signal is T: " << Torque << " F: " << Force << " uf: " << uf << " FF: " << FF << " ut: " << ut << std::endl;
	Action_received = true;

	if (uf >= 0)
	{
		action.linear.x = uf;
		action.angular.z = ut;
	}
	else
	{
		action.linear.x = 0;
		action.angular.z = 1;
	}
	action_pub.publish(action);
	
}

void route_cb(const nautonomous_mpc_msgs::Route::ConstPtr& route_msg)
{
	Route_received = true;
	Temp_route = *route_msg;
	std::cout << "Received route: " << Temp_route << std::endl;
}

void position_cb(const nav_msgs::Odometry::ConstPtr& pos_msg)
{
	Position = *pos_msg;
	current_state.x = Position.pose.pose.position.x;
	current_state.y = Position.pose.pose.position.y;

	p.x = current_state.x;
      	p.y = current_state.y;

      	line_strip.points.push_back(p);

    	marker_pub.publish(line_strip);
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	Imu = *imu_msg;
	float q0 = Imu.orientation.w;
	float q1 = Imu.orientation.x;
	float q2 = Imu.orientation.y;
	float q3 = Imu.orientation.z;

	current_state.theta = 1.57-atan2(2*(q0*q3+q1*q2),1-2*(pow(q2,2) + pow(q3,2)));
	
}


int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "mission_coordinator");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	

	current_state_pub = 	nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("current_state",10);
	ref_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("reference_state",10);
	obstacle_pub = 		nh_private.advertise<nautonomous_mpc_msgs::Obstacle>("obstacle",10);
	start_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("start",10);
	marker_pub = 		nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_pub_2 = 		nh_private.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 10);
	action_pub = 		nh_private.advertise<geometry_msgs::Twist>("/actuation/propulsion/mission_coordinator/cmd_vel", 10);

	next_state_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/MPC/next_state",10, action_cb);
	waypoint_sub = 		nh.subscribe<nautonomous_mpc_msgs::Route>("/Route_generator/waypoint_route", 10, route_cb);
	position_sub = 		nh.subscribe<nav_msgs::Odometry>("/state/odom/utm",10,position_cb);
	imu_sub = 		nh.subscribe<sensor_msgs::Imu>("/sensor/imu/imu",10,imu_cb);


	ros::Rate loop_rate(40);

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

	obstacle_marker.header.frame_id = "/my_frame";
	obstacle_marker.header.stamp = ros::Time::now();
	obstacle_marker.ns = "points_and_lines";
	obstacle_marker.action = visualization_msgs::Marker::ADD;
	obstacle_marker.pose.orientation.w = 1.0;
    	obstacle_marker.id = 0;
	obstacle_marker.type = visualization_msgs::Marker::CYLINDER;

	obstacle_marker.scale.z = 0.5;
	obstacle_marker.color.r = 1.0;
	obstacle_marker.color.b = 1.0;
	obstacle_marker.color.a = 1.0;

	ros::Duration(5).sleep();


	while (ros::ok())
	{	
		switch(Stage){
			case 1 :{	Initialization();
					break;	
				}
			case 2 :{	Select_waypoint();
					break;	
				}
			case 3 :{	Determine_closest_blocking_obstacle();
					break;	
				}
			case 4 :{	Generate_route();
					break;	
				}
			case 5 :{	Wait_for_route();
					break;	
				}
			case 6 :{	Determine_length_of_route();
					break;	
				}
			case 7 :{	Initialize_MPC();
					break;	
				}
			case 8 :{	Send_action_request();
					break;	
				}
			case 9 :{	Wait_for_action();
					break;	
				}
			case 10 :{	Intermediate_point_reached_check();
					break;	
				}
			case 11 :{	MPC_route_without_obstacle();
					break;
				}
			default :{
					break;	
				}
		}
		std::cout << "Stage: " << Stage << std::endl;
		Stage = Next_stage;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
