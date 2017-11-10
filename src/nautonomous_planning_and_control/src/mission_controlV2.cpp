#include "ros/ros.h"
#include "vector"
#include "cmath"
#include "nautonomous_mpc_msgs/StageVariable.h"
#include "nautonomous_mpc_msgs/Route.h"
#include "nautonomous_mpc_msgs/Obstacle.h"
#include "nautonomous_mpc_msgs/Obstacles.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

int Stage = 1;
int Next_stage = 1;
int length;

float temp_x;
float temp_y;
float ellipse_value;

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

ros::Subscriber next_state_sub;
ros::Subscriber waypoint_sub;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable waypoint_state;
nautonomous_mpc_msgs::StageVariable starting_state;
nautonomous_mpc_msgs::StageVariable intermediate_state;

nautonomous_mpc_msgs::Obstacles obstacles;
nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacle closest_obstacle;

nautonomous_mpc_msgs::Route Intermediate_route;

visualization_msgs::Marker line_strip;
visualization_msgs::Marker point;

geometry_msgs::Point p;

std::vector<float> waypoint_x = {40, 40,  0, 0};
std::vector<float> waypoint_y = { 0, 40, 40, 0};
int waypoint_stage = 0;
int intermediate_stage = 0;

void Initialization ()
{
	current_state.x = 0;
	current_state.y = 0;

	obstacle.major_semiaxis = 3;
	obstacle.minor_semiaxis = 5;
	obstacle.state.pose.position.x = 20;
	obstacle.state.pose.position.y = 0;

	obstacles.obstacles.push_back(obstacle);
	obstacles.Nobstacles = 1;

	Next_stage = 2;
}

void Select_waypoint()
{
	starting_state = current_state;

	if (waypoint_stage == 4){waypoint_stage = 0;}
	waypoint_state.x = waypoint_x[waypoint_stage];
	waypoint_state.y = waypoint_y[waypoint_stage];

	Next_stage = 3;
}

void Determine_closest_blocking_obstacle()
{
	Blocking_obstacle_found = false;
	float dist = sqrt(pow(waypoint_state.x - starting_state.x,2) + pow(waypoint_state.y - starting_state.y,2));
	float theta = atan2(waypoint_state.y - starting_state.y,waypoint_state.x - starting_state.x);
	float max_dist = 1e10;
	
	temp_x = starting_state.x;
	temp_y = starting_state.y;
	for ( int j = 0; j < obstacles.Nobstacles; j++)
	{
		obstacle = obstacles.obstacles[j];
		for ( double i = 0; i < dist; i+=0.1)
		{
			temp_x+= 0.1 * cos(theta);
			temp_y+= 0.1 * sin(theta);
			ellipse_value = pow((obstacle.state.pose.position.x - temp_x)/obstacle.major_semiaxis,2) + pow((obstacle.state.pose.position.y - temp_y)/obstacle.minor_semiaxis,2);
			if (ellipse_value < 1)
			{
				Blocking_obstacle_found = true;
				float dist_to_obstacle = i;
				if (dist_to_obstacle < max_dist)
				{
					max_dist = dist_to_obstacle;
					closest_obstacle = obstacle;
					std::cout << "Close obstacle found" << std::endl;
				}
			}
		}
	}
	if (Blocking_obstacle_found){
		Next_stage = 4; 
		obstacle_pub.publish(closest_obstacle);
	}
	else {Next_stage = 11;}
}

void Generate_route()
{
	Route_received = false;
	start_pub.publish(starting_state);
	Next_stage = 5;
}

void Wait_for_route()
{
	if (Route_received){Next_stage = 6;}
	else {Next_stage = 5;}
	Route_received = false;
}

int Determine_length_of_route()
{
	Obstacle_still_blocking = true;
	int i = 0;
	while  (Obstacle_still_blocking && i < Intermediate_route.waypoints.size())
	{
		Obstacle_still_blocking = false;
		temp_x = starting_state.x;
		temp_y = starting_state.y;
		float dist = sqrt(pow(waypoint_state.x - Intermediate_route.waypoints[i].x,2) + pow(waypoint_state.y - Intermediate_route.waypoints[i].y,2));
		float theta = atan2(waypoint_state.y - Intermediate_route.waypoints[i].y,waypoint_state.x - Intermediate_route.waypoints[i].x);
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
		i++;
	}
	Next_stage = 7;
	return i;
}

void Initialize_MPC()
{
	intermediate_stage = 0;
	Next_stage = 8;
}

void Send_action_request()
{
	intermediate_state.x = Intermediate_route.waypoints[intermediate_stage].x;
	intermediate_state.y = Intermediate_route.waypoints[intermediate_stage].y;
	ref_pub.publish(intermediate_state);
	current_state_pub.publish(current_state); 
	Next_stage = 9;
	
	p.x = intermediate_state.x;
      	p.y = intermediate_state.y;

      	point.points.push_back(p);

    	marker_pub.publish(point);
}

void Wait_for_action()
{
	if (Action_received){Next_stage = 10;}
	else {Next_stage = 9;}
	Action_received = false;
}

void Intermediate_point_reached_check()
{
	Waypoint_reached = false;
	Intermediate_point_reached = false;
	float dist_to_waypoint = sqrt(pow(waypoint_state.x - current_state.x,2) + pow(waypoint_state.y - current_state.y,2));
	float dist_to_intermediate = sqrt(pow(intermediate_state.x - current_state.x,2) + pow(intermediate_state.y - current_state.y,2));
	if (dist_to_waypoint < 1){Waypoint_reached = true;}
	if (dist_to_intermediate < 1){Intermediate_point_reached = true;}
	if (Waypoint_reached){Next_stage = 2;
		std::cout << "Waypoint reached" << std::endl;
		waypoint_stage++;
		intermediate_stage = 0;	
	}
	else if (Intermediate_point_reached && Blocking_obstacle_found){
		if(intermediate_stage == length){
			std::cout << "Intermediate final point reached" << std::endl;
			Next_stage = 2;
			intermediate_stage = 0;	
		}
		else{
			std::cout << "Intermediate point reached" << std::endl;
			Next_stage = 8;
			intermediate_stage++;
		}
	}
	else {Next_stage = 8;}
}

void MPC_route_without_obstacle()
{
	Intermediate_route.waypoints[0].x = waypoint_state.x;
	Intermediate_route.waypoints[0].y = waypoint_state.y;
	intermediate_stage = 0;
	Next_stage = 8;
}

void action_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& action_msg)
{
	current_state = *action_msg;
	Action_received = true;
	p.x = current_state.x;
      	p.y = current_state.y;

      	line_strip.points.push_back(p);

    	marker_pub.publish(line_strip);
}

void route_cb(const nautonomous_mpc_msgs::Route::ConstPtr& route_msg)
{
	Route_received = true;
	Intermediate_route = *route_msg;
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
	marker_pub = 		nh_private.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

	next_state_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/MPC/next_state",10, action_cb);
	waypoint_sub = 		nh.subscribe<nautonomous_mpc_msgs::Route>("/Route_generator/waypoint_route", 10, route_cb);

	ros::Rate loop_rate(100);

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
			case 6 :{	length = Determine_length_of_route();
					std::cout << "route length: " << length << std::endl;
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
