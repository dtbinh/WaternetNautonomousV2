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
#include <std_msgs/Float64.h>

int Stage = 1;
int Next_stage = 1;
int length;

float temp_x;
float temp_y;
float ellipse_value;

double ellipse_x1;
double ellipse_x2;

bool Blocking_obstacle_found = false;
bool Obstacle_still_blocking = false;
bool Route_received = false;
bool Action_received = false;
bool Intermediate_point_reached = false;
bool Waypoint_reached = false;
bool Semi_waypoint_reached = false;
bool Kalman_received = false;

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

double ref_velocity;
double safety_margin;

bool use_fuzzy;
int number_of_fuzzy_waypoints;

int maximum_accepted_waypoint_error;
int minimum_waypoint_error;
double decrease_waypoint_error;
double waypoint_error;

std_msgs::Float64 Uncertainty;

ros::Publisher current_state_pub;
ros::Publisher ref_pub;
ros::Publisher refs_pub;
ros::Publisher obstacle_pub;
ros::Publisher start_pub;
ros::Publisher ekf_pub;
ros::Publisher obstacles_pub;

ros::Subscriber next_state_sub;
ros::Subscriber EKF_sub;
ros::Subscriber waypoint_sub;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable mpc_state;
nautonomous_mpc_msgs::StageVariable starting_state;
nautonomous_mpc_msgs::StageVariable temp_state;

nautonomous_mpc_msgs::Waypoint intermediate_state;
nautonomous_mpc_msgs::Waypoint waypoint_state;
nautonomous_mpc_msgs::WaypointList multi_reference_states;

nautonomous_mpc_msgs::Obstacles obstacles;
nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacle closest_obstacle;

nautonomous_mpc_msgs::Route Intermediate_route;
nautonomous_mpc_msgs::Route Temp_route;

geometry_msgs::Point p;
geometry_msgs::Quaternion q;

std::vector<double> start_x, start_y, start_theta, u, v, omega, major_semiaxis, minor_semiaxis;

std::vector<float> waypoint_x;
std::vector<float> waypoint_y;
int waypoint_stage = 0;
int intermediate_stage = 0;

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
	current_state.x = 0;
	current_state.y = 0;

	obstacles.Nobstacles = start_x.size();

	for (int i = 0; i < obstacles.Nobstacles; i++)
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

	Next_stage = 2;
}

void Select_waypoint() // State 2
{
	starting_state = current_state;

	if (waypoint_stage == waypoint_x.size()){Next_stage = 14;}
	else {Next_stage = 3;}

	waypoint_state.stage.x = waypoint_x[waypoint_stage];
	waypoint_state.stage.y = waypoint_y[waypoint_stage];
	std::cout << "Waypoint: " << waypoint_state.stage.x  <<","<< waypoint_state.stage.y << std::endl;
	std::cout << "Current: " << current_state.x  <<","<< current_state.y << std::endl;
}

void Determine_closest_blocking_obstacle() // State 3
{
	waypoint_error = minimum_waypoint_error;
	Blocking_obstacle_found = false;
	float dist = sqrt(pow(waypoint_state.stage.x - starting_state.x,2) + pow(waypoint_state.stage.y - starting_state.y,2));
	float theta = atan2(waypoint_state.stage.y - starting_state.y,waypoint_state.stage.x - starting_state.x);
	float max_dist = 1e10;

	for ( int j = 0; j < obstacles.Nobstacles; j++)
	{
		obstacle = obstacles.obstacles[j];
		std::cout << "Obstacle: " << obstacle << std::endl;

		for ( double i = 0; i < dist; i+=0.1)
		{
			temp_x= current_state.x + i * cos(theta);
			temp_y= current_state.y + i * sin(theta);

			ellipse_x1 = obstacle.state.pose.position.x - temp_x;
			ellipse_x2 = obstacle.state.pose.position.y - temp_y;

			ellipse_value = pow((cos(obstacle.state.pose.orientation.z) * ellipse_x1 + sin(obstacle.state.pose.orientation.z) * ellipse_x2)/obstacle.major_semiaxis,2) + pow((-sin(obstacle.state.pose.orientation.z) * ellipse_x1 + cos(obstacle.state.pose.orientation.z) * ellipse_x2)/obstacle.minor_semiaxis,2);

			if (ellipse_value < 1)
			{
				std::cout << "Ellipse [" << ellipse_x1 << " , " << ellipse_x2 << "] value: " << ellipse_value << std::endl;
				Blocking_obstacle_found = true;
				float dist_to_obstacle = i;
				if (dist_to_obstacle < max_dist)
				{
					max_dist = dist_to_obstacle;
					closest_obstacle = obstacle;
					std::cout << "Close obstacle found" << std::endl;
					waypoint_error = maximum_accepted_waypoint_error;
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
		std::cout << "Closest obstacle: " << closest_obstacle << std::endl;
		std::cout << "State: " << Temp_route.waypoints[i] << std::endl;
		float dist = sqrt(pow(waypoint_state.stage.x - Temp_route.waypoints[i].x,2) + pow(waypoint_state.stage.y - Temp_route.waypoints[i].y,2));
		float theta = atan2(waypoint_state.stage.y - Temp_route.waypoints[i].y,waypoint_state.stage.x - Temp_route.waypoints[i].x);

		for (double j = 0; j < dist; j+=0.1)
		{
			temp_x= Temp_route.waypoints[i].x + j * cos(theta);
			temp_y= Temp_route.waypoints[i].y + j * sin(theta);
 			
			ellipse_x1 = closest_obstacle.state.pose.position.x - temp_x;
			ellipse_x2 = closest_obstacle.state.pose.position.y - temp_y;

			ellipse_value = pow((cos(closest_obstacle.state.pose.orientation.z) * ellipse_x1 + sin(closest_obstacle.state.pose.orientation.z) * ellipse_x2)/(closest_obstacle.major_semiaxis+safety_margin),2) + pow((-sin(closest_obstacle.state.pose.orientation.z) * ellipse_x1 + cos(closest_obstacle.state.pose.orientation.z) * ellipse_x2)/(closest_obstacle.minor_semiaxis+safety_margin),2);

			if (ellipse_value < 1)
			{
				std::cout << "Waypoint [" << i << "] -> ellipse value to closest obstacle: " << ellipse_value << std::endl;
				Obstacle_still_blocking = true;
				break;
			}
		}
		Intermediate_route.waypoints.push_back(Temp_route.waypoints[i]);
		i++;
	}
	Intermediate_route.waypoints.push_back(Temp_route.waypoints[i]);
	Next_stage = 7;
	std::cout << "Route: " << Intermediate_route << std::endl;
	length = intermediate_stage + i; 
	intermediate_stage++;
}

void Initialize_MPC() // State 7
{
	Next_stage = 8;
	intermediate_stage+=5;
}

void Send_action_request() // State 8
{
	intermediate_state.stage.x = Intermediate_route.waypoints[intermediate_stage].x;
	intermediate_state.stage.y = Intermediate_route.waypoints[intermediate_stage].y;	
	std::cout << "Intermediate stage: " << intermediate_stage <<  " Reference: " << intermediate_state.stage.x <<","<<intermediate_state.stage.y << std::endl;
	std::cout << "Current: " << current_state.x <<","<<current_state.y << std::endl;
	intermediate_state.stage.u = ref_velocity/3.6;

	if (not(use_fuzzy))
	{
		std::cout << "Don't use fuzzy" << std::endl;
		intermediate_state.Uncertainty.data = waypoint_error;
		ref_pub.publish(intermediate_state);
	}
	else 
	{
		std::cout << "Use fuzzy" << std::endl;
		multi_reference_states.stages.clear();
		multi_reference_states.Uncertainty.clear();
		for (int i = 0; i < fmin(5,Intermediate_route.waypoints.size() - intermediate_stage); i++)
		{
			std::cout << "Waypoint[" << i << "]" << std::endl;
			temp_state.x = Intermediate_route.waypoints[intermediate_stage+i].x;
			temp_state.y = Intermediate_route.waypoints[intermediate_stage+i].y;
			temp_state.u = ref_velocity/3.6;
			multi_reference_states.stages.push_back(temp_state);
			Uncertainty.data = fmax(waypoint_error - i * decrease_waypoint_error, 1);
			multi_reference_states.Uncertainty.push_back(Uncertainty);
		}
		refs_pub.publish(multi_reference_states);
	}
	current_state_pub.publish(current_state); 
	Next_stage = 9;
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
	float dist_to_waypoint = sqrt(pow(waypoint_state.stage.x - current_state.x,2) + pow(waypoint_state.stage.y - current_state.y,2));
	float dist_to_intermediate = sqrt(pow(intermediate_state.stage.x - current_state.x,2) + pow(intermediate_state.stage.y - current_state.y,2));
	if (dist_to_waypoint < waypoint_error){Waypoint_reached = true;}
	if (dist_to_intermediate < waypoint_error){Intermediate_point_reached = true;}
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
		Next_stage = 12;
		intermediate_stage++;
		waypoint_error = fmax(minimum_waypoint_error, waypoint_error-decrease_waypoint_error);
	}
	else
	{
		Next_stage = 12;
	}
}

void Send_Kalman_filter_request() // State 12
{
	ekf_pub.publish(current_state);
	Next_stage = 13;
}

void Wait_for_Kalman_filter() // State 13
{
	if (Kalman_received){Next_stage = 8;}
	else {Next_stage = 13;}
	Kalman_received = false;

}

void MPC_route_without_obstacle() // State 11
{
	float dist_to_waypoint = sqrt(pow(waypoint_state.stage.x - current_state.x,2) + pow(waypoint_state.stage.y - current_state.y,2));
	float theta = atan2(waypoint_state.stage.y - current_state.y,waypoint_state.stage.x - current_state.x);
	p.x = waypoint_state.stage.x;
	p.y = waypoint_state.stage.y;
	Intermediate_route.waypoints.push_back(p);
	std::cout << "Route: " << Intermediate_route << std::endl;
	Next_stage = 8;
}

void action_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& action_msg)
{
	mpc_state = *action_msg;
	float uf = mpc_state.T_l;
	float ut = mpc_state.T_r;
	float FF = FF1 * pow(uf,3) + FF2 * pow(uf,2) + FF3 * uf + FF4;
	float Force = F1 * tanh(F2 * uf - F3) - F4;
	float Torque = T1 * tanh(T2 * (ut - FF) - T3) - T4;
	current_state.T_l = 0.5 * Force + Torque;
	current_state.T_r = 0.5 * Force - Torque;
	Action_received = true;
}

void route_cb(const nautonomous_mpc_msgs::Route::ConstPtr& route_msg)
{
	Route_received = true;
	Temp_route = *route_msg;
	std::cout << "Received route: " << Temp_route <<std::endl;
}

void EKF_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& ekf_msg)
{
	Kalman_received = true;
	current_state = *ekf_msg;

	std::cout << "Current state: " << current_state <<std::endl;
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
	nh_private.getParam("number_of_fuzzy_waypoints", number_of_fuzzy_waypoints);
	nh_private.getParam("use_fuzzy", use_fuzzy);

	nh_private.getParam("waypoint_error/maximum_accepted_waypoint_error", maximum_accepted_waypoint_error);
	nh_private.getParam("waypoint_error/minimum_waypoint_error", minimum_waypoint_error);
	nh_private.getParam("waypoint_error/decrease_waypoint_error", decrease_waypoint_error);

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
	ref_pub = 		nh_private.advertise<nautonomous_mpc_msgs::Waypoint>("reference_state",10);
	refs_pub = 		nh_private.advertise<nautonomous_mpc_msgs::WaypointList>("reference_states",10);
	obstacle_pub = 		nh_private.advertise<nautonomous_mpc_msgs::Obstacle>("obstacle",10);
	obstacles_pub = 	nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",10);
	start_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("start",10);
	ekf_pub = 		nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("start_ekf",10);

	next_state_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/PID/next_state",10, action_cb);
	EKF_sub =	 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/Ekf/next_state",10, EKF_cb);
	waypoint_sub = 		nh.subscribe<nautonomous_mpc_msgs::Route>("/Route_generator/waypoint_route", 10, route_cb);

	ros::Rate loop_rate(200);

	
	ros::Duration(1).sleep();

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
			case 12 :{	Send_Kalman_filter_request();
					break;
				}
			case 13 :{	Wait_for_Kalman_filter();
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
