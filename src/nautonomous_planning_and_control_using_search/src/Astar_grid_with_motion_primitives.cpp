#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/WaypointList.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nautonomous_planning_and_control_using_search/node_grid.h>
#include <Eigen/Dense>

#define INF 1000000
#define PI 3.141592653589793238462643383279502884197169399375105820974
#define sq2 1.414213562373095048801688724209698078569671875376948073176
#define sq5 2.236067977499789696409173668731276235440618359611525724270
#define sq10 3.162277660168379331998893544432718533719555139325216826857

using namespace Eigen;

int i = 0;

VectorXd NodeNrMatrix(600000);

ros::Subscriber map_sub;
ros::Subscriber start_sub;
ros::Subscriber goal_sub;
ros::Subscriber obstacle_sub;

ros::Publisher map_pub;
ros::Publisher marker_pub;
ros::Publisher marker_2_pub;
ros::Publisher marker_3_pub;

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid weighted_map;
nav_msgs::OccupancyGrid temp_map;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable goal_state;
nautonomous_mpc_msgs::StageVariable waypoint;
nautonomous_mpc_msgs::WaypointList Route;
nautonomous_mpc_msgs::Obstacles Obstacles;

node* starting_node = new node();
node* current_node = new node();
node* new_node = new node();
node* final_node = new node();
node* min_node = new node();

float minCost = INF;
float minRow;
float temp_x;
float temp_y;
float temp_theta;
float temp_dist;
float current_x;
float current_y;

float step_size = 1;
int weighted_map_border = 10;

float cost_c;
float cost_i;

float map_width = 0;
float map_height = 0;
float map_center_x = 0;
float map_center_y = 0;
float resolution;

int checks = 0;
float check1 = 1;
float check2 = 0;
float check3 = 0;
float check4 = 0;
float check5 = 0;
float check6 = 0;
float check7 = 0;
int route_length = 0;

std::vector<node>* Network = new std::vector<node>();
std::vector<node>* OpenList = new std::vector<node>();
int max_open_list_size = 0;

geometry_msgs::Point p;
geometry_msgs::PoseStamped p1;
geometry_msgs::Point p2;


visualization_msgs::Marker line_list;
nav_msgs::Path route_list;
nav_msgs::Path flipped_route_list;

int next_node = 1;
int current_node_nr = 0;
int new_node_nr = 0;
int start_node_nr = 0;
int Nx = 0;
int Ny = 0;
float new_cost = 0;
float new_dist;
float map_weight;
float turning_penalty = 0.00;
int time_stamp = 0;

bool useplot = true;
bool final_path_point_found = false;
bool tile_is_occupied = false;
bool obstacle_is_blocking = false;

float Xobst = 0;
float Yobst = 0;
float Xcan = 0;
float Ycan = 0;

float obstacle_x = 0;
float obstacle_y = 0;
float obstacle_th = 0;
float obstacle_u = 0;
float obstacle_v = 0;
float obstacle_w = 0;
float obstacle_a = 1;
float obstacle_b = 1;

std::string inputString;

void smooth_path()
{
	route_list.poses.clear();

	p1.pose.position.x = flipped_route_list.poses[0].pose.position.x;
	p1.pose.position.y = flipped_route_list.poses[0].pose.position.y;
	route_list.poses.push_back(p1);

	final_path_point_found = false;
	int current_waypoint = 0;
	while (!final_path_point_found)
	{
		current_x = flipped_route_list.poses[current_waypoint].pose.position.x;
		current_y = flipped_route_list.poses[current_waypoint].pose.position.y;

		for (i = current_waypoint + 1; i < flipped_route_list.poses.size(); i++)
		{
			temp_theta = atan2(flipped_route_list.poses[i].pose.position.y - current_y, flipped_route_list.poses[i].pose.position.x - current_x);
			for (int j = 0; j < (5 * (i - current_waypoint)); j++)
			{
				temp_x = current_x + cos(temp_theta) * 0.2 * j;
				temp_y = current_y + sin(temp_theta) * 0.2 * j;

				obstacle_is_blocking = false;
				for (int k = 0; k < Obstacles.obstacles.size(); k++)
				{
					obstacle_x = Obstacles.obstacles[k].state.pose.position.x;
					obstacle_y = Obstacles.obstacles[k].state.pose.position.y;
					obstacle_th = Obstacles.obstacles[k].state.pose.position.z;
					obstacle_u = Obstacles.obstacles[k].state.twist.linear.x;
					obstacle_v = Obstacles.obstacles[k].state.twist.linear.y;
					obstacle_w = Obstacles.obstacles[k].state.twist.linear.z;
					obstacle_a = Obstacles.obstacles[k].major_semiaxis;
					obstacle_b = Obstacles.obstacles[k].minor_semiaxis;

					Xobst = obstacle_x + 2 * obstacle_a * cos(obstacle_th) + obstacle_b  * sin(obstacle_th) + (obstacle_u * cos(obstacle_th) + obstacle_v * sin(obstacle_th)) * (current_node->getTimeStamp() + 1);
					Yobst = obstacle_y + 2 * obstacle_a * sin(obstacle_th) - obstacle_b  * cos(obstacle_th) + (obstacle_u * sin(obstacle_th) - obstacle_v * cos(obstacle_th)) * (current_node->getTimeStamp() + 1);
					Xcan = (Xobst - temp_x) * cos(obstacle_th) + (Yobst - temp_y) * sin(obstacle_th);
					Ycan = -(Xobst - temp_x) * sin(obstacle_th) + (Yobst - temp_y) * cos(obstacle_th);

					if((pow(Xcan/(obstacle_a*3),6) + pow(Ycan/(obstacle_b*2),6)) < 1.5)
					{
						obstacle_is_blocking = true;
					}
				}
				if ((int)weighted_map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 10)
				{
					tile_is_occupied = true;
					break;
				}
				else if	(obstacle_is_blocking)
				{
					tile_is_occupied = true;
					break;
				}
			}
			if (tile_is_occupied)
			{
				break;
			}
		}
		temp_dist = sqrt(pow(current_x - flipped_route_list.poses[i-1].pose.position.x,2)  + pow(current_y - flipped_route_list.poses[i-1].pose.position.y,2));
		current_waypoint = i ;


		if (temp_dist < 0.1)
		{
			p1.pose.position.x = current_x;
			p1.pose.position.y = current_y;
			route_list.poses.push_back(p1);
		}
		else
		{
			for (i = 0; i < temp_dist; ++i)
			{ 
				p1.pose.position.x = current_x + cos(temp_theta) * i;
				p1.pose.position.y = current_y + sin(temp_theta) * i;
				route_list.poses.push_back(p1);
			}
		}
		tile_is_occupied = false;

		if ((flipped_route_list.poses.size()-1) <= current_waypoint)
		{
			final_path_point_found = true;
		}
		marker_3_pub.publish(route_list);
	}
}

void get_minimum_node()
{
	minCost = INF;

	ROS_DEBUG_STREAM( "OpenList size is: " << OpenList->size() );
	for ( i = 0; i < OpenList->size() ; i++)
	{
		ROS_DEBUG_STREAM( "OpenList cost is: " << OpenList->at(i).getTotalCost() );
		if (OpenList->at(i).getTotalCost() < minCost)
		{
			minCost = OpenList->at(i).getTotalCost();
			min_node = &OpenList->at(i);
			minRow = i;
		}
	}
}

void add_new_node()
{
	ROS_DEBUG_STREAM( "Temp_x: " << temp_x );
	ROS_DEBUG_STREAM( "Temp_y: " << temp_y );
	ROS_DEBUG_STREAM( "map_center_x: " << map_center_x );
	ROS_DEBUG_STREAM( "Nx: " << Nx );
	ROS_DEBUG_STREAM( "map_center_y: " << map_center_y );

	new_node_nr = (temp_x - map_center_x) + Nx * (temp_y - map_center_y) + Nx * Ny * 4 * temp_theta;

	ROS_DEBUG_STREAM( "new_node_nr: " << new_node_nr );

	obstacle_is_blocking = false;
	for (i = 0; i < Obstacles.obstacles.size(); i++)
	{
		obstacle_x = Obstacles.obstacles[i].state.pose.position.x;
		obstacle_y = Obstacles.obstacles[i].state.pose.position.y;
		obstacle_th = Obstacles.obstacles[i].state.pose.position.z;
		obstacle_u = Obstacles.obstacles[i].state.twist.linear.x;
		obstacle_v = Obstacles.obstacles[i].state.twist.linear.y;
		obstacle_w = Obstacles.obstacles[i].state.twist.linear.z;
		obstacle_a = Obstacles.obstacles[i].major_semiaxis;
		obstacle_b = Obstacles.obstacles[i].minor_semiaxis;

		Xobst = obstacle_x + 2 * obstacle_a * cos(obstacle_th) + obstacle_b  * sin(obstacle_th) + (obstacle_u * cos(obstacle_th) + obstacle_v * sin(obstacle_th)) * (current_node->getTimeStamp() + 1);
		Yobst = obstacle_y + 2 * obstacle_a * sin(obstacle_th) - obstacle_b  * cos(obstacle_th) + (obstacle_u * sin(obstacle_th) - obstacle_v * cos(obstacle_th)) * (current_node->getTimeStamp() + 1);
		Xcan = (Xobst - temp_x) * cos(obstacle_th) + (Yobst - temp_y) * sin(obstacle_th);
		Ycan = -(Xobst - temp_x) * sin(obstacle_th) + (Yobst - temp_y) * cos(obstacle_th);

		if((pow(Xcan/(obstacle_a*3),6) + pow(Ycan/(obstacle_b*2),6)) < 1.5)
		{
			obstacle_is_blocking = true;
			break;
		}
	}

	if (useplot)
	{
		p.x = current_node->getX();
		p.y = current_node->getY();
		line_list.points.push_back(p);
		p.x = temp_x;
		p.y = temp_y;
		line_list.points.push_back(p);
	}
	
	if((temp_x < -(map_width*resolution/2)) || (temp_x > (map_width*resolution/2)) || (temp_y < -(map_height*resolution/2)) || (temp_y > (map_height*resolution/2)))
	{	
		new_dist = INF;
	}
	else if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 90)
	{
		new_dist = INF;
	}	
	else if	(obstacle_is_blocking)
	{
		new_dist = INF;
	}		
	else
	{	
		new_dist = sqrt(pow(temp_x - final_node->getX(),2) + pow(temp_y - final_node->getY(),2));
	}

	new_cost += weighted_map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)]/weighted_map_border;

	if ((new_cost + new_dist) >= INF)
	{
		new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), new_node_nr, false, time_stamp);
	}
	else if (NodeNrMatrix(new_node_nr) >= INF)
	{
		new_node->initializeNode(temp_x, temp_y, temp_theta, new_dist, new_cost,current_node->getNode(), new_node_nr, false, time_stamp);
		Network->push_back(*new_node);
		current_node->addConnectedNode(next_node);
		OpenList->push_back(*new_node);
		NodeNrMatrix(new_node_nr) = next_node;
		next_node++;

	}
	else if ((new_dist + new_cost) < Network->at(NodeNrMatrix(new_node_nr)).getTotalCost())
	{
		new_node->initializeNode(temp_x, temp_y, temp_theta, new_dist, new_cost,current_node->getNode(), new_node_nr, false, time_stamp);
		Network->at(NodeNrMatrix(new_node_nr)) = *new_node;
		OpenList->push_back(*new_node);
	}

	ROS_DEBUG_STREAM( "Node " << new_node_nr << " is at [" << new_node->getX() << ", " << new_node->getY() << "] at a theta of " << temp_theta << " pi with a cost of " << new_node->getTotalCost() );

}

void calculate_route()
{
	// Clear and initialize
	double begin_check4 = ros::Time::now().toSec();	
	p.z = 0.5;

	ROS_INFO_STREAM( "//////////////////START NEW ROUTE//////////////////" );

	new_node_nr = (start_state.x - map_center_x) + Nx * (start_state.y - map_center_y);

	start_node_nr = new_node_nr;

	starting_node->initializeNode(start_state.x, start_state.y, start_state.theta, sqrt(pow(start_state.x - goal_state.x,2) + pow(start_state.y - goal_state.y,2)), 0.0, 0, new_node_nr, false, time_stamp);

	Network->push_back(*starting_node);

	current_node = starting_node;

	OpenList->push_back(*starting_node);
	NodeNrMatrix(new_node_nr) = 0.0;

	check4 += ros::Time::now().toSec() - begin_check4;

	ROS_INFO_STREAM( "//////////////////Generate//////////////////" );
	double begin_check5 = ros::Time::now().toSec();	
	while (current_node->getDistToFinish() > step_size )
	{
		double begin_check2 = ros::Time::now().toSec();	
	
		temp_theta = current_node->getTheta();
		if ( temp_theta == 0.0)
		{
			// Node right forward	
			temp_theta = 1.75;
			temp_x = current_node->getX() + 2;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 0.0;
			temp_x = current_node->getX() + 2;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.75;
			temp_x = current_node->getX() + 3;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node right forward	
			temp_theta = 0.0;
			temp_x = current_node->getX() + 3;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node forwards
		
			temp_theta = 0.0;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 0;
			new_cost = current_node->getCost() + 1;
	
			add_new_node();

			// Node left forward
		
			temp_theta = 0.25;
			temp_x = current_node->getX() + 2;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.0;
			temp_x = current_node->getX() + 2;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.25;
			temp_x = current_node->getX() + 3;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.0;
			temp_x = current_node->getX() + 3;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();
		}
		else if (temp_theta == 0.25)
		{
			// Node right forward	
			temp_theta = 0.0;
			temp_x = current_node->getX() + 2;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 0.25;
			temp_x = current_node->getX() + 2;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 0.0;
			temp_x = current_node->getX() + 3;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node right forward	
			temp_theta = 0.25;
			temp_x = current_node->getX() + 3;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node forwards
		
			temp_theta = 0.25;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq2;
	
			add_new_node();

			// Node left forward
		
			temp_theta = 0.25;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.5;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.25;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.5;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();
		}
		else if ( temp_theta == 0.5)
		{
			// Node right forward	
			temp_theta = 0.5;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 2;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 0.25;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 2;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 0.5;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 3;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node right forward	
			temp_theta = 0.25;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() + 3;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node forwards
		
			temp_theta = 0.5;
			temp_x = current_node->getX() + 0;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + 1;

			add_new_node();

			// Node left forward
		
			temp_theta = 0.5;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.75;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.5;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.75;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();
		}

		else if (temp_theta == 0.75)
		{
			// Node right forward	
			temp_theta = 1.0;
			temp_x = current_node->getX() - 2;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 0.75;
			temp_x = current_node->getX() - 2;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.0;
			temp_x = current_node->getX() - 3;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node right forward	
			temp_theta = 0.75;
			temp_x = current_node->getX() - 3;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node forwards
		
			temp_theta = 0.75;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq2;
	
			add_new_node();

			// Node left forward
		
			temp_theta = 0.5;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.75;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.5;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.75;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();
		}

		else if ( temp_theta == 1.0)
		{
			// Node right forward	
			temp_theta = 0.75;
			temp_x = current_node->getX() - 2;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.0;
			temp_x = current_node->getX() - 2;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 0.75;
			temp_x = current_node->getX() - 3;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node right forward	
			temp_theta = 1.0;
			temp_x = current_node->getX() - 3;
			temp_y = current_node->getY() + 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node forwards
		
			temp_theta = 1.0;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() + 0;
			new_cost = current_node->getCost() + 1;
	
			add_new_node();

			// Node left forward
		
			temp_theta = 1.25;
			temp_x = current_node->getX() - 2;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.0;
			temp_x = current_node->getX() - 2;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.25;
			temp_x = current_node->getX() - 3;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.0;
			temp_x = current_node->getX() - 3;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();
		}

		else if (temp_theta == 1.25)
		{
			// Node right forward	
			temp_theta = 1.0;
			temp_x = current_node->getX() - 2;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.25;
			temp_x = current_node->getX() - 2;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.0;
			temp_x = current_node->getX() - 3;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node right forward	
			temp_theta = 1.25;
			temp_x = current_node->getX() - 3;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node forwards
		
			temp_theta = 1.25;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq2;
	
			add_new_node();

			// Node left forward
		
			temp_theta = 1.25;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.5;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.25;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.5;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();
		}

		else if ( temp_theta == 1.5)
		{
			// Node right forward	
			temp_theta = 1.25;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 2;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.5;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 2;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.25;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 3;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node right forward	
			temp_theta = 1.5;
			temp_x = current_node->getX() - 1;
			temp_y = current_node->getY() - 3;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node forwards
		
			temp_theta = 1.5;
			temp_x = current_node->getX() + 0;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + 1;
	
			add_new_node();

			// Node left forward
		
			temp_theta = 1.75;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.5;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 2;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.75;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.5;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 3;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();
		}

		else if (temp_theta == 1.75)
		{
			// Node right forward	
			temp_theta = 1.5;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 2;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.75;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 2;
			new_cost = current_node->getCost() + sq5;

			add_new_node();

			// Node right forward	
			temp_theta = 1.5;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 3;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node right forward	
			temp_theta = 1.75;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 3;
			new_cost = current_node->getCost() + sq10;

			add_new_node();

			// Node forwards
		
			temp_theta = 1.75;
			temp_x = current_node->getX() + 1;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq2;
	
			add_new_node();

			// Node left forward
		
			temp_theta = 1.75;
			temp_x = current_node->getX() + 2;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.0;
			temp_x = current_node->getX() + 2;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq5;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 1.75;
			temp_x = current_node->getX() + 3;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();

			// Node left forward
		
			temp_theta = 0.0;
			temp_x = current_node->getX() + 3;
			temp_y = current_node->getY() - 1;
			new_cost = current_node->getCost() + sq10;
			
			add_new_node();
		}

		else
		{
			ROS_DEBUG_STREAM( "Angle: " << fmod(current_node->getTheta(),2) );
			break;
		}

		check2 += ros::Time::now().toSec() - begin_check2;
		ROS_DEBUG_STREAM( "Elapsed time of check 2 is: " << check2 );

		double begin_check3 = ros::Time::now().toSec();	

		max_open_list_size = fmax(max_open_list_size,OpenList->size());

		OpenList->erase(OpenList->begin() + minRow);
	
		ROS_DEBUG_STREAM( "Check minimum node" );

		get_minimum_node();

		ROS_DEBUG_STREAM( "Minimum node is: " << min_node->getNode());
		current_node = min_node;
		check1++;

		check3 += ros::Time::now().toSec() - begin_check3;
		ROS_DEBUG_STREAM( "Elapsed time of check 3 is: " << check3 );

		double begin_check6 = ros::Time::now().toSec();	
		if (useplot)
		{
			marker_pub.publish(line_list);		
		}
		check6 += ros::Time::now().toSec() - begin_check6;
		ROS_DEBUG_STREAM( "Elapsed time of check 6 is: " << check6 );

		/*ros::Duration(1).sleep();
		checks++;
		if(checks >= 100)
		{
			break;
		}*/		
	}

	ROS_INFO_STREAM( "Elapsed time of initialization: " << check4 );


	ROS_INFO_STREAM( "//////////////////Track back//////////////////" );

	p1.pose.position.z = 1.0;
	double begin_check7 = ros::Time::now().toSec();	

	p1.pose.position.x = current_node->getX();
	p1.pose.position.y = current_node->getY();
	route_list.poses.push_back(p1);

	while (not(current_node->getNode() == start_node_nr))
	{
		current_node = &Network->at(NodeNrMatrix(current_node->getPreviousNode()));
		p1.pose.position.x = current_node->getX();
		p1.pose.position.y = current_node->getY();
		route_list.poses.push_back(p1);
		route_length++;
	}

	std::cout << "//////////////////Flipping route//////////////////" <<std::endl;
	std::cout << "Size:" << route_list.poses.size() <<std::endl;

	for (i = 1; i <= route_list.poses.size(); i++)
	{
		flipped_route_list.poses.push_back(route_list.poses[route_list.poses.size() - i]);
	}
		
	std::cout << "//////////////////Final point//////////////////" <<std::endl;
	flipped_route_list.poses[0].pose.position.x = start_state.x;
	flipped_route_list.poses[0].pose.position.y = start_state.y;

	marker_2_pub.publish(flipped_route_list);

	smooth_path();

	check7 += ros::Time::now().toSec() - begin_check7;

	check5 += ros::Time::now().toSec() - begin_check5;
	ROS_INFO_STREAM( "Elapsed time of route calculation: " << check5 );
	ROS_INFO_STREAM( "Total time of minimum node calculation: " << check3 );
	ROS_INFO_STREAM( "Total time of marker publishing: " << check6 );
	ROS_INFO_STREAM( "Total time of node generation: " << check2 );
	ROS_INFO_STREAM( "Total time of backtracking: " << check7 );
	ROS_INFO_STREAM( "Network size: " << Network->size() );
	ROS_INFO_STREAM( "Number of checked nodes: " << check1 );
	ROS_INFO_STREAM( "Length of the route: " << route_length * step_size << "[m]" );
	ROS_INFO_STREAM( "Maximum size of the open list: " << max_open_list_size );
}

void generate_route()
{
	while (not(current_node->getNode() == 0))
	{
		waypoint.x = current_node->getX();
		waypoint.y = current_node->getY();
		Route.stages.push_back(waypoint);
		current_node = &Network->at(current_node->getPreviousNode());
	}
}

void start_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{

	start_state = *state_msg;
	ROS_DEBUG_STREAM( "Start: " << start_state );

	ROS_DEBUG_STREAM( "End: " << goal_state );

	double begin = ros::Time::now().toSec();
	
	calculate_route();

	double end = ros::Time::now().toSec();	
	ROS_INFO_STREAM( "Elapsed time is: " << end-begin );

	checks = 0;
	check1 = 1;
	check2 = 0;
	check3 = 0;
	check4 = 0;
	check5 = 0;
	check6 = 0;
	check7 = 0;
	route_length = 0;
	Network->clear();
	OpenList->clear();
	flipped_route_list.poses.clear();
	route_list.poses.clear();
	NodeNrMatrix = VectorXd::Ones(4000000) * INF;
	Network->reserve(100000);
	OpenList->reserve(100000);

	next_node = 1;
	current_node_nr = 0;
	new_node_nr = 0;
	start_node_nr = 0;
	new_cost = 0;
	checks = 0;
}

void goal_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	goal_state = *state_msg;
	final_node->initializeNode(goal_state.x, goal_state.y, goal_state.theta, 0.0, INF, 0, 0, false, time_stamp);
}

void make_weighted_map()
{
	weighted_map = map;
	for (i = 0; i < map_width; i++)
	{
		for (int j = 0; j < map_height; j++)
		{
			if ((int)map.data[j * map_width + i] > 90)
			{
				weighted_map.data[j * map_width + i] = 100;
			}
			else if ((int)map.data[j * map_width + i] < 0)
			{
				weighted_map.data[j * map_width + i] = 100;
			}
		}
	}

	for (i = 0; i < map_width; i++)
	{
		for (int j = 1; j < map_height-1; j++)
		{
			for (int k = -weighted_map_border; k <= weighted_map_border ; k++)
			{
				if (k == -weighted_map_border)
				{	
					map_weight = map.data[(j+k) * map_width + i];
				}
				else
				{
					map_weight += map.data[(j+k) * map_width + i];
				}
			}
			temp_map.data[j * map_width + i] = (int)(map_weight / (2 * weighted_map_border + 1));
		}
	}
		

	for (i = 1; i < map_width-1; i++)
	{
		for (int j = 0; j < map_height; j++)
		{
			for ( int k = -weighted_map_border; k <= weighted_map_border; k++)
			{
				if ( k == -weighted_map_border)
				{
					map_weight = temp_map.data[j * map_width + i + k];
				}
				else
				{
					map_weight += temp_map.data[j * map_width + i + k];
				}
			}
			weighted_map.data[j * map_width + i] = (int)(map_weight / (2 * weighted_map_border + 1));
		}
	}

	for (i = 0; i < map_width; i++)
	{
		for (int j = 0; j < map_height; j++)
		{
			if ((int)map.data[j * map_width + i] > 0)
			{
				weighted_map.data[j * map_width + i] = 100;
			}
			else if ((int)map.data[j * map_width + i] < 0)
			{
				weighted_map.data[j * map_width + i] = 100;
			}
		}
	}
	


	ROS_INFO_STREAM("Weighted map published");

	
	map_pub.publish(weighted_map);
}

void map_cb (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	ROS_DEBUG_STREAM( "map received" );
	map = *map_msg;
	temp_map = map;
	
	ROS_DEBUG_STREAM( "Data length is: " << map.data.size() );
	map_width = (float)map.info.width;
	map_height = (float)map.info.height;

	Nx = ceil(map_width / step_size);
	Ny = ceil(map_height / step_size);
 
	map_center_x = (float)map.info.origin.position.x;
	map_center_y = (float)map.info.origin.position.y;

	resolution = (float)map.info.resolution;
	ROS_DEBUG_STREAM( "Map center: " << map_center_x << ", " << map_center_y );
	ROS_DEBUG_STREAM( "Map size: " << map_width << " x " << map_height );

	make_weighted_map();
}

void obstacle_cb (const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacles_msg)
{
	Obstacles = *obstacles_msg;

	std::cout << "Obstacle received" << std::endl;
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Local_planner");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	map_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);
	start_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);
	goal_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/goal",10,goal_cb);
	obstacle_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/mission_coordinator/obstacles",10,obstacle_cb);

	map_pub = 	nh.advertise<nav_msgs::OccupancyGrid>("/map_tree_opt",10);
	marker_pub = 	nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_2_pub = 	nh_private.advertise<nav_msgs::Path>("non_smooth_route", 10);
	marker_3_pub = 	nh_private.advertise<nav_msgs::Path>("route", 10);

	Network->reserve(1000000);
	OpenList->reserve(1000000);

	p1.pose.position.z = 0;
	p1.pose.orientation.w = 1;

	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.1;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	route_list.header.frame_id = "/map";
	route_list.header.stamp = ros::Time::now();

	flipped_route_list.header.frame_id = "/map";
	flipped_route_list.header.stamp = ros::Time::now();

	Network->clear();
	OpenList->clear();
	NodeNrMatrix = VectorXd::Ones(4000000) * INF;
	next_node = 1;

	ros::spin();	
}
