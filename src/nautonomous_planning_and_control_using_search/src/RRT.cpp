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
#include <nautonomous_planning_and_control_using_search/node_rrt.h>
#include <Eigen/Dense>

#define INF 1000000
#define PI 3.141592653589793238462643383279502884197169399375105820974
#define sq2 1.414213562373095048801688724209698078569671875376948073176
#define sq5 2.236067977499789696409173668731276235440618359611525724270
#define sq10 3.162277660168379331998893544432718533719555139325216826857
#define sq17 4.123105625617660549821409855974077025147199225373620434398

using namespace Eigen;

int i = 0;

ros::Subscriber map_sub;
ros::Subscriber start_sub;
ros::Subscriber goal_sub;
ros::Subscriber obstacle_sub;

ros::Publisher map_pub;
ros::Publisher marker_pub;
ros::Publisher marker_2_pub;
ros::Publisher marker_3_pub;
ros::Publisher	marker_4_pub;

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
node* rand_node = new node();

float minCost = INF;
int rand_it;
float temp_x;
float temp_y;
float temp_theta;
float temp_dist;
float current_x;
float current_y;
float rand_x;
float rand_y;

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
visualization_msgs::Marker marker;
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
bool closest_point_found = false;

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

void get_closest_node()
{
	float best_dist = INF;
	closest_point_found = false;
	for (int j = 0; j < Network->size(); j++)
	{
		temp_theta = atan2(rand_y - Network->at(j).getY(), rand_x - Network->at(j).getX()) - Network->at(j).getTheta();
		if ((temp_theta < 1 ) && (temp_theta > -1 ))
		{
			new_dist = sqrt(pow(Network->at(j).getX() - rand_x,2) + pow(Network->at(j).getY() - rand_y,2));
			if (new_dist < best_dist)
			{
				rand_node = &Network->at(j);
				best_dist = new_dist;
				closest_point_found = true;
			}
		}
	}
}

void add_new_node()
{
	temp_theta = atan2(rand_y - rand_node->getY(), rand_x - rand_node->getX());
	temp_x = rand_node->getX() + step_size*cos(temp_theta);
	temp_y = rand_node->getY() + step_size*sin(temp_theta);
	new_cost = rand_node->getCost() + step_size;

	ROS_DEBUG_STREAM( "Temp_x: " << temp_x );
	ROS_DEBUG_STREAM( "Temp_y: " << temp_y );
	ROS_DEBUG_STREAM( "map_center_x: " << map_center_x );
	ROS_DEBUG_STREAM( "Nx: " << Nx );
	ROS_DEBUG_STREAM( "map_center_y: " << map_center_y );

	if (useplot)
	{
		p.x = rand_node->getX();
		p.y = rand_node->getY();
		line_list.points.push_back(p);
		p.x = temp_x;
		p.y = temp_y;
		line_list.points.push_back(p);
	}

	if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 90)
	{

	}			
	else
	{	
		new_cost += weighted_map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)]/weighted_map_border;
		new_node->initializeNode(temp_x, temp_y, temp_theta, new_cost, rand_node->getNode(), Network->size(), false, time_stamp);
		Network->push_back(*new_node);
	}
	ROS_DEBUG_STREAM( "Node " << new_node_nr << " is at [" << new_node->getX() << ", " << new_node->getY() << "] at a theta of " << temp_theta << " with a cost of " << new_node->getCost() );

}

void calculate_route()
{
	 srand (time(NULL));

	// Clear and initialize
	double begin_check4 = ros::Time::now().toSec();	
	p.z = 0.5;

	ROS_INFO_STREAM( "//////////////////START NEW ROUTE//////////////////" );

	starting_node->initializeNode(start_state.x, start_state.y, start_state.theta, 0.0, 0, 0, true, time_stamp);

	Network->push_back(*starting_node);

	new_node->initializeNode(start_state.x + step_size * cos(start_state.theta), start_state.y + step_size * sin(start_state.theta), start_state.theta, 0.0, 0, 0, false, time_stamp);

	temp_x = start_state.x;
	temp_y = start_state.y;

	Network->push_back(*new_node);

	if (useplot)
	{
		p.x = starting_node->getX();
		p.y = starting_node->getY();
		line_list.points.push_back(p);
		p.x = new_node->getX();
		p.y = new_node->getY();
		line_list.points.push_back(p);
	}

	ROS_INFO_STREAM( "//////////////////Generate//////////////////" );
	ROS_DEBUG_STREAM( "Distance to finish is: " << (pow(current_node->getX() - final_node->getX(),2) + pow(current_node->getY() - final_node->getY(),2)));

	while ((pow(temp_x - final_node->getX(),2) + pow(temp_y - final_node->getY(),2)) > (step_size * step_size) && checks < 10000)
	{
		rand_x = round(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * map_width * resolution) + map_center_x;
		rand_y = round(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * map_height * resolution) + map_center_y;

		marker.pose.position.x = rand_x;
		marker.pose.position.y = rand_y;

		marker_4_pub.publish(marker);

		ROS_DEBUG_STREAM( "Rand_x: " << rand_x );
		ROS_DEBUG_STREAM( "Rand_y: " << rand_y );

		get_closest_node();

		if(closest_point_found)
		{
			add_new_node();
		}		

	
		if (useplot)
		{
			marker_pub.publish(line_list);		
		}
		ROS_DEBUG_STREAM( "Distance to finish is: " << (pow(current_node->getX() - final_node->getX(),2) + pow(current_node->getY() - final_node->getY(),2)));
		checks++;
	}

	ROS_INFO_STREAM( "Elapsed time of initialization: " << check4 );


	ROS_INFO_STREAM( "//////////////////Track back//////////////////" );

	current_node = new_node;

	p1.pose.position.z = 1.0;

	p1.pose.position.x = current_node->getX();
	p1.pose.position.y = current_node->getY();
	route_list.poses.push_back(p1);

	ROS_DEBUG_STREAM( "Network size: " << Network->size() );

	ROS_DEBUG_STREAM( "Current x: " << current_node->getX() );
	ROS_DEBUG_STREAM( "Current y: " << current_node->getY() );
	ROS_DEBUG_STREAM( "Previous node: " << current_node->getPreviousNode() );
	while (not(current_node->getNode() == 0))
	{
		current_node = &Network->at(current_node->getPreviousNode());
		ROS_DEBUG_STREAM( "Current x: " << current_node->getX() );
		ROS_DEBUG_STREAM( "Current y: " << current_node->getY() );
		ROS_DEBUG_STREAM( "Current node: " << current_node->getNode() );
		ROS_DEBUG_STREAM( "Previous node: " << current_node->getPreviousNode() );	
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

/*	smooth_path();

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
	ROS_INFO_STREAM( "Maximum size of the open list: " << max_open_list_size );*/
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
	line_list.points.clear();
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
	final_node->initializeNode(goal_state.x, goal_state.y, goal_state.theta, INF, 0, 0, false, time_stamp);
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


//	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
//	   ros::console::notifyLoggerLevelsChanged();
//	}


	map_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);
	start_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);
	goal_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/goal",10,goal_cb);
	obstacle_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/mission_coordinator/obstacles",10,obstacle_cb);

	map_pub = 	nh.advertise<nav_msgs::OccupancyGrid>("/map_tree_opt",10);
	marker_pub = 	nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_4_pub = 	nh_private.advertise<visualization_msgs::Marker>("visualization_marker_2", 10);
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
	line_list.scale.x = 0.05;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	route_list.header.frame_id = "/map";
	route_list.header.stamp = ros::Time::now();

	flipped_route_list.header.frame_id = "/map";
	flipped_route_list.header.stamp = ros::Time::now();

	Network->clear();
	OpenList->clear();
	next_node = 1;

	ros::spin();	
}
