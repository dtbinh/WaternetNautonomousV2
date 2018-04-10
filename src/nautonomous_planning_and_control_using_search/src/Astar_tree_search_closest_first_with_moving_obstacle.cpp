#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/WaypointList.h>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nautonomous_planning_and_control_using_search/node_tree.h>
#include <Eigen/Dense>

#define INF 1000000
#define PI 3.141592653589793238462643383279502884197169399375105820974

using namespace Eigen;

MatrixXd NodeNrMatrix(3000,3000);
MatrixXd CostMatrix(100000,3);
MatrixX2i PosMatrix(100000,2);

MatrixXf::Index minRow, minCol;

ros::Subscriber map_sub;
ros::Subscriber start_sub;
ros::Subscriber goal_sub;
ros::Subscriber obstacle_sub;

ros::Publisher map_pub;
ros::Publisher marker_pub;
ros::Publisher marker_2_pub;

int next_node = 1;

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid temp_map;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable goal_state;
nautonomous_mpc_msgs::StageVariable waypoint;
nautonomous_mpc_msgs::WaypointList Route;
nautonomous_mpc_msgs::Obstacle Obstacle;

node* starting_node = new node();
node* current_node = new node();
node* new_node = new node();
node* final_node = new node();

float minCost = INF;
float temp_x;
float temp_y;
float temp_theta;


int dx;
int dy;
int h;

float step_size = 1;
float grid_size = 0.05;
float angle_step = 0.05;

float cost_c;
float cost_i;

float map_width = 0;
float map_height = 0;
float map_center_x = 0;
float map_center_y = 0;
float resolution;

float check1 = 0;
float check2 = 0;
float check3 = 0;
float check4 = 0;
float check5 = 0;
float check6 = 0;
float checks = 0;

std::vector<node>* Network = new std::vector<node>();

geometry_msgs::Point p;
geometry_msgs::PoseStamped p2;

visualization_msgs::Marker line_list;
nav_msgs::Path route_list;
nav_msgs::Path flipped_route_list;

int current_node_nr = 0;
int start_node_nr = 0;
int Nx = 0;
int Ny = 0;
float new_cost = 0;
float new_dist = 0;

bool useplot = true;

float obstacle_x = 0;
float obstacle_y = 0;
float obstacle_th = 0;
float obstacle_u = 0;
float obstacle_v = 0;
float obstacle_w = 0;
float obstacle_a = 1;
float obstacle_b = 1;

float Xobst = 0;
float Yobst = 0;
float Xcan = 0;
float Ycan = 0;

void get_minimum_node()
{
	minRow = 0;
	minCost = INF;

	for ( int i = 0; i < next_node ; i++)
	{		
		if (CostMatrix(i,0) < minCost)
		{
			minCost = CostMatrix(i,0);
			minRow = i;
		}
	}
}

void add_new_node()
{
	PosMatrix(next_node,0) = round(temp_x);
	PosMatrix(next_node,1) = round(temp_y);
	

	Xobst = obstacle_x + 2 * obstacle_a * cos(obstacle_th) + obstacle_b  * sin(obstacle_th) + (obstacle_u * cos(obstacle_th) + obstacle_v * sin(obstacle_th)) * (current_node->getCost() + 1);
	Yobst = obstacle_y + 2 * obstacle_a * sin(obstacle_th) - obstacle_b  * cos(obstacle_th) + (obstacle_u * sin(obstacle_th) - obstacle_v * cos(obstacle_th)) * (current_node->getCost() + 1);
	Xcan = (Xobst - temp_x) * cos(obstacle_th) + (Yobst - temp_y) * sin(obstacle_th);
	Ycan = -(Xobst - temp_x) * sin(obstacle_th) + (Yobst - temp_y) * cos(obstacle_th);

	if((temp_x < 0) || (temp_x > (map_width*resolution)) || (temp_y < 0) || (temp_y > (map_height*resolution)))
	{	
		new_dist = INF;
	}
	else if((int)map.data[(round((temp_y)/resolution)-1) * map_width + round((temp_x)/resolution)] > 50)
	{
		new_dist = INF;
	}
	else if	((pow(Xcan/(obstacle_a*3),6) + pow(Ycan/(obstacle_b*2),6)) < 1.5)
	{
		new_dist = INF;
	}		
	else
	{	
		new_dist = sqrt(pow(temp_x - final_node->getX(),2) + pow(temp_y - final_node->getY(),2));
	}

	new_cost = current_node->getCost() + step_size;



	if ((new_cost + new_dist) >= INF)
	{
		ROS_DEBUG_STREAM( "New node cost is too high" );
	}
	else if (NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size)) >= INF)
	{
		new_node->initializeNode(temp_x, temp_y, temp_theta, new_dist, new_cost ,current_node->getNode(), next_node, false);
		current_node->addConnectedNode(next_node);
		CostMatrix(next_node,0) = new_node->getDistToFinish();
		CostMatrix(next_node,1) = round(temp_x);
		CostMatrix(next_node,2) = round(temp_y);
		NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size)) = next_node;
		next_node++;
		Network->push_back(*new_node);
		ROS_DEBUG_STREAM( "Node does not exist" );
		ROS_DEBUG_STREAM( "Node " << new_node->getNode() << " is at [" << new_node->getX() << ", " << new_node->getY() << "] at a theta of " << new_node->getTheta() << " with a cost of " << new_node->getCost() << " and a distTofinish of " << new_node->getDistToFinish() << " and node " << new_node->getPreviousNode() << " before");
		if (useplot)
		{
			p.x = current_node->getX() + map_center_x;
			p.y = current_node->getY() + map_center_y;
			line_list.points.push_back(p);
			p.x = temp_x + map_center_x;
			p.y = temp_y + map_center_y;
			line_list.points.push_back(p);
		}

	}
	else if ((new_cost + new_dist) < Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).getTotalCost())
	{
		Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).setCost(new_cost,current_node->getNode());;
		CostMatrix(next_node,0) = new_node->getDistToFinish();
		CostMatrix(next_node,1) = round(temp_x);
		CostMatrix(next_node,2) = round(temp_y);
		ROS_DEBUG_STREAM( "Node already exists and is smaller" );
		ROS_DEBUG_STREAM( "Node " << Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).getNode() << " is at [" << Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).getX() << ", " << Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).getY() << "] at a theta of " << Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).getTheta() << " with a cost of " << Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).getCost() << " and a distTofinish of " << Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).getDistToFinish() << " and node " << Network->at(NodeNrMatrix(round(temp_x / grid_size),round(temp_y / grid_size))).getPreviousNode() << " before");
	}
	else
	{
		ROS_DEBUG_STREAM( "New node cost is higher than existing node" );
	}

}

void calculate_route()
{
	// Clear and initialize
	double begin_check4 = ros::Time::now().toSec();	
	p.z = 0.5;

	std::cout << "//////////////////START NEW ROUTE//////////////////" <<std::endl;

	starting_node->initializeNode(start_state.x - map_center_x, start_state.y - map_center_y, start_state.theta, sqrt(pow(start_state.x - map_center_x - final_node->getX(),2) + pow(start_state.y - map_center_y - final_node->getY(),2)), 0.0, 0, 0, false);
	

	Network->push_back(*starting_node);

	current_node = starting_node;

	ROS_INFO_STREAM("Finish is at [" << final_node->getX() << ", " << final_node->getY() << "]" );
	ROS_INFO_STREAM("Starting node is at [" << start_state.x - map_center_x << ", " << start_state.y - map_center_y << "] at a theta of " << start_state.theta << " with a cost of " << starting_node->getCost() << " and a distance to the finish of " << starting_node->getDistToFinish() );

	ROS_DEBUG_STREAM( NodeNrMatrix.cols() );
	ROS_DEBUG_STREAM( NodeNrMatrix.rows() );
	
	CostMatrix(0,0) = 0.0;
	CostMatrix(0,1) = round(start_state.x - map_center_x);
	CostMatrix(0,2) = round(start_state.y - map_center_y);
	ROS_DEBUG_STREAM( round((start_state.x - map_center_x) / grid_size) );
	ROS_DEBUG_STREAM( round((start_state.y - map_center_y) / grid_size) );

	NodeNrMatrix(round((start_state.x - map_center_x) / grid_size),round((start_state.y - map_center_y) / grid_size)) = 0.0;
	PosMatrix(0,0) = round(start_state.x);
	PosMatrix(0,1) = round(start_state.y);

	check4 += ros::Time::now().toSec() - begin_check4;

	std::cout << "//////////////////Generate//////////////////" <<std::endl;
	double begin_check5 = ros::Time::now().toSec();	
	while (current_node->getDistToFinish() > step_size )
	{
		double begin_check2 = ros::Time::now().toSec();	
	
		// Node right forward	
		temp_theta = current_node->getTheta() - angle_step * PI;
		temp_x = current_node->getX() + step_size * cos(temp_theta);
		temp_y = current_node->getY() + step_size * sin(temp_theta);
		new_cost = current_node->getCost() + step_size;
		ROS_DEBUG_STREAM( "Current node " << current_node->getNode() << " is at [" << current_node->getX() << ", " << current_node->getY() << "]" );

		add_new_node();

		// Node slightly right forward	
		temp_theta = current_node->getTheta() - angle_step / 2 * PI;
		temp_x = current_node->getX() + step_size * cos(temp_theta);
		temp_y = current_node->getY() + step_size * sin(temp_theta);
		new_cost = current_node->getCost() + step_size;
		ROS_DEBUG_STREAM( "Current node " << current_node->getNode() << " is at [" << current_node->getX() << ", " << current_node->getY() << "]" );

		add_new_node();

		// Node forwards
		temp_theta = current_node->getTheta();
		temp_x = current_node->getX() + step_size * cos(temp_theta);
		temp_y = current_node->getY() + step_size * sin(temp_theta);
		new_cost = current_node->getCost() + step_size;
		ROS_DEBUG_STREAM( "Current node " << current_node->getNode() << " is at [" << current_node->getX() << ", " << current_node->getY() << "]" );

		add_new_node();

		// Node left forward
		temp_theta = current_node->getTheta() +  angle_step * PI;
		temp_x = current_node->getX() + step_size * cos(temp_theta);
		temp_y = current_node->getY() + step_size * sin(temp_theta);
		new_cost = current_node->getCost() + step_size;
		ROS_DEBUG_STREAM( "Current node " << current_node->getNode() << " is at [" << current_node->getX() << ", " << current_node->getY() << "]" );

		add_new_node();

		// Node slightly left forward
		temp_theta = current_node->getTheta() +  angle_step / 2 * PI;
		temp_x = current_node->getX() + step_size * cos(temp_theta);
		temp_y = current_node->getY() + step_size * sin(temp_theta);
		new_cost = current_node->getCost() + step_size;
		ROS_DEBUG_STREAM( "Current node " << current_node->getNode() << " is at [" << current_node->getX() << ", " << current_node->getY() << "]" );

		add_new_node();

		check2 += ros::Time::now().toSec() - begin_check2;
		ROS_DEBUG_STREAM( "Elapsed time of check 2 is: " << check2 );

		double begin_check3 = ros::Time::now().toSec();	

		ROS_DEBUG_STREAM( current_node->getX() / grid_size );
		ROS_DEBUG_STREAM( current_node->getY() / grid_size );

		CostMatrix(NodeNrMatrix(round(current_node->getX() / grid_size),round(current_node->getY() / grid_size)),0) = INF;
	
		ROS_DEBUG_STREAM( "Check minimum node" );
		get_minimum_node();

		ROS_DEBUG_STREAM( "Minimum node is: " << minRow << " at " << Network->at(minRow).getNode() );
		current_node = &Network->at(minRow);

		check3 += ros::Time::now().toSec() - begin_check3;
		ROS_DEBUG_STREAM( "Elapsed time of check 3 is: " << check3 );

		double begin_check6 = ros::Time::now().toSec();	
		if (useplot)
		{
			marker_pub.publish(line_list);		
		}
		check6 += ros::Time::now().toSec() - begin_check6;
		ROS_DEBUG_STREAM( "Elapsed time of check 6 is: " << check6 );

		ROS_DEBUG_STREAM( "next node is: " << next_node );
	}

	std::cout << "Elapsed time of initialization: " << check4 << std::endl;


	std::cout << "//////////////////Track back//////////////////" <<std::endl;
	p.z = 1.0;
	while (not(current_node->getNode() == 0))
	{
		p2.pose.position.x = current_node->getX() + map_center_x;
		p2.pose.position.y = current_node->getY() + map_center_y;
      		route_list.poses.push_back(p2);
		current_node = &Network->at(current_node->getPreviousNode());
	}

	std::cout << "//////////////////Flipping route//////////////////" <<std::endl;
	std::cout << "Size:" << route_list.poses.size() <<std::endl;

	for (int i = 1; i <= route_list.poses.size(); i++)
	{
		flipped_route_list.poses.push_back(route_list.poses[route_list.poses.size() - i]);
	}
		
	std::cout << "//////////////////Final point//////////////////" <<std::endl;
	flipped_route_list.poses[0].pose.position.x = start_state.x;
	flipped_route_list.poses[0].pose.position.y = start_state.y;

	marker_2_pub.publish(flipped_route_list);

	check5 += ros::Time::now().toSec() - begin_check5;
	std::cout << "Elapsed time of route calculation: " << check5 << std::endl;
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

void reinitialize()
{
	Network->clear();
	CostMatrix = MatrixXd::Ones(100000,3) * INF;
	NodeNrMatrix = MatrixXd::Ones(NodeNrMatrix.rows(),NodeNrMatrix.cols()) * INF;
	PosMatrix = MatrixX2i::Zero(100000,2);
	Network->reserve(100000);
	flipped_route_list.poses.clear();
	route_list.poses.clear();
	next_node = 1;
	current_node_nr = 0;
	start_node_nr = 0;
	Nx = 0;
	Ny = 0;
	new_cost = 0;
	new_dist = 0;

	check1 = 0;
	check2 = 0;
	check3 = 0;
	check4 = 0;
	check5 = 0;
	check6 = 0;
	checks = 0;
}

void start_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	double begin = ros::Time::now().toSec();	
	start_state = *state_msg;	

	calculate_route();
	double end = ros::Time::now().toSec();	
	std::cout << "Elapsed time is: " << end-begin << std::endl;
	std::cout << "Nodes checked: " << next_node << std::endl;

	reinitialize();

}

void goal_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	goal_state = *state_msg;
	final_node->initializeNode(goal_state.x - map_center_x, goal_state.y - map_center_y, goal_state.theta, 0.0, INF, 0, 0, false);
	std::cout << "Final node: [" << final_node->getX() << ", " << final_node->getY() << "]" << std::endl;
}

void map_cb (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	std::cout << "map received" << std::endl;
	map = *map_msg;
	temp_map = map;
	
	std::cout << "Data length is: " << map.data.size() <<std::endl;
	map_width = (float)map.info.width;
	map_height = (float)map.info.height;

	Nx = ceil(map_width / step_size);
	Ny = ceil(map_height / step_size);
 
	map_center_x = (float)map.info.origin.position.x;
	map_center_y = (float)map.info.origin.position.y;

	resolution = (float)map.info.resolution;

	//NodeNrMatrix.resize(map_width / grid_size, map_height / grid_size);

	std::cout << "Map center: " << map_center_x << ", " << map_center_y <<std::endl;
	std::cout << "Map size: " << map_width << " x " << map_height <<std::endl;
}

void obstacle_cb (const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg)
{
	Obstacle = *obstacle_msg;
	map = temp_map;

	std::cout << "Obstacle received" << std::endl;
	
	obstacle_x = Obstacle.pose.position.x - map_center_x;
	obstacle_y = Obstacle.pose.position.y - map_center_y;
	obstacle_th = Obstacle.pose.position.z;
	obstacle_u = Obstacle.twist.linear.x;
	obstacle_v = Obstacle.twist.linear.y;
	obstacle_w = Obstacle.twist.linear.z;
	obstacle_a = Obstacle.major_semiaxis;
	obstacle_b = Obstacle.minor_semiaxis;

	map_pub.publish(map);

	std::cout << "Obstacle map published" << std::endl;
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Obstacle_avoidance_planner");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	map_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);
	start_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);
	goal_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/goal",10,goal_cb);
	obstacle_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",10,obstacle_cb);

	map_pub = 	nh.advertise<nav_msgs::OccupancyGrid>("/map_tree_opt",10);
	marker_pub = nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_2_pub = nh_private.advertise<nav_msgs::Path>("route", 10);

	Network->reserve(100000);

	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.01;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	flipped_route_list.header.frame_id = route_list.header.frame_id = "/map";
	flipped_route_list.header.stamp = route_list.header.stamp = ros::Time::now();

	p2.header.frame_id = "/map";
	p2.header.stamp = ros::Time::now();

	Network->clear();
	CostMatrix = MatrixXd::Ones(100000,3) * INF;
	NodeNrMatrix = MatrixXd::Ones(NodeNrMatrix.rows(),NodeNrMatrix.cols()) * INF;
	PosMatrix = MatrixX2i::Zero(100000,2);


	next_node = 1;

	ros::spin();	
}
