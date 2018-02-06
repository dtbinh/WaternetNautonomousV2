#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/WaypointList.h>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <geometry_msgs/Point.h>
#include <nautonomous_planning_and_control_using_search/node_tree.h>
#include <Eigen/Dense>

#define INF 1000000
#define PI 3.141592653589793238462643383279502884197169399375105820974
#define sq2 1.414213562373095048801688724209698078569671875376948073176

using namespace Eigen;

VectorXd CostMatrix(100000);	// Colummn 0: cost, Colummn 1: X-pos, Colummn 2: Y-pos
MatrixX2i PosMatrix(100000,2);		// Colummn 0: cost, Colummn 1: X-pos, Colummn 2: Y-pos

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

float step_size = 1;
float angle_step = 0.25*PI;

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

std::vector<node>* Network = new std::vector<node>();

geometry_msgs::Point p;
geometry_msgs::Point p1;
geometry_msgs::Point p2;


visualization_msgs::Marker line_list;
visualization_msgs::Marker route_list;

int current_node_nr = 0;;
float new_cost = 0;

bool useplot = true;

bool check_pos_already_exists(int pos_x_, int pos_y_)
{
	double begin_check1 = ros::Time::now().toSec();	
	
	bool return_value = false;
	for (int i = 0; i < next_node; i++)
	{
		if (PosMatrix(i,0) == pos_x_)
		{
			if (PosMatrix(i,1) == pos_y_)
			{
				return_value = true;
				break;
			}
		}
	}
	check1 += ros::Time::now().toSec() - begin_check1;
	std::cout << "Elapsed time of check 1 is: " << check1 << std::endl;
	return return_value;
}

void get_minimum_node()
{
	minRow = 0;
	minCost = INF;
	for ( int i = 0; i < next_node ; i++)
	{
		if (CostMatrix(i) < minCost)
		{
			minCost = CostMatrix(i);
			minRow = i;
		}
	}

}

void calculate_route()
{
	// Clear and initialize
	double begin_check4 = ros::Time::now().toSec();	
	Network->clear();
	CostMatrix = VectorXd::Ones(100000) * INF;
	next_node = 1;

	std::cout << "//////////////////START NEW ROUTE//////////////////" <<std::endl;

	starting_node->initializeNode(start_state.x, start_state.y, start_state.theta, sqrt(pow(start_state.x - goal_state.x,2) + pow(start_state.y - goal_state.y,2)), 0.0, 0, 0, false);
	
	Network->push_back(*starting_node);

	current_node = starting_node;

	CostMatrix(0) = 0.0;
	PosMatrix(0,0) = floor(start_state.x);
	PosMatrix(0,1) = floor(start_state.y);

	check4 += ros::Time::now().toSec() - begin_check4;

	std::cout << "//////////////////Generate//////////////////" <<std::endl;
	double begin_check5 = ros::Time::now().toSec();	
	while ((current_node->getDistToFinish() > step_size ))
	{
		double begin_check2 = ros::Time::now().toSec();	
	
		if ((std::abs(fmod(current_node->getTheta(),0.5*PI)) < 0.01) || (std::abs(fmod(current_node->getTheta(),0.5*PI) - 0.5 * PI) < 0.01) || (std::abs(fmod(current_node->getTheta(),0.5*PI) + 0.5 * PI) < 0.01))
		{
			// Node right forward	
			temp_theta = current_node->getTheta() - angle_step;
			temp_x = current_node->getX() + step_size * sq2 * cos(temp_theta);
			temp_y = current_node->getY() + step_size * sq2 * sin(temp_theta);
			new_cost = current_node->getCost() + step_size * sq2;

			if (useplot)
			{
				p.x = current_node->getX();
				p.y = current_node->getY();
	      			line_list.points.push_back(p);
				p.x = temp_x;
				p.y = temp_y;
	      			line_list.points.push_back(p);
			}
			PosMatrix(next_node,0) = floor(temp_x);
			PosMatrix(next_node,1) = floor(temp_y);
		
			if((temp_x < -(map_width*resolution/2)) || (temp_x > (map_width*resolution/2)) || (temp_y < -(map_height*resolution/2)) || (temp_y > (map_height*resolution/2)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
			else if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 50)
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				current_node->addConnectedNode(next_node);
				Network->push_back(*new_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
			else if(check_pos_already_exists(PosMatrix(next_node,0), PosMatrix(next_node,1)))
			{

				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}			
			else
			{	
				new_node->initializeNode(temp_x, temp_y, temp_theta, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = new_node->getTotalCost();
				next_node++;
			}
 
			// Node forwards
		
			temp_theta = current_node->getTheta();
			temp_x = current_node->getX() + step_size * cos(temp_theta);
			temp_y = current_node->getY() + step_size * sin(temp_theta);
			new_cost = current_node->getCost() + step_size;
			if (useplot)
			{
				p.x = current_node->getX();
				p.y = current_node->getY();
	      			line_list.points.push_back(p);
				p.x = temp_x;
				p.y = temp_y;
	      			line_list.points.push_back(p);
			}
			PosMatrix(next_node,0) = floor(temp_x);
			PosMatrix(next_node,1) = floor(temp_y);
		
			if((temp_x < -(map_width*resolution/2)) || (temp_x > (map_width*resolution/2)) || (temp_y < -(map_height*resolution/2)) || (temp_y > (map_height*resolution/2)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
			else if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 50)
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
		
			else if(check_pos_already_exists(PosMatrix(next_node,0), PosMatrix(next_node,1)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}			
			else
			{	
				new_node->initializeNode(temp_x, temp_y, temp_theta, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = new_node->getTotalCost();
				next_node++;
			}

			// Node left forward
		
			temp_theta = current_node->getTheta() + angle_step;
			temp_x = current_node->getX() + step_size * sq2 * cos(temp_theta);
			temp_y = current_node->getY() + step_size * sq2 * sin(temp_theta);
			new_cost = current_node->getCost() + step_size * sq2;
			if (useplot)
			{
				p.x = current_node->getX();
				p.y = current_node->getY();
	      			line_list.points.push_back(p);
				p.x = temp_x;
				p.y = temp_y;
	      			line_list.points.push_back(p);
			}
			PosMatrix(next_node,0) = floor(temp_x);
			PosMatrix(next_node,1) = floor(temp_y);
		
			if((temp_x < -(map_width*resolution/2)) || (temp_x > (map_width*resolution/2)) || (temp_y < -(map_height*resolution/2)) || (temp_y > (map_height*resolution/2)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
			else if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 50)
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
		
			else if(check_pos_already_exists(PosMatrix(next_node,0), PosMatrix(next_node,1)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}			
			else
			{	
				new_node->initializeNode(temp_x, temp_y, temp_theta, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = new_node->getTotalCost();
				next_node++;
			}
		}

		else if ((std::abs(fmod(current_node->getTheta(),0.5*PI) - 0.25*PI) < 0.01) || (std::abs(fmod(current_node->getTheta(),0.5*PI) + 0.25*PI) < 0.01))
		{
			// Node right forward	
			temp_theta = current_node->getTheta() - angle_step;
			temp_x = current_node->getX() + step_size * cos(temp_theta);
			temp_y = current_node->getY() + step_size * sin(temp_theta);
			new_cost = current_node->getCost() + step_size;
			if (useplot)
			{
				p.x = current_node->getX();
				p.y = current_node->getY();
	      			line_list.points.push_back(p);
				p.x = temp_x;
				p.y = temp_y;
	      			line_list.points.push_back(p);
			}
			PosMatrix(next_node,0) = floor(temp_x);
			PosMatrix(next_node,1) = floor(temp_y);
		
			if((temp_x < -(map_width*resolution/2)) || (temp_x > (map_width*resolution/2)) || (temp_y < -(map_height*resolution/2)) || (temp_y > (map_height*resolution/2)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
			else if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 50)
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
		
			else if(check_pos_already_exists(PosMatrix(next_node,0), PosMatrix(next_node,1)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}			
			else
			{	
				new_node->initializeNode(temp_x, temp_y, temp_theta, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = new_node->getTotalCost();
				next_node++;
			}

			// Node forwards
			temp_theta = current_node->getTheta();
			temp_x = current_node->getX() + step_size * sq2 * cos(temp_theta);
			temp_y = current_node->getY() + step_size * sq2 * sin(temp_theta);
			new_cost = current_node->getCost() + step_size * sq2;
			if (useplot)
			{
				p.x = current_node->getX();
				p.y = current_node->getY();
	      			line_list.points.push_back(p);
				p.x = temp_x;
				p.y = temp_y;
	      			line_list.points.push_back(p);
			}
			PosMatrix(next_node,0) = floor(temp_x);
			PosMatrix(next_node,1) = floor(temp_y);

			if((temp_x < -(map_width*resolution/2)) || (temp_x > (map_width*resolution/2)) || (temp_y < -(map_height*resolution/2)) || (temp_y > (map_height*resolution/2)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
			else if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 50)
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
		
			else if(check_pos_already_exists(PosMatrix(next_node,0), PosMatrix(next_node,1)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}			
			else
			{	
				new_node->initializeNode(temp_x, temp_y, temp_theta, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = new_node->getTotalCost();
				next_node++;
			}

			// Node left forward
			temp_theta = current_node->getTheta() + angle_step;
			temp_x = current_node->getX() + step_size * cos(temp_theta);
			temp_y = current_node->getY() + step_size * sin(temp_theta);
			new_cost = current_node->getCost() + step_size;
			if (useplot)
			{
				p.x = current_node->getX();
				p.y = current_node->getY();
	      			line_list.points.push_back(p);
				p.x = temp_x;
				p.y = temp_y;
	      			line_list.points.push_back(p);
			}
			PosMatrix(next_node,0) = floor(temp_x);
			PosMatrix(next_node,1) = floor(temp_y);

			if((temp_x < -(map_width*resolution/2)) || (temp_x > (map_width*resolution/2)) || (temp_y < -(map_height*resolution/2)) || (temp_y > (map_height*resolution/2)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
			else if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] > 50)
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}
		
			else if(check_pos_already_exists(PosMatrix(next_node,0), PosMatrix(next_node,1)))
			{
				new_node->initializeNode(temp_x, temp_y, temp_theta, INF, new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = INF;
				next_node++;
			}			
			else
			{	
				new_node->initializeNode(temp_x, temp_y, temp_theta, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), new_cost,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				CostMatrix(next_node) = new_node->getTotalCost();
				next_node++;
			}
		}

		else
		{
			std::cout << "Angle: " << fmod(current_node->getTheta(),0.5*PI) << std::endl;
			break;
		}

		check2 += ros::Time::now().toSec() - begin_check2;
		std::cout << "Elapsed time of check 2 is: " << check2 << std::endl;

		double begin_check3 = ros::Time::now().toSec();	
		CostMatrix(current_node->getNode()) = INF;


	
		get_minimum_node();



		current_node = &Network->at(minRow);

		check3 += ros::Time::now().toSec() - begin_check3;
		std::cout << "Elapsed time of check 3 is: " << check3 << std::endl;

		double begin_check6 = ros::Time::now().toSec();	

		marker_pub.publish(line_list);		

		check6 += ros::Time::now().toSec() - begin_check3;
		std::cout << "Elapsed time of check 6 is: " << check6 << std::endl;		

	}

	std::cout << "Elapsed time of initialization: " << check4 << std::endl;


	std::cout << "//////////////////Track back//////////////////" <<std::endl;
	p.z = 0.25;
	while (not(current_node->getNode() == 0))
	{
		p.x = current_node->getX();
		p.y = current_node->getY();
      		route_list.points.push_back(p);

		current_node = &Network->at(current_node->getPreviousNode());
		p.x = current_node->getX();
		p.y = current_node->getY();
      		route_list.points.push_back(p);
		marker_2_pub.publish(route_list);
	}

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

void start_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	double begin = ros::Time::now().toSec();	
	start_state = *state_msg;

	

	calculate_route();
	double end = ros::Time::now().toSec();	
	std::cout << "Elapsed time is: " << end-begin << std::endl;
	std::cout << "Nodes checked: " << next_node << std::endl;
}

void goal_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	goal_state = *state_msg;
	final_node->initializeNode(goal_state.x, goal_state.y, goal_state.theta, 0.0, INF, 0, 0, false);
}

void map_cb (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	std::cout << "map received" << std::endl;
	map = *map_msg;
	temp_map = map;
	
	std::cout << "Data length is: " << map.data.size() <<std::endl;
	map_width = (float)map.info.width;
	map_height = (float)map.info.height;

	map_center_x = (float)map.info.origin.position.x;
	map_center_y = (float)map.info.origin.position.y;

	resolution = (float)map.info.resolution;
	std::cout << "Map center: " << map_center_x << ", " << map_center_y <<std::endl;
	std::cout << "Map size: " << map_width << " x " << map_height <<std::endl;
}

void obstacle_cb (const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg)
{
	Obstacle = *obstacle_msg;
	map = temp_map;
	for (float i = -Obstacle.major_semiaxis; i < Obstacle.major_semiaxis; i+= resolution)
	{
		for (float j = -Obstacle.minor_semiaxis; j < Obstacle.minor_semiaxis; j+= resolution)
		{
			temp_x = Obstacle.state.pose.position.x + i;
			temp_y = Obstacle.state.pose.position.y + j;
			map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] = 100;
		}		
	}

	map_pub.publish(map);

	std::cout << "Obstacle map published" << std::endl;
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"A_star_tree_path_finding_opt");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	map_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);
	start_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);
	goal_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/goal",10,goal_cb);
	obstacle_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",10,obstacle_cb);

	map_pub = 	nh.advertise<nav_msgs::OccupancyGrid>("/map_tree_opt",10);
	marker_pub = nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_2_pub = nh_private.advertise<visualization_msgs::Marker>("visualization_marker_route", 10);

	Network->reserve(100000);

	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.2;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	route_list.header.frame_id = "/map";
	route_list.header.stamp = ros::Time::now();
	route_list.ns = "points_and_lines";
	route_list.action = visualization_msgs::Marker::ADD;
	route_list.pose.orientation.w = 1.0;

	route_list.id = 0;
	route_list.type = visualization_msgs::Marker::LINE_LIST;
	route_list.scale.x = 0.5;
	route_list.color.g = 1.0;
	route_list.color.a = 1.0;

	ros::spin();	
}
