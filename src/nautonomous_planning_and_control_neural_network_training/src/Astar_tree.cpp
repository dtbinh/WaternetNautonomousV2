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
#include <nautonomous_planning_and_control_neural_network_training/node_tree.h>


#define INF 1000000
#define PI 3.141592653589793238462643383279502884197169399375105820974

ros::Subscriber map_sub;
ros::Subscriber start_sub;
ros::Subscriber goal_sub;
ros::Subscriber obstacle_sub;

ros::Publisher map_pub;
ros::Publisher marker_pub;
ros::Publisher marker_2_pub;

int next_node = 1;

nav_msgs::OccupancyGrid map;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable goal_state;
nautonomous_mpc_msgs::StageVariable waypoint;
nautonomous_mpc_msgs::WaypointList Route;
nautonomous_mpc_msgs::Obstacle Obstacle;

node* starting_node = new node();
node* current_node = new node();
node* new_node = new node();
node* final_node = new node();

float temp_x;
float temp_y;


float step_size = 2.5;
float angle_step = 0.25*PI;

float cost_c;
float cost_i;

float map_width = 0;
float map_height = 0;
float map_center_x = 0;
float map_center_y = 0;
float resolution;

std::vector<node>* Network = new std::vector<node>();

geometry_msgs::Point p;
geometry_msgs::Point p1;
geometry_msgs::Point p2;

float theta;

visualization_msgs::Marker line_list;
visualization_msgs::Marker route_list;

void calculate_route()
{
	Network->clear();
	Network->reserve(100000);
	next_node = 1;

	std::cout << "//////////////////START NEW ROUTE//////////////////" <<std::endl;

	starting_node->initializeNode(start_state.x, start_state.y, start_state.theta, sqrt(pow(start_state.x - goal_state.x,2) + pow(start_state.y - goal_state.y,2)), 0.0, 0, 0, false);
	
	Network->push_back(*starting_node);

	current_node = starting_node;

	p.z = 0.75;

	while ((current_node->getDistToFinish() > step_size ))
	{
		if (not(current_node->getDistToFinish() == INF) && not(current_node->isConnected()))
		{
			// Node left forward
			temp_x = current_node->getX() + step_size * cos(current_node->getTheta() - angle_step);
			temp_y = current_node->getY() + step_size * sin(current_node->getTheta() - angle_step);
			p.x = current_node->getX();
			p.y = current_node->getY();
      			line_list.points.push_back(p);
			p.x = temp_x;
			p.y = temp_y;
      			line_list.points.push_back(p);
		
			
			if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] < 50)
			{
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta() - angle_step, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
			}
			else
			{	
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta() - angle_step, INF, current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
			}

			// Node forwards
			
			temp_x = current_node->getX() + step_size * cos(current_node->getTheta());
			temp_y = current_node->getY() + step_size * sin(current_node->getTheta());
			p.x = current_node->getX();
			p.y = current_node->getY();
      			line_list.points.push_back(p);
			p.x = temp_x;
			p.y = temp_y;
      			line_list.points.push_back(p);
	
			if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] < 50)
			{
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta(), sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
			}
			else
			{
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta(), INF, current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
			}

			// Node right forward
			
			temp_x = current_node->getX() + step_size * cos(current_node->getTheta() + angle_step);
			temp_y = current_node->getY() + step_size * sin(current_node->getTheta() + angle_step);
			p.x = current_node->getX();
			p.y = current_node->getY();
      			line_list.points.push_back(p);
			p.x = temp_x;
			p.y = temp_y;
      			line_list.points.push_back(p);

			if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] < 50)
			{
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta() + angle_step, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
			}
			else
			{
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta() + angle_step, INF, current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
			}

			cost_c = Network->at(next_node - 3).getTotalCost();
			cost_i = 0;
			for (int i = 1; i < 3 ; i++)
			{
				if (Network->at(next_node - 3 + i).getTotalCost() < cost_c)
				{
					cost_c = Network->at(next_node - 3 + i).getTotalCost();
					cost_i = i;
				}
			}
			if (not(cost_c >= INF))
			{
				current_node = &Network->at(next_node - 3 + cost_i);
			}	
			else
			{
				current_node->setDistToFinishToINF();
				current_node = &Network->at(current_node->getPreviousNode());
			}
		}
		else
		{
			cost_c = Network->at(current_node->getConnectedNodes().at(0)).getTotalCost();
			cost_i = 0;
			for (int i = 1; i < 3 ; i++)
			{
				if (Network->at(current_node->getConnectedNodes().at(i)).getTotalCost() < cost_c)
				{
					cost_c = Network->at(current_node->getConnectedNodes().at(i)).getTotalCost();
					cost_i = i;
				}
			}

			if (not(cost_c == INF))
			{
				current_node = &Network->at(current_node->getConnectedNodes().at(cost_i));
			}
			else

			{
				current_node->setDistToFinishToINF();
				current_node = &Network->at(current_node->getPreviousNode());
			}
		}
		marker_pub.publish(line_list);
	}

	std::cout << "//////////////////Track back//////////////////" <<std::endl;
	p.z = 1;
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
	std::cout << "Nodes checked: " << next_node-1 << std::endl;
}

void goal_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	goal_state = *state_msg;
	final_node->initializeNode(goal_state.x, goal_state.y, goal_state.theta, 0.0, INF, 0, 0, false);
}

void obstacle_cb (const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg)
{
	Obstacle = *obstacle_msg;
	for (float i = -Obstacle.major_semiaxis; i < Obstacle.major_semiaxis; i+= resolution)
	{
		for (float j = -Obstacle.minor_semiaxis; j < Obstacle.minor_semiaxis; j+= resolution)
		{
			temp_x = Obstacle.state.pose.position.x + i;
			temp_y = Obstacle.state.pose.position.y + j;
			map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] = 100;
			std::cout << "Set obstacle" <<std::endl;
		}		
	}


	map_pub.publish(map);

	std::cout << "Obstacle map published" << std::cout;
}

void map_cb (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	std::cout << "map received" << std::endl;
	map = *map_msg;
	
	std::cout << "Data length is: " << map.data.size() <<std::endl;
	map_width = (float)map.info.width;
	map_height = (float)map.info.height;

	map_center_x = (float)map.info.origin.position.x;
	map_center_y = (float)map.info.origin.position.y;

	resolution = (float)map.info.resolution;
	std::cout << "Map center: " << map_center_x << ", " << map_center_y <<std::endl;
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"A_star_tree_path_finding");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	map_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);
	start_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);
	goal_sub = 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/goal",10,goal_cb);
	obstacle_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",10,obstacle_cb);

	map_pub = 	nh_private.advertise<nav_msgs::OccupancyGrid>("obstacle_map", 10);
	marker_pub = 	nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_2_pub = 	nh_private.advertise<visualization_msgs::Marker>("visualization_marker_route", 10);

	Network->reserve(100000);

	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.2;
	line_list.color.b = 1.0;
	line_list.color.a = 1.0;

	route_list.header.frame_id = "/map";
	route_list.header.stamp = ros::Time::now();
	route_list.ns = "points_and_lines";
	route_list.action = visualization_msgs::Marker::ADD;
	route_list.pose.orientation.w = 1.0;

	route_list.id = 0;
	route_list.type = visualization_msgs::Marker::LINE_LIST;
	route_list.scale.x = 0.5;
	route_list.color.r = 0.5;
	route_list.color.b = 1.0;
	route_list.color.a = 1.0;

	ros::spin();	
}
