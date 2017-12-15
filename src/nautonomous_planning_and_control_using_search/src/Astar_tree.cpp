#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <iostream>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <geometry_msgs/Point.h>
#include <nautonomous_planning_and_control_using_search/node.h>

#define INF 1000000
#define PI 3.14

ros::Subscriber map_sub;
ros::Subscriber start_sub;
ros::Subscriber goal_sub;

ros::Publisher marker_pub;

int next_node = 1;

nav_msgs::OccupancyGrid map;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable goal_state;

node* starting_node = new node();
node* current_node = new node();
node* new_node = new node();
node* final_node = new node();

float temp_x;
float temp_y;


float step_size = 1;
float angle_step = 0.05*PI;

float cost_c;
float cost_i;

float map_width = 0;
float map_height = 0;
float map_center_x = 0;
float map_center_y = 0;
float resolution;

std::vector<node>* Network = new std::vector<node>();
geometry_msgs::Point p;

visualization_msgs::Marker line_list;

void start_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	double begin = ros::Time::now().toSec();	

	std::cout << "//////////////////CLEAR PREVIOUS ROUTE//////////////////" << std::endl;
	Network->clear();
	next_node = 1;

	std::cout << "//////////////////START NEW ROUTE//////////////////" <<std::endl;

	start_state = *state_msg;
	starting_node->initializeNode(start_state.x, start_state.y, start_state.theta, sqrt(pow(start_state.x - goal_state.x,2) + pow(start_state.y - goal_state.y,2)), 0.0, 0, 0, false);
	
	Network->push_back(*starting_node);

	current_node = starting_node;

	while ((current_node->getDistToFinish() > 1 ))
	{
		if (not(current_node->getDistToFinish() == INF) && not(current_node->isConnected()))
		{
			// Node left forward
			temp_x = current_node->getX() + step_size * cos(current_node->getTheta() - angle_step);
			temp_y = current_node->getY() + step_size * sin(current_node->getTheta() - angle_step);
			std::cout << "Point 1: "
 << temp_x << ", " << temp_y <<std::endl; 
			p.x = current_node->getX();
			p.y = current_node->getY();
      			line_list.points.push_back(p);
			p.x = temp_x;
			p.y = temp_y;
      			line_list.points.push_back(p);

		
			
			if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] < 50)
			{
				std::cout << "Create new node" << std::endl;
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta() - angle_step, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
				std::cout << "Node is free" << std::endl;
			}
			else
			{	
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta() - angle_step, INF, current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
				std::cout << "Node is occupied" << std::endl;
				std::cout << "Map value at: " << (floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution) << " is: " << (int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] << std::endl;
			}

			// Node forwards
			
			temp_x = current_node->getX() + step_size * cos(current_node->getTheta());
			temp_y = current_node->getY() + step_size * sin(current_node->getTheta());
			std::cout << "Point 2: " << temp_x << ", " << temp_y <<std::endl;
			p.x = current_node->getX();
			p.y = current_node->getY();
      			line_list.points.push_back(p);
			p.x = temp_x;
			p.y = temp_y;
      			line_list.points.push_back(p);

			if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] < 50)
			{
				std::cout << "Create new node" << std::endl;
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta(), sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
				std::cout << "Node is free" << std::endl;
			}
			else
			{
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta(), INF, current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
				std::cout << "Node is occupied" << std::endl;
				std::cout << "Map value at: " << (floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution) << " is: " << (int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] << std::endl;
			}

			// Node right forward
			
			temp_x = current_node->getX() + step_size * cos(current_node->getTheta() + angle_step);
			temp_y = current_node->getY() + step_size * sin(current_node->getTheta() + angle_step);
			std::cout << "Point 3: " << temp_x << ", " << temp_y <<std::endl;
			p.x = current_node->getX();
			p.y = current_node->getY();
      			line_list.points.push_back(p);
			p.x = temp_x;
			p.y = temp_y;
      			line_list.points.push_back(p);

			if((int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] < 50)
			{
				std::cout << "Create new node" << std::endl;
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta() + angle_step, sqrt(pow(temp_x - goal_state.x,2) + pow(temp_y - goal_state.y,2)), current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
				std::cout << "Node is free" << std::endl;
			}
			else
			{
				new_node->initializeNode(temp_x, temp_y, current_node->getTheta() + angle_step, INF, current_node->getCost()+1,current_node->getNode(), next_node, false);
				Network->push_back(*new_node);
				current_node->addConnectedNode(next_node);
				next_node++;
				std::cout << "Node is occupied" << std::endl;
				std::cout << "Map value at: " << (floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x) /resolution) << " is: " << (int)map.data[(floor((temp_y-map_center_y)/resolution)-1) * map_width + floor((temp_x-map_center_x)/resolution)] << std::endl;
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
				std::cout << "Node is a dead end" << std::endl;
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

		std::cout << "Current node is now: node " << current_node->getNode() <<std::endl; 
		marker_pub.publish(line_list);
	}
	double end = ros::Time::now().toSec();	
	std::cout << "Elapsed time is: " << end-begin << std::endl;
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

	marker_pub = nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	Network->reserve(100000);

	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 1.0;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;
	ros::spin();	
}
