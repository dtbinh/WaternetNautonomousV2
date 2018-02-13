#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

#include <nav_msgs/OccupancyGrid.h>
#include <nautonomous_planning_and_control_using_search/node_jps.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;

float start_x;
float start_y;
float end_x = 25;
float end_y = 25;
float current_x;
float current_y;
float new_x;
float new_y;
int node_nr;
int final_node_nr;
int new_node_nr;
int node_it = 0;
int min_node = 0;
bool setStart = true;

int OpenListSize = 0;

float dx;
float dy;
float h;

//MatrixXd OpenList(10000,4);	// Colummn 0: cost, Colummn 1: X-pos, Colummn 2: Y-pos
bool ClosedList[15128] = {false};
MatrixXd FastList(100,2);

ros::Subscriber map_sub;
ros::Subscriber pose_sub;

ros::Publisher map_pub;
ros::Publisher marker_pub;

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid temp_map;

geometry_msgs::Point p1;
geometry_msgs::Point p2;
visualization_msgs::Marker line_list;

float map_width = 0;
float map_height = 0;
float map_center_x = 0;
float map_center_y = 0;
float resolution;

int Nx = 0;
int Ny = 0;

float step_size = 1;

bool bottom_occupied = false;
bool top_occupied = false;

node* New_Node = new node();
node* Current_Node = new node();

std::vector<node>* Grid = new std::vector<node>();
std::vector<int>* JumpPointList = new std::vector<int>();
std::vector<int>* OpenList = new std::vector<int>();
std::vector<int>* CostList = new std::vector<int>();
std::vector<float>* xposList = new std::vector<float>();
std::vector<float>* yposList = new std::vector<float>();

void plot_current_node(int height)
{
	ROS_DEBUG_STREAM("Start plotting current node");

	p1.z = height;
	p2.z = height;


	ROS_DEBUG_STREAM("Left dist is: " << Current_Node->getJumpLeft());
	p1.x = current_x;
	p1.y = current_y;

	ROS_DEBUG_STREAM("Original point (" << p1.x << ", " << p1.y << ")" );
	line_list.points.push_back(p1);
	p2.x = current_x - abs(Current_Node->getJumpLeft()/2.0);
	p2.y = current_y;
	ROS_DEBUG_STREAM("Left jump new (" << p2.x << ", " << p2.y << ")" );
	line_list.points.push_back(p2);

	ROS_DEBUG_STREAM("Right dist is: " << Current_Node->getJumpRight());
	p1.x = current_x;
	p1.y = current_y;
	line_list.points.push_back(p1);
	p2.x = current_x + abs(Current_Node->getJumpRight()/2.0);
	p2.y = current_y;
	ROS_DEBUG_STREAM("Right jump new (" << p2.x << ", " << p2.y << ")" );
	line_list.points.push_back(p2);

	ROS_DEBUG_STREAM("Down dist is: " << Current_Node->getJumpDown());
	p1.x = current_x;
	p1.y = current_y;
	line_list.points.push_back(p1);
	p2.x = current_x;
	p2.y = current_y - abs(Current_Node->getJumpDown()/2.0);
	ROS_DEBUG_STREAM("Down jump new (" << p2.x << ", " << p2.y << ")" );
	line_list.points.push_back(p2);

	ROS_DEBUG_STREAM("Up dist is: " << Current_Node->getJumpUp());
	p1.x = current_x;
	p1.y = current_y;
	line_list.points.push_back(p1);
	p2.x = current_x;
	p2.y = current_y + abs(Current_Node->getJumpUp()/2.0);
	ROS_DEBUG_STREAM("Up jump new (" << p2.x << ", " << p2.y << ")" );
	line_list.points.push_back(p2);

	ROS_DEBUG_STREAM("LeftUp dist is: " << Current_Node->getJumpLeftUp());
	p1.x = current_x;
	p1.y = current_y;
	line_list.points.push_back(p1);
	p2.x = current_x - abs(Current_Node->getJumpLeftUp()/2.0);
	p2.y = current_y + abs(Current_Node->getJumpLeftUp()/2.0);
	ROS_DEBUG_STREAM("LeftUp jump new (" << p2.x << ", " << p2.y << ")" );
	line_list.points.push_back(p2);

	ROS_DEBUG_STREAM("RightUp dist is: " << Current_Node->getJumpRightUp());
	p1.x = current_x;
	p1.y = current_y;
	line_list.points.push_back(p1);
	p2.x = current_x + abs(Current_Node->getJumpRightUp()/2.0);
	p2.y = current_y + abs(Current_Node->getJumpRightUp()/2.0);
	ROS_DEBUG_STREAM("RightUp jump new (" << p2.x << ", " << p2.y << ")" );
	line_list.points.push_back(p2);

	ROS_DEBUG_STREAM("LeftDown dist is: " << Current_Node->getJumpLeftDown());
	p1.x = current_x;
	p1.y = current_y;
	line_list.points.push_back(p1);
	p2.x = current_x - abs(Current_Node->getJumpLeftDown()/2.0);
	p2.y = current_y - abs(Current_Node->getJumpLeftDown()/2.0);
	ROS_DEBUG_STREAM("LeftDown jump new (" << p2.x << ", " << p2.y << ")" );
	line_list.points.push_back(p2);

	ROS_DEBUG_STREAM("RightDown dist is: " << Current_Node->getJumpRightDown());
	p1.x = current_x;
	p1.y = current_y;
	line_list.points.push_back(p1);
	p2.x = current_x + abs(Current_Node->getJumpRightDown()/2.0);
	p2.y = current_y - abs(Current_Node->getJumpRightDown()/2.0);
	ROS_DEBUG_STREAM("RightDown jump new (" << p2.x << ", " << p2.y << ")" );
	line_list.points.push_back(p2);


	marker_pub.publish(line_list);
}

int find_minimum_node()
{
	int temp_min = CostList->at(0);
	int temp_min_it = 0;
	
	for (int i = 1; i < CostList->size(); i++)
	{
		if (CostList->at(i) < temp_min)
		{
			temp_min = CostList->at(i);
			temp_min_it = i;
		}
	}
	ROS_DEBUG_STREAM("Found min");		
	
	ROS_DEBUG_STREAM("Minimum node is: " << OpenList->at(temp_min_it));		

	return temp_min_it;
}

void remove_node(int pos)
{
	OpenList->erase(OpenList->begin()+pos);
	CostList->erase(CostList->begin()+pos);
	xposList->erase(xposList->begin()+pos);
	yposList->erase(yposList->begin()+pos);
}

void find_optimal_solution()
{
	double start_time = ros::Time::now().toSec();
	current_x = start_x;
	current_y = start_y;
	node_nr = (2*current_x + map_width/2) + (2*current_y + map_height/2) * map_width;
	ROS_DEBUG_STREAM("Current node nr: " << node_nr);
	final_node_nr = (2*end_x + map_width/2) + (2*end_y + map_height/2) * map_width;
	Current_Node = &Grid->at(node_nr);
	Current_Node->setCost(0);

	dx = fabs(current_x - end_x);
	dy = fabs(current_y - end_y);
	h = fmin(dx, dy) * 14 + fabs(dx - dy) * 10 ;
	
	Current_Node->setDist(h);

	plot_current_node(0);

	for (int i = 1; i < 1000; i++)
	{
		// 1
		new_node_nr = node_nr + abs(Current_Node->getJumpRight());
		New_Node = &Grid->at(new_node_nr);
		if ((!(New_Node->getObstacle())) && !(ClosedList[new_node_nr]))
		{
			New_Node->setCost(Current_Node->getCost() + 10 * abs(Current_Node->getJumpRight()));
			new_x = current_x + abs(Current_Node->getJumpRight()) / 2.0;
			new_y = current_y;
			dx = abs(new_x - end_x);
			dy = abs(new_y - end_y);
			h = fmin(dx, dy) * 14 + abs(dx - dy) * 10 ;
			New_Node->setDist(h);

			OpenList->push_back(new_node_nr);
			CostList->push_back(New_Node->getTotalCost());
			xposList->push_back(new_x);
			yposList->push_back(new_y);

			ClosedList[new_node_nr] = true;

			node_it++;
		}
		else
		{
			ROS_DEBUG_STREAM("Right is an obstacle");
		}

		// 2
		new_node_nr = node_nr - abs(Current_Node->getJumpLeft());
		New_Node = &Grid->at(new_node_nr);
		if ((!(New_Node->getObstacle())) && !(ClosedList[new_node_nr]))
		{
			New_Node->setCost(Current_Node->getCost() + 10 * abs(Current_Node->getJumpLeft()));
			new_x = current_x - abs(Current_Node->getJumpLeft()) / 2.0;
			new_y = current_y;
			dx = abs(new_x - end_x);
			dy = abs(new_y - end_y);
			h = fmin(dx, dy) * 14 + abs(dx - dy) * 10 ;
			New_Node->setDist(h);

			OpenList->push_back(new_node_nr);
			CostList->push_back(New_Node->getTotalCost());
			xposList->push_back(new_x);
			yposList->push_back(new_y);

			ClosedList[new_node_nr] = true;

			node_it++;
		}
		else
		{
			ROS_DEBUG_STREAM("Left is an obstacle");
		}

		// 3
		new_node_nr = node_nr + map_width * abs(Current_Node->getJumpUp());
		New_Node = &Grid->at(new_node_nr);
		if ((!(New_Node->getObstacle())) && !(ClosedList[new_node_nr]))
		{
			New_Node->setCost(Current_Node->getCost() + 10 * abs(Current_Node->getJumpUp()));
			new_x = current_x;
			new_y = current_y + abs(Current_Node->getJumpUp()) / 2.0;
			dx = abs(new_x - end_x);
			dy = abs(new_y - end_y);
			h = fmin(dx, dy) * 14 + abs(dx - dy) * 10 ;
			New_Node->setDist(h);

			OpenList->push_back(new_node_nr);
			CostList->push_back(New_Node->getTotalCost());
			xposList->push_back(new_x);
			yposList->push_back(new_y);

			ClosedList[new_node_nr] = true;

			node_it++;
		}
		else
		{
			ROS_DEBUG_STREAM("Up is an obstacle");
		}

		// 4
		new_node_nr = node_nr - map_width * abs(Current_Node->getJumpDown());
		New_Node = &Grid->at(new_node_nr);
		if ((!(New_Node->getObstacle())) && !(ClosedList[new_node_nr]))
		{
			New_Node->setCost(Current_Node->getCost() + 10 * abs(Current_Node->getJumpDown()));
			new_x = current_x;
			new_y = current_y - abs(Current_Node->getJumpDown()) / 2.0;
			dx = abs(new_x - end_x);
			dy = abs(new_y - end_y);
			h = fmin(dx, dy) * 14 + abs(dx - dy) * 10 ;
			New_Node->setDist(h);

			OpenList->push_back(new_node_nr);
			CostList->push_back(New_Node->getTotalCost());
			xposList->push_back(new_x);
			yposList->push_back(new_y);

			ClosedList[new_node_nr] = true;

			node_it++;
		}
		else
		{
			ROS_DEBUG_STREAM("Down is an obstacle");
		}

		// 5
		new_node_nr = node_nr + (map_width + 1) * abs(Current_Node->getJumpRightUp());
		New_Node = &Grid->at(new_node_nr);
		if ((!(New_Node->getObstacle())) && !(ClosedList[new_node_nr]))
		{
			New_Node->setCost(Current_Node->getCost() + 14 * abs(Current_Node->getJumpRightUp()));
			new_x = current_x + abs(Current_Node->getJumpRightUp()) / 2.0;
			new_y = current_y + abs(Current_Node->getJumpRightUp()) / 2.0;
			dx = abs(new_x - end_x);
			dy = abs(new_y - end_y);
			h = fmin(dx, dy) * 14 + abs(dx - dy) * 10 ;
			New_Node->setDist(h);

			OpenList->push_back(new_node_nr);
			CostList->push_back(New_Node->getTotalCost());
			xposList->push_back(new_x);
			yposList->push_back(new_y);

			ClosedList[new_node_nr] = true;

			node_it++;
		}
		else
		{
			ROS_DEBUG_STREAM("RightUp is an obstacle");
		}

		// 6
		new_node_nr = node_nr + (-map_width + 1) * abs(Current_Node->getJumpRightDown());
		New_Node = &Grid->at(new_node_nr);
		if ((!(New_Node->getObstacle())) && !(ClosedList[new_node_nr]))
			{
			New_Node->setCost(Current_Node->getCost() + 14 * abs(Current_Node->getJumpRightDown()));
			new_x = current_x + abs(Current_Node->getJumpRightDown()) / 2.0;
			new_y = current_y - abs(Current_Node->getJumpRightDown()) / 2.0;
			dx = abs(new_x - end_x);
			dy = abs(new_y - end_y);
			h = fmin(dx, dy) * 14 + abs(dx - dy) * 10 ;
			New_Node->setDist(h);

			OpenList->push_back(new_node_nr);
			CostList->push_back(New_Node->getTotalCost());
			xposList->push_back(new_x);
			yposList->push_back(new_y);

			ClosedList[new_node_nr] = true;

			node_it++;
		}
		else
		{
			ROS_DEBUG_STREAM("RightDown is an obstacle");
		}

		// 7
		new_node_nr = node_nr + (map_width - 1) * abs(Current_Node->getJumpLeftUp());
		New_Node = &Grid->at(new_node_nr);
		if ((!(New_Node->getObstacle())) && !(ClosedList[new_node_nr]))
			{
			New_Node->setCost(Current_Node->getCost() + 14 * abs(Current_Node->getJumpLeftUp()));
			new_x = current_x - abs(Current_Node->getJumpLeftUp()) / 2.0;
			new_y = current_y + abs(Current_Node->getJumpLeftUp()) / 2.0;
			dx = abs(new_x - end_x);
			dy = abs(new_y - end_y);
			h = fmin(dx, dy) * 14 + abs(dx - dy) * 10 ;
			New_Node->setDist(h);

			OpenList->push_back(new_node_nr);
			CostList->push_back(New_Node->getTotalCost());
			xposList->push_back(new_x);
			yposList->push_back(new_y);

			ClosedList[new_node_nr] = true;

			node_it++;
		}
		else
		{
			ROS_DEBUG_STREAM("LeftUp is an obstacle");
		}

		// 8
		new_node_nr = node_nr - (map_width + 1) * abs(Current_Node->getJumpLeftDown());
		New_Node = &Grid->at(new_node_nr);
		if ((!(New_Node->getObstacle())) && !(ClosedList[new_node_nr]))
			{
			New_Node->setCost(Current_Node->getCost() + 14 * abs(Current_Node->getJumpLeftDown()));
			new_x = current_x - abs(Current_Node->getJumpLeftDown()) / 2.0;
			new_y = current_y - abs(Current_Node->getJumpLeftDown()) / 2.0;
			dx = abs(new_x - end_x);
			dy = abs(new_y - end_y);
			h = fmin(dx, dy) * 14 + abs(dx - dy) * 10 ;
			New_Node->setDist(h);

			OpenList->push_back(new_node_nr);
			CostList->push_back(New_Node->getTotalCost());
			xposList->push_back(new_x);
			yposList->push_back(new_y);

			ClosedList[new_node_nr] = true;

			node_it++;
		}
		else
		{
			ROS_DEBUG_STREAM("LeftDown is an obstacle");
		}
		
		min_node = find_minimum_node();
		current_x = xposList->at(min_node);
		current_y = yposList->at(min_node);
		node_nr = OpenList->at(min_node);
		Current_Node = &Grid->at(node_nr);

		OpenListSize = fmax(OpenList->size(), OpenListSize);

		remove_node(min_node);

		if (node_nr == final_node_nr)
		{
			ROS_INFO_STREAM("Final node found");			
			break;
		}
		plot_current_node(0);
	}

	float elapsed_time = ros::Time::now().toSec() - start_time;
	ROS_INFO_STREAM("Elapsed time: " << elapsed_time );
	ROS_INFO_STREAM("Maximum open list size: " << OpenListSize );

}

void waypoint_cb (const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
	if (setStart)
	{
		start_x = floor(pose_msg->pose.position.x*2)/2;
		start_y = floor(pose_msg->pose.position.y*2)/2;
	
		ROS_INFO_STREAM("Starting point is: (" << start_x << ", " << start_y << ")" );
		find_optimal_solution();
		setStart = false;
	}
	else
	{
		current_x = floor(pose_msg->pose.position.x*2)/2;
		current_y = floor(pose_msg->pose.position.y*2)/2;

		node_nr = (2*current_x + map_width/2) + (2*current_y + map_height/2) * map_width;
		Current_Node = &Grid->at(node_nr);
		ROS_INFO_STREAM("Point is an obstacle: " << Current_Node->getObstacle() );
	}
}

void map_cb (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	ROS_INFO_STREAM( "map received" );
	map = *map_msg;
	temp_map = map;
	
	ROS_INFO_STREAM( "Data length is: " << map.data.size() );
	map_width = (float)map.info.width;
	map_height = (float)map.info.height;

	Grid->reserve(map_width*map_height);

	ROS_INFO_STREAM( "Map width: " << map_width );
	ROS_INFO_STREAM( "Map height: " << map_height );
	ROS_INFO_STREAM( "Map size: " << map_width*map_height );

	Nx = ceil(map_width / step_size);
	Ny = ceil(map_height / step_size);
 
	map_center_x = (float)map.info.origin.position.x;
	map_center_y = (float)map.info.origin.position.y;

	resolution = (float)map.info.resolution;

	double start_time = ros::Time::now().toSec();
	for (int j = 0; j < map_height; j++)
	{
		for (int i = 0; i < map_width; i++)
		{
			Grid->push_back(*New_Node);
			if ((int)map.data[(j * map_width + i)] > 50)
			{
				Grid->at(j*map_width + i).setObstacle();
			}
			else if (((j-map_height/2)/2 == end_y) && ((i-map_width/2)/2 < end_x))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(3);
				map.data[(j*map_width + i)] = -1;
			}
			else if (((j-map_height/2)/2 == end_y) && ((i-map_width/2)/2 > end_x))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(4);
				map.data[(j*map_width + i)] = -1;
			}
			else if (((i-map_width/2)/2 == end_x) && ((j-map_height/2)/2 < end_y))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(6);
				map.data[(j*map_width + i)] = -1;
			}
			else if (((i-map_width/2)/2 == end_x) && ((j-map_height/2)/2 > end_y))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(1);
				map.data[(j*map_width + i)] = -1;
			}
			else if ((((i-map_width/2)/2 - end_x) == ((j-map_height/2)/2 - end_y)) && ((j-map_height/2)/2 < end_y))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(5);
				map.data[(j*map_width + i)] = -1;
			}
			else if ((((i-map_width/2)/2 - end_x) == ((j-map_height/2)/2 - end_y)) && ((j-map_height/2)/2 > end_y))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(2);
				map.data[(j*map_width + i)] = -1;
			}
			else if ((((i-map_width/2)/2 - end_x) == -((j-map_height/2)/2 - end_y)) && ((j-map_height/2)/2 < end_y))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(7);
				map.data[(j*map_width + i)] = -1;
			}
			else if ((((i-map_width/2)/2 - end_x) == -((j-map_height/2)/2 - end_y)) && ((j-map_height/2)/2 > end_y))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(0);
				map.data[(j*map_width + i)] = -1;
			}
			else if (((i-map_width/2)/2 == end_x) && ((j-map_height/2)/2 == end_y))
			{
				Grid->at(j*map_width + i).setTargetJumpPoint(6);
				map.data[(j*map_width + i)] = -1;
			}
		}
	}

	for (int j = 1; j < map_height-1; j++)
	{
		for (int i = 1; i < map_width-1; i++)
		{
			if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[((j-1) * map_width + 1 + i)] == 0 && (int)map.data[((j-1) * map_width + i)] == 100)
			{
				map.data[(j*map_width+1+i)] = -1;
				if (!(Grid->at(j*map_width+1+i).getJumpPoint()))
				{
					JumpPointList->push_back(j*map_width+1+i);
				}
				Grid->at(j*map_width+1+i).setJumpPointLeft();
				ROS_DEBUG_STREAM("Left jump point found at " << j*map_width+1+i);
				
			}
			else if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[((j+1) * map_width + 1 + i)] == 0 && (int)map.data[((j+1) * map_width + i)] == 100)
			{
				map.data[(j*map_width+1+i)] = -1;
				if (!(Grid->at(j*map_width+1+i).getJumpPoint()))
				{
					JumpPointList->push_back(j*map_width+1+i);
				}
				Grid->at(j*map_width+1+i).setJumpPointLeft();
				ROS_DEBUG_STREAM("Left jump point found at " << j*map_width+1+i);

			}
		}
	}

	for (int j = 1; j < map_height-1; j++)
	{
		for (int i = 1; i < map_width-1; i++)
		{
			if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[((j-1) * map_width - 1 + i)] == 0 && (int)map.data[((j-1) * map_width + i)] == 100)
			{
				map.data[(j*map_width-1+i)] = -1;
				if (!(Grid->at(j*map_width-1+i).getJumpPoint()))
				{
					JumpPointList->push_back(j*map_width-1+i);
				}
				Grid->at(j*map_width-1+i).setJumpPointRight();
				ROS_DEBUG_STREAM("Right jump point found at " << j*map_width-1+i);

			}
			else if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[((j+1) * map_width - 1 + i)] == 0 && (int)map.data[((j+1) * map_width + i)] == 100)
			{
				map.data[(j*map_width-1+i)] = -1;
				if (!(Grid->at(j*map_width-1+i).getJumpPoint()))
				{
					JumpPointList->push_back(j*map_width-1+i);
				}
				Grid->at(j*map_width-1+i).setJumpPointRight();
				ROS_DEBUG_STREAM("Right jump point found at " << j*map_width-1+i);

			}
		}
	}

	for (int i = 1; i < map_width-1; i++)
	{
		for (int j = 1; j < map_height-1; j++)
		{
			if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[(j * map_width + 1 + i)] == 100 && (int)map.data[((j+1) * map_width + i + 1)] == 0)
			{
				map.data[((j+1)*map_width+i)] = -1;
				if (!(Grid->at((j+1)*map_width+i).getJumpPoint()))
				{
					JumpPointList->push_back((j+1)*map_width+i);
				}
				Grid->at((j+1)*map_width+i).setJumpPointDown();
				ROS_DEBUG_STREAM("Down jump point found at " << (j+1)*map_width+i);

			}
			else if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[(j * map_width - 1 + i)] == 100 && (int)map.data[((j+1) * map_width + i - 1)] == 0)
			{
				map.data[((j+1)*map_width+i)] = -1;
				if (!(Grid->at((j+1)*map_width+i).getJumpPoint()))
				{
					JumpPointList->push_back((j+1)*map_width+i);
				}
				Grid->at((j+1)*map_width+i).setJumpPointDown();
				ROS_DEBUG_STREAM("Down jump point found at " << (j+1)*map_width+i);

			}
		}
	}

	for (int i = 1; i < map_width-1; i++)
	{
		for (int j = 1; j < map_height-1; j++)
		{
			if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[(j * map_width + 1 + i)] == 100 && (int)map.data[((j-1) * map_width + i + 1)] == 0)
			{
				map.data[((j-1)*map_width+i)] = -1;
				if (!(Grid->at((j-1)*map_width+i).getJumpPoint()))
				{
					JumpPointList->push_back((j-1)*map_width+i);
				}
				Grid->at((j-1)*map_width+i).setJumpPointUp();
				ROS_DEBUG_STREAM("Up jump point found at " << (j-1)*map_width+i);

			}
			else if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[(j * map_width - 1 + i)] == 100 && (int)map.data[((j-1) * map_width + i - 1)] == 0)
			{
				map.data[((j-1)*map_width+i)] = -1;
				if (!(Grid->at((j-1)*map_width+i).getJumpPoint()))
				{
					JumpPointList->push_back((j-1)*map_width+i);
				}
				Grid->at((j-1)*map_width+i).setJumpPointUp();
				ROS_DEBUG_STREAM("Up jump point found at " << (j-1)*map_width+i);

			}
		}
	}

	ROS_INFO_STREAM("All jump points found");
	map_pub.publish(map);

	for (int i = 0; i < JumpPointList->size(); i++)
	{
		int j = 1;
		if (Grid->at(JumpPointList->at(i)).getJumpPointLeft())
		{
			Current_Node = &Grid->at(JumpPointList->at(i) - j );
			while (true)
			{
				if(Current_Node->getObstacle())
				{
					break;
				}
				/*else if(Current_Node->getTargetJumpPoint())
				{
					break;
				}*/
				else
				{

					Current_Node->setPointRight();
					ROS_DEBUG_STREAM("Set " << JumpPointList->at(i) - j << " to right pointer");
					map.data[JumpPointList->at(i) - j ] = 30;
					Current_Node = &Grid->at(JumpPointList->at(i) - (++j) );
				}
			}	
		}

		j = 1;
		if (Grid->at(JumpPointList->at(i)).getJumpPointRight())
		{
			Current_Node = &Grid->at(JumpPointList->at(i) + j );
			while (true)
			{
				if(Current_Node->getObstacle())
				{
					break;
				}
				/*else if(Current_Node->getTargetJumpPoint())
				{
					break;
				}*/
				else
				{
					Current_Node->setPointLeft();
					ROS_DEBUG_STREAM("Set " << JumpPointList->at(i) + j << " to left pointer");
					map.data[JumpPointList->at(i) + j ] = 30;
					Current_Node = &Grid->at(JumpPointList->at(i) + (++j) );
				}
			}	
		}

		j = 1;
		if (Grid->at(JumpPointList->at(i)).getJumpPointUp())
		{
			Current_Node = &Grid->at(JumpPointList->at(i) + j * map_width );
			while (true)
			{
				if(Current_Node->getObstacle())
				{
					break;
				}
				/*else if(Current_Node->getTargetJumpPoint())
				{
					break;
				}*/

				else
				{
					Current_Node->setPointDown();
					ROS_DEBUG_STREAM("Set " << JumpPointList->at(i) + j * map_width << " to down pointer");
					map.data[JumpPointList->at(i) + j * map_width ] = 30;
					Current_Node = &Grid->at(JumpPointList->at(i) + (++j) * map_width );
				}
			}	
		}

		j = 1;
		if (Grid->at(JumpPointList->at(i)).getJumpPointDown())
		{
			Current_Node = &Grid->at(JumpPointList->at(i) - j * map_width );
			while (true)
			{
				if(Current_Node->getObstacle())
				{
					break;
				}
				/*else if(Current_Node->getTargetJumpPoint())
				{
					break;
				}*/
				else
				{
					Current_Node->setPointUp();
					ROS_DEBUG_STREAM("Set " << JumpPointList->at(i) - j * map_width << " to up pointer");
					map.data[JumpPointList->at(i) - j * map_width ] = 30;
					Current_Node = &Grid->at(JumpPointList->at(i) - (++j) * map_width );
				}
			}	
		}
	}
	ROS_INFO_STREAM("All pointers found");

	for (int i = 0; i < map_width * map_height; i++)
	{
		if (Grid->at(i).getObstacle())
		{
			ROS_DEBUG_STREAM("Point " << i << " is an obstacle");
		}
		else
		{
			// Check Right
			int j = 1;
			Current_Node = &Grid->at(i + j );
			while (true)
			{
				if (Current_Node->getObstacle())
				{
					ROS_DEBUG_STREAM("Point " << i << " has an obstacle " << j << " steps to the right");
					Grid->at(i).setJumpRight(-j);
					break;
				}
				else if (Current_Node->getJumpPoint())
				{
					ROS_DEBUG_STREAM("Point " << i << " has a jump point " << j << " steps to the right");
					Grid->at(i).setJumpRight(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(5))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps to the right");
					Grid->at(i).setJumpRight(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(0))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps to the right");
					Grid->at(i).setJumpRight(j);
					break;
				}
				else
				{
					Current_Node = &Grid->at(i + (++j));
				}
			}
			// Check Left
			j = 1;
			Current_Node = &Grid->at(i - j );
			while (true)
			{
				if (Current_Node->getObstacle())
				{
					ROS_DEBUG_STREAM("Point " << i << " has an obstacle " << j << " steps to the left");
					Grid->at(i).setJumpLeft(-j);
					break;
				}
				else if (Current_Node->getJumpPoint())
				{
					ROS_DEBUG_STREAM("Point " << i << " has a jump point " << j << " steps to the left");
					Grid->at(i).setJumpLeft(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(7))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps to the left");
					Grid->at(i).setJumpLeft(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(2))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps to the left");
					Grid->at(i).setJumpLeft(j);
					break;
				}
				else
				{
					Current_Node = &Grid->at(i - (++j));
				}
			}
			// Check Up
			j = 1;
			Current_Node = &Grid->at(i + j * map_width);
			while (true)
			{
				if (Current_Node->getObstacle())
				{
					ROS_DEBUG_STREAM("Point " << i << " has an obstacle " << j << " steps upwards");
					Grid->at(i).setJumpUp(-j);
					break;
				}
				else if (Current_Node->getJumpPoint())
				{
					ROS_DEBUG_STREAM("Point " << i << " has a jump point " << j << " steps upwards");
					Grid->at(i).setJumpUp(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(5))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps upwards");
					Grid->at(i).setJumpUp(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(7))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps upwards");
					Grid->at(i).setJumpUp(j);
					break;
				}
				else
				{
					Current_Node = &Grid->at(i + (++j) * map_width);
				}
			}
			// Check Down
			j = 1;
			Current_Node = &Grid->at(i - j * map_width);
			while (true)
			{
				if (Current_Node->getObstacle())
				{
					ROS_DEBUG_STREAM("Point " << i << " has an obstacle " << j << " steps downwards");
					Grid->at(i).setJumpDown(-j);
					break;
				}
				else if (Current_Node->getJumpPoint())
				{
					ROS_DEBUG_STREAM("Point " << i << " has a jump point " << j << " steps downwards");
					Grid->at(i).setJumpDown(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(0))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps downwards");
					Grid->at(i).setJumpDown(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(2))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps downwards");
					Grid->at(i).setJumpDown(j);
					break;
				}
				else
				{
					Current_Node = &Grid->at(i - (++j) * map_width);
				}
			}
			// Check Right Up
			j = 1;
			Current_Node = &Grid->at(i + j * (1 + map_width));
			while (true)
			{
				if (Current_Node->getObstacle())
				{
					ROS_DEBUG_STREAM("Point " << i << " has an obstacle " << j << " steps right up");
					Grid->at(i).setJumpRightUp(-j);
					break;
				}
				else if (Current_Node->getJumpPoint())
				{
					ROS_DEBUG_STREAM("Point " << i << " has a jump point " << j << " steps right up");
					Grid->at(i).setJumpRightUp(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(3))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps right up");
					Grid->at(i).setJumpRightUp(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(6))
				{
					ROS_DEBUG_STREAM("Point " << i << " has a target jump point " << j << " steps right up");
					Grid->at(i).setJumpRightUp(j);
					break;
				}
				else if (Current_Node->getPointRight())
				{
					ROS_DEBUG_STREAM("Point " << i << " has a right pointer " << j << " steps right up");
					Grid->at(i).setJumpRightUp(j);
					break;
				}
				else if (Current_Node->getPointUp())
				{
					ROS_DEBUG_STREAM("Point " << i << " has an upwards pointer " << j << " steps right up");
					Grid->at(i).setJumpRightUp(j);
					break;
				}
				else
				{
					Current_Node = &Grid->at(i + (++j) * (1 + map_width));
				}
			}
			// Check Right Down
			j = 1;
			Current_Node = &Grid->at(i + j * (1 - map_width));
			while (true)
			{
				if (Current_Node->getObstacle())
				{
					Grid->at(i).setJumpRightDown(-j);
					break;
				}
				else if (Current_Node->getJumpPoint())
				{
					Grid->at(i).setJumpRightDown(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(1))
				{
					Grid->at(i).setJumpRightDown(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(3))
				{
					Grid->at(i).setJumpRightDown(j);
					break;
				}
				else if (Current_Node->getPointRight())
				{
					Grid->at(i).setJumpRightDown(j);
					break;
				}
				else if (Current_Node->getPointDown())
				{
					Grid->at(i).setJumpRightDown(j);
					break;
				}
				else
				{
					Current_Node = &Grid->at(i + (++j) * (1 - map_width));
				}
			}
			// Check Left Up
			j = 1;
			Current_Node = &Grid->at(i + j * (-1 + map_width));
			while (true)
			{
				if (Current_Node->getObstacle())
				{
					Grid->at(i).setJumpLeftUp(-j);
					break;
				}
				else if (Current_Node->getJumpPoint())
				{
					Grid->at(i).setJumpLeftUp(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(4))
				{
					Grid->at(i).setJumpLeftUp(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(6))
				{
					Grid->at(i).setJumpLeftUp(j);
					break;
				}
				else if (Current_Node->getPointLeft())
				{
					Grid->at(i).setJumpLeftUp(j);
					break;
				}
				else if (Current_Node->getPointUp())
				{
					Grid->at(i).setJumpLeftUp(j);
					break;
				}
				else
				{
					Current_Node = &Grid->at(i + (++j) * (-1 + map_width));
				}
			}
			// Check Left Down
			j = 1;
			Current_Node = &Grid->at(i + j * (-1 - map_width));
			while (true)
			{
				if (Current_Node->getObstacle())
				{
					Grid->at(i).setJumpLeftDown(-j);
					break;
				}
				else if (Current_Node->getJumpPoint())
				{
					Grid->at(i).setJumpLeftDown(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(1))
				{
					Grid->at(i).setJumpLeftDown(j);
					break;
				}
				else if (Current_Node->getTargetJumpPoint(4))
				{
					Grid->at(i).setJumpLeftDown(j);
					break;
				}
				else if (Current_Node->getPointLeft())
				{
					Grid->at(i).setJumpLeftDown(j);
					break;
				}
				else if (Current_Node->getPointDown())
				{
					Grid->at(i).setJumpLeftDown(j);
					break;
				}
				else
				{
					Current_Node = &Grid->at(i + (++j) * (-1 - map_width));
				}
			}
		}

	}

	ROS_DEBUG_STREAM("Map build finished");
	float elapsed_time = ros::Time::now().toSec() - start_time;
	ROS_INFO_STREAM("Elapsed time: " << elapsed_time );
	map_pub.publish(map);
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Jump_point_search");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	map_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);
	pose_sub = 	nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,waypoint_cb);

	map_pub = 	nh.advertise<nav_msgs::OccupancyGrid>("/map_tree_opt",10);
	marker_pub = 	nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.1;
	line_list.scale.y = 0.5;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	Grid->reserve(map_width*map_height);
	JumpPointList->reserve(10000);
	Grid->clear();
	JumpPointList->clear();
	ros::spin();	
}
