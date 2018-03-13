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
#include <Eigen/Dense>

using namespace Eigen;

int i = 0;

VectorXd NodeNrMatrix(4000000);

ros::Subscriber map_sub;

ros::Publisher map_pub;
ros::Publisher marker_pub;

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid weighted_map;
nav_msgs::OccupancyGrid temp_map;

float map_width = 0;
float map_height = 0;
float map_center_x = 0;
float map_center_y = 0;
float resolution;

float rand_1;
float rand_x_1;
float rand_y_1;

float rand_2;
float rand_x_2;
float rand_y_2;

float best_x_1;
float best_y_1;

float best_x_2;
float best_y_2;

float dist;

int weighted_map_border = 10;

float map_weight;

geometry_msgs::Point p;
geometry_msgs::PoseStamped p1;
geometry_msgs::Point p2;

visualization_msgs::Marker line_list;
nav_msgs::Path route_list;
nav_msgs::Path flipped_route_list;

std::vector<geometry_msgs::Point>* Edge_point_list = new std::vector<geometry_msgs::Point>();
std::vector<geometry_msgs::Point>* Inliers = new std::vector<geometry_msgs::Point>();
std::vector<geometry_msgs::Point>* BestInliers = new std::vector<geometry_msgs::Point>();

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

void fit_lines()
{
	srand (time(NULL));
	for (i = 0; i < map_width; i++)
	{
		for (int j = 0; j < map_height; j++)
		{
			if ((int)weighted_map.data[j * map_width + i] > 50)
			{
				p.x = i;
				p.y = j;
				Edge_point_list->push_back(p);
			}
		}
	}
	for (int j = 0; j < 1000; j++)
	{
		rand_1 = floor(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * Edge_point_list->size());
		rand_x_1 = Edge_point_list->at(rand_1).x;
		rand_y_1 = Edge_point_list->at(rand_1).y;

		rand_2 = floor(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * Edge_point_list->size());
		rand_x_2 = Edge_point_list->at(rand_2).x;
		rand_y_2 = Edge_point_list->at(rand_2).y;
		
		for (int k = 0; k < Edge_point_list->size(); k++)
		{
			dist = fabs((rand_y_2 - rand_y_1) * Edge_point_list->at(k).x - (rand_x_2 - rand_x_1) * Edge_point_list->at(k).y + rand_x_2 * rand_y_1 - rand_y_2 * rand_x_1)/sqrt(pow(rand_y_2 - rand_y_1,2) + pow(rand_x_2 - rand_x_1,2));
			if (dist < 1)
			{
				Inliers->push_back(Edge_point_list->at(k));
			}
		}
		if(Inliers->size() > BestInliers->size())
		{
			BestInliers->clear();
			for (int k = 0; k < Inliers->size(); k++)
			{
				BestInliers->push_back(Inliers->at(k));
			}
			std::cout << "Best inliers size: " << BestInliers->size() << std::endl;
		}
		Inliers->clear();
	}

	std::cout << "Best set found" << std::endl;
	for (int k = 0; k < BestInliers->size()-1; k++)
	{
		p.x = BestInliers->at(k).x * resolution + map_center_x;
		p.y = BestInliers->at(k).y * resolution + map_center_y;
		line_list.points.push_back(p);
		p.x = BestInliers->at(k+1).x * resolution + map_center_x;
		p.y = BestInliers->at(k+1).y * resolution + map_center_y;
		line_list.points.push_back(p);
	}

	marker_pub.publish(line_list);
}

void detect_edges()
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
			for (int k = -1; k <= 1 ; k++)
			{
				if (k == -1)
				{	
					map_weight = map.data[(j+k) * map_width + i];
				}
				else
				{
					map_weight += map.data[(j+k) * map_width + i];
				}
			}
			temp_map.data[j * map_width + i] = (int)(map_weight / 3);
		}
	}
		

	for (i = 1; i < map_width-1; i++)
	{
		for (int j = 0; j < map_height; j++)
		{
			for ( int k = -1; k <= 1; k++)
			{
				if ( k == -1)
				{
					map_weight = temp_map.data[j * map_width + i + k];
				}
				else
				{
					map_weight += temp_map.data[j * map_width + i + k];
				}
			}
			weighted_map.data[j * map_width + i] = (int)(map_weight / (3));
		}
	}


	for (i = 0; i < map_width; i++)
	{
		for (int j = 0; j < map_height; j++)
		{
			weighted_map.data[j * map_width + i] = fabs(weighted_map.data[j * map_width + i] - map.data[j * map_width + i]);
		}
	}

	map_pub.publish(weighted_map);

	fit_lines();
}

void map_cb (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	ROS_DEBUG_STREAM( "map received" );
	map = *map_msg;
	temp_map = map;
	
	ROS_DEBUG_STREAM( "Data length is: " << map.data.size() );
	map_width = (float)map.info.width;
	map_height = (float)map.info.height;
 
	map_center_x = (float)map.info.origin.position.x;
	map_center_y = (float)map.info.origin.position.y;

	resolution = (float)map.info.resolution;
	ROS_DEBUG_STREAM( "Map center: " << map_center_x << ", " << map_center_y );
	ROS_DEBUG_STREAM( "Map size: " << map_width << " x " << map_height );

	detect_edges();
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Map_modifier");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	map_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);

	map_pub = 	nh.advertise<nav_msgs::OccupancyGrid>("/map_tree_opt",10);
	marker_pub = 	nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	p1.pose.position.z = 1;
	p1.pose.orientation.w = 1;

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
