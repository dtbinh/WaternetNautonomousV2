#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nautonomous_planning_and_control_using_search/node_jps.h>

ros::Subscriber map_sub;
ros::Publisher map_pub;

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid temp_map;

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

int row = 1;

node* New_Node = new node();
node* Current_Node = new node();

std::vector<node>* Grid = new std::vector<node>();
std::vector<int>* JumpPointList = new std::vector<int>();

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

	for (int j = 0; j < map_height; j++)
	{
		for (int i = 0; i < map_width; i++)
		{
			Grid->push_back(*New_Node);
			if ((int)map.data[(j * map_width + i)] == 100)
			{
				Grid->at(j*map_width + i).setObstacle();
			}
		}
	}

	for (int j = 1; j < map_height-1; j++)
	{
		for (int i = 1; i < map_width-1; i++)
		{
			if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[((j-1) * map_width + 1 + i)] == 0 && (int)map.data[((j-1) * map_width + i)] == 100)
			{
				ROS_INFO_STREAM("Left jump point found");
				map.data[(j*map_width+1+i)] = -1;
				if (!(Grid->at(j*map_width+1+i).getJumpPoint()))
				{
					JumpPointList->push_back(j*map_width+1+i);
				}
				Grid->at(j*map_width+1+i).setJumpPointLeft();
				
			}
			else if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[((j+1) * map_width + 1 + i)] == 0 && (int)map.data[((j+1) * map_width + i)] == 100)
			{
				ROS_INFO_STREAM("Left jump point found");
				map.data[(j*map_width+1+i)] = -1;
				if (!(Grid->at(j*map_width+1+i).getJumpPoint()))
				{
					JumpPointList->push_back(j*map_width+1+i);
				}
				Grid->at(j*map_width+1+i).setJumpPointLeft();

			}
		}
	}

	for (int j = 1; j < map_height-1; j++)
	{
		for (int i = 1; i < map_width-1; i++)
		{
			if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[((j-1) * map_width - 1 + i)] == 0 && (int)map.data[((j-1) * map_width + i)] == 100)
			{
				ROS_INFO_STREAM("Right jump point found");
				map.data[(j*map_width-1+i)] = -1;
				if (!(Grid->at(j*map_width-1+i).getJumpPoint()))
				{
					JumpPointList->push_back(j*map_width-1+i);
				}
				Grid->at(j*map_width-1+i).setJumpPointRight();

			}
			else if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[((j+1) * map_width - 1 + i)] == 0 && (int)map.data[((j+1) * map_width + i)] == 100)
			{
				ROS_INFO_STREAM("Right jump point found");
				map.data[(j*map_width-1+i)] = -1;
				if (!(Grid->at(j*map_width-1+i).getJumpPoint()))
				{
					JumpPointList->push_back(j*map_width-1+i);
				}
				Grid->at(j*map_width-1+i).setJumpPointRight();

			}
		}
	}

	for (int i = 1; i < map_width-1; i++)
	{
		for (int j = 1; j < map_height-1; j++)
		{
			if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[(j * map_width + 1 + i)] == 100 && (int)map.data[((j+1) * map_width + i + 1)] == 0)
			{
				ROS_INFO_STREAM("Upwards jump point found");
				map.data[((j+1)*map_width+i)] = -1;
				if (!(Grid->at((j+1)*map_width+i).getJumpPoint()))
				{
					JumpPointList->push_back((j+1)*map_width+i);
				}
				Grid->at((j+1)*map_width+i).setJumpPointUp();

			}
			else if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[(j * map_width - 1 + i)] == 100 && (int)map.data[((j+1) * map_width + i - 1)] == 0)
			{
				ROS_INFO_STREAM("Upwards jump point found");
				map.data[((j+1)*map_width+i)] = -1;
				if (!(Grid->at((j+1)*map_width+i).getJumpPoint()))
				{
					JumpPointList->push_back((j+1)*map_width+i);
				}
				Grid->at((j+1)*map_width+i).setJumpPointUp();

			}
		}
	}

	for (int i = 1; i < map_width-1; i++)
	{
		for (int j = 1; j < map_height-1; j++)
		{
			if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[(j * map_width + 1 + i)] == 100 && (int)map.data[((j-1) * map_width + i + 1)] == 0)
			{
				ROS_INFO_STREAM("Downwards jump point found");
				map.data[((j-1)*map_width+i)] = -1;
				if (!(Grid->at((j-1)*map_width+i).getJumpPoint()))
				{
					JumpPointList->push_back((j-1)*map_width+i);
				}
				Grid->at((j-1)*map_width+i).setJumpPointDown();

			}
			else if ((int)map.data[(j * map_width + i)] == 0 &&  (int)map.data[(j * map_width - 1 + i)] == 100 && (int)map.data[((j-1) * map_width + i - 1)] == 0)
			{
				ROS_INFO_STREAM("Downwards jump point found");
				map.data[((j-1)*map_width+i)] = -1;
				if (!(Grid->at((j-1)*map_width+i).getJumpPoint()))
				{
					JumpPointList->push_back((j-1)*map_width+i);
				}
				Grid->at((j-1)*map_width+i).setJumpPointDown();

			}
		}
	}
/*
	ROS_INFO_STREAM("All jump points found");
	map_pub.publish(map);

	for (int i = 0; i < JumpPointList->size(); i++)
	{
		int j = 1;
		ROS_INFO_STREAM("Check left");
		if (Grid->at(JumpPointList->at(i)).getJumpPointLeft())
		{
			Current_Node = &Grid->at(JumpPointList->at(i) - j );
			while (true)
			{
				if(Current_Node->getObstacle())
				{
					break;
				}
				else
				{
					Current_Node->setPointRight();
					map.data[JumpPointList->at(i) - j ] = 30;
					Current_Node = &Grid->at(JumpPointList->at(i) - (++j) );
				}
			}	
		}

		j = 1;
		ROS_INFO_STREAM("Check right");
		if (Grid->at(JumpPointList->at(i)).getJumpPointRight())
		{
			Current_Node = &Grid->at(JumpPointList->at(i) + j );
			while (true)
			{
				if(Current_Node->getObstacle())
				{
					break;
				}
				else
				{
					Current_Node->setPointLeft();
					map.data[JumpPointList->at(i) + j ] = 30;
					Current_Node = &Grid->at(JumpPointList->at(i) + (++j) );
				}
			}	
		}

		j = 1;
		ROS_INFO_STREAM("Check up");
		if (Grid->at(JumpPointList->at(i)).getJumpPointUp())
		{
			Current_Node = &Grid->at(JumpPointList->at(i) - j * map_width );
			ROS_INFO_STREAM("Checking point: " << JumpPointList->at(i)-  j * map_width);
			while (true)
			{
				if(Current_Node->getObstacle())
				{
					break;
				}
				else
				{
					Current_Node->setPointDown();
					map.data[JumpPointList->at(i) - j * map_width ] = 30;
					Current_Node = &Grid->at(JumpPointList->at(i) - (++j) * map_width );
			ROS_INFO_STREAM("Checking point: " << JumpPointList->at(i)-  j * map_width);
				}
			}	
		}

		j = 1;
		ROS_INFO_STREAM("Check down");
		if (Grid->at(JumpPointList->at(i)).getJumpPointDown())
		{
			Current_Node = &Grid->at(JumpPointList->at(i) + j * map_width );
			while (true)
			{
				if(Current_Node->getObstacle())
				{
					break;
				}
				else
				{
					Current_Node->setPointUp();
					map.data[JumpPointList->at(i) + j * map_width ] = 30;
					Current_Node = &Grid->at(JumpPointList->at(i) + (++j) * map_width );
				}
			}	
		}
	}
*/
	
	map_pub.publish(map);

}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Jump_point_search");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	map_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);

	map_pub = 	nh.advertise<nav_msgs::OccupancyGrid>("/map_tree_opt",10);

	Grid->reserve(map_width*map_height);
	JumpPointList->reserve(10000);
	Grid->clear();
	JumpPointList->clear();
	ros::spin();	
}
