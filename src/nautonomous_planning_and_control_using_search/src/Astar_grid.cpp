#include <ros/ros.h>
#include <ros/console.h>
#include <nautonomous_planning_and_control_using_search/node_grid.h>
#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/WaypointList.h>
#include <geometry_msgs/Point.h>


#define INF 1000000
#define PI 3.14
#define SQ2 1.414

ros::Subscriber map_sub;
ros::Subscriber start_sub;
ros::Subscriber goal_sub;

ros::Publisher marker_pub;

nav_msgs::OccupancyGrid map;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable goal_state;
nautonomous_mpc_msgs::StageVariable waypoint;
nautonomous_mpc_msgs::WaypointList Route;

float map_width;
float map_height;
float map_center_x;
float map_center_y;
float resolution;
float x;
float y;
float final_x;
float final_y;
float new_cost;
float new_x;
float new_y;
float new_node;

int final_node_nr;
int next_node = 0;

int it = 0;

node* starting_node = new node();
node* current_node = new node();

node* final_node = new node();
std::vector<node>* Network = new std::vector<node>();

Eigen::VectorXd weights;
Eigen::MatrixXf::Index minRow, minCol;

void set_node()
{
	if ((int)map.data[new_node] < 50)
	{
		if(new_cost < Network->at(new_node).getCost())
		{
			Network->at(new_node).initializeNode(new_x, new_y , final_x, final_y, false, new_cost, current_node->getNode(), new_node);
			weights(new_node) = Network->at(new_node).getTotalCost();
			std::cout << "Weight: " << weights(new_node) << std::endl;
			std::cout << "Node is free" << std::endl;
		}
	}
	else
	{
		Network->at(new_node).initializeNode(new_x, new_y , final_x, final_y, true,  new_cost, current_node->getNode(), new_node);
		std::cout << "Node is occupied" << std::endl;
	}
}

void calculate_route()
{
	Network->clear();
	Network->resize(map_width*map_height);
	weights.resize(map_width*map_height);

	for (int i = 0; i < weights.size(); i++)
	{
		weights(i) = INF;
	}

	std::cout << "//////////////////START NEW ROUTE//////////////////" <<std::endl;

	new_x = floor(start_state.x/resolution);
	new_y = floor(start_state.y/resolution);
	new_node = new_y * map_width + new_x;

	starting_node->initializeNode(new_x, new_y , final_x, final_y, false, 0.0, 0, new_node);

	Network->at(new_node).initializeNode(new_x, new_y , final_x, final_y, false, 0.0, 0, new_node);;

	current_node = &Network->at(new_node);
	
	std::cout << "Starting node" << current_node->getNode()  << std::endl;
	std::cout << "Final node" << final_node_nr  << std::endl;
	while (not(current_node->getNode() == final_node_nr ))
	{
		x = current_node->getX();
		y = current_node->getY();

		std::cout << "Check first border" <<std::endl;
		if (x == 0) // left border
		{
			if (y == 0) // bottom
			{
				// Set straight up
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width;
				set_node();

				// Set straight right
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() + 1;
				set_node();

				// Set right up
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width + 1;
				set_node();
			}
			else if (y == map_height - 1) // top
			{
				// Set straight down
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width;
				set_node();

				// Set straight right
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() + 1;
				set_node();

				// Set right down
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width + 1;
				set_node();
			}
			else // everywhere else
			{
				// Set straight up
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width;
				set_node();

				// Set straight right
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() + 1;
				set_node();

				// Set right up
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width + 1;
				set_node();

				// Set straight down
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width;
				set_node();

				// Set right down
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width + 1;
				set_node();
			}
		}
		
		else if (x == map_width - 1 ) // right border
		{
			if (y == 0) // bottom
			{
				// Set straight up
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width;
				set_node();

				// Set straight left
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() - 1;
				set_node();

				// Set left up
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width - 1;
				set_node();
			}
			else if (y == map_height - 1) // top
			{
				// Set straight down
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width;
				set_node();

				// Set straight left
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() - 1;
				set_node();

				// Set left down
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width - 1;
				set_node();
			}
			else // everywhere else
			{
				// Set straight up
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width;
				set_node();

				// Set straight left
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() - 1;
				set_node();

				// Set left up
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width - 1;
				set_node();

				// Set straight down
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width;
				set_node();

				// Set left down
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width - 1;
				set_node();
			}
		}

		else // everywhere else
		{
			if (y == 0) // bottom
			{
				// Set straight up
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width;
				set_node();

				// Set straight right
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() + 1;
				set_node();

				// Set right up
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width + 1;
				set_node();

				// Set straight left
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() - 1;
				set_node();

				// Set left up
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width - 1;
				set_node();
			}
			else if (y == map_height - 1) // top
			{
				// Set straight down
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width;
				set_node();

				// Set straight right
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() + 1;
				set_node();

				// Set right down
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width + 1;
				set_node();

				// Set straight left
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() - 1;
				set_node();

				// Set left down
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width - 1;
				set_node();
			}
			else // everywhere else
			{
				// Set straight up
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width;
				set_node();

				// Set straight right
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() + 1;
				set_node();

				// Set right up
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width + 1;
				set_node();

				// Set straight left
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY();
				new_node = current_node->getNode() - 1;
				set_node();

				// Set left up
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY() + 1;
				new_node = current_node->getNode() + map_width - 1;
				set_node();

				// Set straight down
				new_cost = current_node->getCost() + 1;
				new_x = current_node->getX();
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width;
				set_node();

				// Set right down
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() + 1;
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width + 1;
				set_node();

				// Set left down
				new_cost = current_node->getCost() + SQ2;
				new_x = current_node->getX() - 1;
				new_y = current_node->getY() - 1;
				new_node = current_node->getNode() - map_width - 1;
				set_node();
			}
		}
		weights(current_node->getNode()) = INF;
		weights.minCoeff(&minRow, &minCol);
		std::cout << "Weights: " << weights(0) << " ," << weights(1)  << " ," << weights(map_width) << " ," << weights(map_width +1) <<std::endl;
		current_node = &Network->at(minRow);
		std::cout << "Next node: (" << minRow << ") [" << current_node->getX() << ", " << current_node->getY() << "]" <<std::endl;

		if (it < 100){it++;}
		else{break;}
	}

}

void start_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	double begin = ros::Time::now().toSec();	
	start_state = *state_msg;

	calculate_route();
	
	double end = ros::Time::now().toSec();	
	std::cout << "Elapsed time is: " << end-begin << std::endl;
}

void goal_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg)
{
	goal_state = *state_msg;
	final_x = floor(goal_state.x/resolution);
	final_y = floor(goal_state.y/resolution);
	final_node_nr = final_x + (final_y - 1 ) * map_width;
	final_node->initializeNode(final_x, final_y, final_x, final_y, false, INF, 0, final_node_nr);
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

/*	line_list.header.frame_id = "/map";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 1.0;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;*/
	ros::spin();	
}
