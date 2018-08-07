#include <ros/ros.h>
#include <ros/console.h>
#include <cmath>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/WaypointList.h>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <nautonomous_obstacle_detection/Eigen/Eigenvalues>
#include <nautonomous_obstacle_detection/Quaternion_conversion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>

using namespace Eigen;

#define PI 3.141592653589793238462643383279502884197169399375105820974

int i = 0;

VectorXd NodeNrMatrix(4000000);

ros::Subscriber map_sub;

ros::Publisher map_pub;
ros::Publisher map_2_pub;
ros::Publisher marker_pub;
ros::Publisher marker_2_pub;
ros::Publisher markerarray_pub;
ros::Publisher border_pub;

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

int weighted_map_border = 7;
int number_of_borders = 30;
int max_line_dist = 10;

int occupied_point_list = 0;
int checks_ransac = 1000;

float map_weight;

geometry_msgs::Point p;
geometry_msgs::Point pmax;
geometry_msgs::Point pmin;
geometry_msgs::PoseStamped p1;
geometry_msgs::Point p2;

visualization_msgs::MarkerArray Markers;
visualization_msgs::Marker line_list;
visualization_msgs::Marker rectangle_list;

nav_msgs::Path route_list;
nav_msgs::Path flipped_route_list;

nautonomous_mpc_msgs::Obstacle obstacle; 
nautonomous_mpc_msgs::Obstacles obstacles;

std::vector<geometry_msgs::Point>* Edge_point_list = new std::vector<geometry_msgs::Point>();
std::vector<bool>* Part_of_inliers = new std::vector<bool>();
std::vector<bool>* Part_of_obstacles = new std::vector<bool>();
std::vector<int>* Inliers = new std::vector<int>();
std::vector<int>* BestInliers = new std::vector<int>();
std::vector<geometry_msgs::Point>* InliersSorted = new std::vector<geometry_msgs::Point>();
std::vector<geometry_msgs::Point>* Cornerpoints = new std::vector<geometry_msgs::Point>();
std::vector<float>* SortedAngles = new std::vector<float>();

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
			if ((int)weighted_map.data[j * map_width + i] > 0)
			{
				p.x = i;
				p.y = j;
				Edge_point_list->push_back(p);
				weighted_map.data[j * map_width + i] = 50;
				Part_of_inliers->push_back(false);
				Part_of_obstacles->push_back(false);
			}
		}
	}
	map_pub.publish(weighted_map);
	for (int l = 0; l < number_of_borders; l++)
	{
		ROS_INFO_STREAM("Points handled: " << occupied_point_list << "/" << Edge_point_list->size() );
		if((Edge_point_list->size()-occupied_point_list) < 100)
		{
			l = number_of_borders;
			break;
		}
		for (int k = 0; k < weighted_map.data.size(); k++)
		{
			weighted_map.data[k] = 0;
		}
		for (int j = 0; j < checks_ransac; j++)
		{
			ROS_DEBUG_STREAM("Check " << j << "/" << checks_ransac );
			rand_1 = floor(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * Edge_point_list->size());
			rand_x_1 = Edge_point_list->at(rand_1).x;
			rand_y_1 = Edge_point_list->at(rand_1).y;
			Inliers->push_back(rand_1);
			Part_of_inliers->at(rand_1) = true;

			rand_2 = floor(static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * Edge_point_list->size());
			rand_x_2 = Edge_point_list->at(rand_2).x;
			rand_y_2 = Edge_point_list->at(rand_2).y;
		
			for (int n = 0; n < 25; n++)
			{
				for (int k = 0; k < Edge_point_list->size(); k++)
				{
					if (!Part_of_inliers->at(k) && !Part_of_obstacles->at(k))
					{
						dist = fabs((rand_y_2 - rand_y_1) * Edge_point_list->at(k).x - (rand_x_2 - rand_x_1) * Edge_point_list->at(k).y + rand_x_2 * rand_y_1 - rand_y_2 * rand_x_1)/sqrt(pow(rand_y_2 - rand_y_1,2) + pow(rand_x_2 - rand_x_1,2));
						{				
						if (dist < max_line_dist)
							for (int m = 0; m < Inliers->size(); m++)
							{
								dist = sqrt(pow(Edge_point_list->at(Inliers->at(m)).x - Edge_point_list->at(k).x,2) + pow(Edge_point_list->at(Inliers->at(m)).y - Edge_point_list->at(k).y,2));						
								if ((dist < 5) && (dist > 0.1) && !Part_of_inliers->at(k))
								{
									Inliers->push_back(k);
									Part_of_inliers->at(k) = true;
								}
							}
						}
					}
				}
			}
			if(Inliers->size() > BestInliers->size())
			{
				BestInliers->clear();
				for (int k = 0; k < Inliers->size(); k++)
				{
					BestInliers->push_back(Inliers->at(k));
				}
				ROS_DEBUG_STREAM("Best inliers size: " << BestInliers->size() );
			}
			Inliers->clear();
			for (int k = 0; k < Part_of_inliers->size(); k++)
			{
				Part_of_inliers->at(k) = false;
			}
		}

		ROS_DEBUG_STREAM("Best set found" );
		
		float minpoint_x = Edge_point_list->at(BestInliers->at(0)).x * resolution + map_center_x;
		float minpoint_y = Edge_point_list->at(BestInliers->at(0)).y * resolution + map_center_y;

		for (int k = 0; k < BestInliers->size()-1; k++)
		{
			if (Edge_point_list->at(BestInliers->at(k)).y * resolution + map_center_y < minpoint_y)
			{
				minpoint_x = Edge_point_list->at(BestInliers->at(k)).x * resolution + map_center_x;
				minpoint_y = Edge_point_list->at(BestInliers->at(k)).y * resolution + map_center_y; 
			}
			else if ((Edge_point_list->at(BestInliers->at(k)).y * resolution + map_center_y == minpoint_y) && (Edge_point_list->at(BestInliers->at(k)).x * resolution + map_center_x < minpoint_x))
			{
				minpoint_x = Edge_point_list->at(BestInliers->at(k)).x * resolution + map_center_x;
				minpoint_y = Edge_point_list->at(BestInliers->at(k)).y * resolution + map_center_y; 
			}
		}

		ROS_DEBUG_STREAM("Minpoint: (" << minpoint_x << ", " << minpoint_y << ")" );

		p.x = Edge_point_list->at(BestInliers->at(0)).x * resolution + map_center_x;
		p.y = Edge_point_list->at(BestInliers->at(0)).y * resolution + map_center_y;
		float angle = atan2(p.y - minpoint_y, p.x - minpoint_x);
		InliersSorted->push_back(p);
		SortedAngles->push_back(angle);
		weighted_map.data[Edge_point_list->at(BestInliers->at(0)).y * map_width + Edge_point_list->at(BestInliers->at(0)).x] = 100;

		for (int k = 1; k < BestInliers->size()-1; k++)
		{
			p.x = Edge_point_list->at(BestInliers->at(k)).x * resolution + map_center_x;
			p.y = Edge_point_list->at(BestInliers->at(k)).y * resolution + map_center_y;
			weighted_map.data[Edge_point_list->at(BestInliers->at(k)).y * map_width + Edge_point_list->at(BestInliers->at(k)).x] = 100;
			angle = atan2(p.y - minpoint_y, p.x - minpoint_x);
			bool insert_found = false;
			for (int m = 0; m < InliersSorted->size(); m++)
			{
				if ((angle < SortedAngles->at(m)) || ((angle == SortedAngles->at(m)) && (p.x < InliersSorted->at(m).x)))
				{
					InliersSorted->insert(InliersSorted->begin() + m,p);
					SortedAngles->insert(SortedAngles->begin() + m,angle);
					insert_found = true;
					break;
				}
			}
			if(!insert_found)
			{
				InliersSorted->push_back(p);
				SortedAngles->push_back(angle);
			}
		}

		for (int k = 0; k < InliersSorted->size()-1; k++)
		{
			line_list.points.push_back(InliersSorted->at(k));
			line_list.points.push_back(InliersSorted->at(k+1));
		}

		marker_pub.publish(line_list);


		ROS_DEBUG_STREAM("Angles sorted" );
		Cornerpoints->push_back(InliersSorted->at(0));
		Cornerpoints->push_back(InliersSorted->at(1));
		Cornerpoints->push_back(InliersSorted->at(2));
		for (int k = 3; k < InliersSorted->size(); k++)
		{	
			p.x = InliersSorted->at(k).x;
			p.y = InliersSorted->at(k).y;

			//ROS_DEBUG_STREAM("Compare points (" << Cornerpoints->at(Cornerpoints->size()-2).x << ", " << Cornerpoints->at(Cornerpoints->size()-2).y << "), (" << Cornerpoints->at(Cornerpoints->size()-1).x << ", " << Cornerpoints->at(Cornerpoints->size()-1).y << "), (" << p.x << ", " << p.y << ")" );
			float angle_1 = atan2(Cornerpoints->at(Cornerpoints->size()-2).y - Cornerpoints->at(Cornerpoints->size()-1).y, Cornerpoints->at(Cornerpoints->size()-2).x - Cornerpoints->at(Cornerpoints->size()-1).x);
			//ROS_DEBUG_STREAM("Angle 1 : " << angle_1 );
			float angle_2 = atan2(p.y - Cornerpoints->at(Cornerpoints->size()-1).y, p.x - Cornerpoints->at(Cornerpoints->size()-1).x);
			//ROS_DEBUG_STREAM("Angle 2 : " << angle_2 );

			if (angle_1 < 0)
			{
				angle_1+= 2 * PI;
			}

			if (angle_2 < 0)
			{
				angle_2+= 2 * PI;
			}

			angle = angle_1 - angle_2;
			ROS_DEBUG_STREAM("Angle : " << angle );
			while (!(((angle > 0) && (angle < (PI+0.0000001))) || (angle < -(PI-0.0000001))))
			{
				ROS_DEBUG_STREAM("Remove point" );
				Cornerpoints->erase(Cornerpoints->begin() + Cornerpoints->size() - 1);
				ROS_DEBUG_STREAM("Compare points (" << Cornerpoints->at(Cornerpoints->size()-2).x << ", " << Cornerpoints->at(Cornerpoints->size()-2).y << "), (" << Cornerpoints->at(Cornerpoints->size()-1).x << ", " << Cornerpoints->at(Cornerpoints->size()-1).y << "), (" << p.x << ", " << p.y << ")" );
				float angle_1 = atan2(Cornerpoints->at(Cornerpoints->size()-2).y - Cornerpoints->at(Cornerpoints->size()-1).y, Cornerpoints->at(Cornerpoints->size()-2).x - Cornerpoints->at(Cornerpoints->size()-1).x);
				ROS_DEBUG_STREAM("Angle 1 : " << angle_1 );
				float angle_2 = atan2(p.y - Cornerpoints->at(Cornerpoints->size()-1).y, p.x - Cornerpoints->at(Cornerpoints->size()-1).x);
				ROS_DEBUG_STREAM("Angle 2 : " << angle_2 );

				if (angle_1 < 0)
				{
					angle_1+= 2 * PI;
				}

				if (angle_2 < 0)
				{
					angle_2+= 2 * PI;
				}

				angle = angle_1 - angle_2;
				ROS_DEBUG_STREAM("Angle : " << angle );
			}
			Cornerpoints->push_back(p);
		}

		line_list.points.clear();
		line_list.color.b = 1.0;
		line_list.color.r = 0.0;

		for (int k = 0; k < Cornerpoints->size()-1; k++)
		{
			line_list.points.push_back(Cornerpoints->at(k));
			line_list.points.push_back(Cornerpoints->at(k+1));
		}

		marker_2_pub.publish(line_list);

		ROS_DEBUG_STREAM("Fill pointmatrix" );
		MatrixXd PointsMatrix = MatrixXd::Zero(4,Cornerpoints->size());
		for (int k = 0; k < Cornerpoints->size(); k++)
		{
			PointsMatrix(0,k) = Cornerpoints->at(k).x;
			PointsMatrix(1,k) = Cornerpoints->at(k).y; 
			PointsMatrix(2,k) = 0;
			PointsMatrix(3,k) = 1;
		}

		ROS_DEBUG_STREAM("Get minimum area" );
		MatrixXd TransformedPointsMatrix = MatrixXd::Zero(Cornerpoints->size(),4);
		MatrixXd TransformationMatrix = MatrixXd::Zero(4,4);
		float minArea = 1e6;

		for (int k = 0; k < Cornerpoints->size() - 1; k++)
		{
			angle = atan2(Cornerpoints->at(k+1).y - Cornerpoints->at(k).y,Cornerpoints->at(k+1).x - Cornerpoints->at(k).x);
			TransformationMatrix(0,0) = cos(angle);
			TransformationMatrix(0,1) = sin(angle);
			TransformationMatrix(1,0) = -sin(angle);
			TransformationMatrix(1,1) = cos(angle);
			TransformationMatrix(2,2) = 1;
			TransformationMatrix(0,3) = -(cos(angle) * Cornerpoints->at(k).x + sin(angle) * Cornerpoints->at(k).y);
			TransformationMatrix(1,3) = -(-sin(angle) * Cornerpoints->at(k).x + cos(angle) * Cornerpoints->at(k).y);
			TransformationMatrix(3,3) = 1;

			TransformedPointsMatrix = TransformationMatrix * PointsMatrix;

			VectorXd MaxCoeffs = TransformedPointsMatrix.rowwise().maxCoeff();
			VectorXd MinCoeffs = TransformedPointsMatrix.rowwise().minCoeff();
			if (minArea > (MaxCoeffs(0) - MinCoeffs(0)) * (MaxCoeffs(1) - MinCoeffs(1)))
			{
				rectangle_list.pose.orientation = toQuaternion(0,0,angle);
				rectangle_list.scale.x = MaxCoeffs(0) - MinCoeffs(0) + resolution;
				rectangle_list.scale.y = MaxCoeffs(1) - MinCoeffs(1) + resolution;
				rectangle_list.pose.position.x = Cornerpoints->at(k).x + cos(angle) * (MaxCoeffs(0) + MinCoeffs(0))/2 - sin(angle) * (MaxCoeffs(1) + MinCoeffs(1))/2;
				rectangle_list.pose.position.y = Cornerpoints->at(k).y + sin(angle) * (MaxCoeffs(0) + MinCoeffs(0))/2 + cos(angle) * (MaxCoeffs(1) + MinCoeffs(1))/2;

				obstacle.pose.position.x = rectangle_list.pose.position.x;
				obstacle.pose.position.y = rectangle_list.pose.position.y;
				obstacle.pose.orientation = toQuaternion(0,0,angle);
				obstacle.major_semiaxis = (MaxCoeffs(0) - MinCoeffs(0) + resolution)/2;
				obstacle.minor_semiaxis = (MaxCoeffs(1) - MinCoeffs(1) + resolution)/2;
				
				minArea = (MaxCoeffs(0) - MinCoeffs(0)) * (MaxCoeffs(1) - MinCoeffs(1));	
			}
		}

		rectangle_list.scale.z = 1;
		rectangle_list.ns = Markers.markers.size();

		Markers.markers.push_back(rectangle_list);
		markerarray_pub.publish(Markers);
	
		ROS_DEBUG_STREAM("MinArea is : " << minArea );

		for (int k = 0; k < BestInliers->size(); k++)
		{
			Part_of_obstacles->at(BestInliers->at(k)) = true;
			occupied_point_list++;
		}		
		
		BestInliers->clear();
		for (int k = 0; k < Part_of_inliers->size(); k++)
		{
			Part_of_inliers->at(k) = false;
		}

		for (int k = 0; k < Edge_point_list->size(); k++)
		{
			if (!Part_of_obstacles->at(k))
			{
				weighted_map.data[Edge_point_list->at(k).y * map_width + Edge_point_list->at(k).x] = 50;
			}
		}

		obstacles.obstacles.push_back(obstacle);

		map_pub.publish(weighted_map);


		InliersSorted->clear();
		Cornerpoints->clear();
		SortedAngles->clear();
	}

	border_pub.publish(obstacles);
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

	map_2_pub.publish(weighted_map);

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

        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 		{
                   ros::console::notifyLoggerLevelsChanged();
        }

	map_sub = 		nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);

	map_pub = 		nh.advertise<nav_msgs::OccupancyGrid>("/edge_map",10);
	map_2_pub = 		nh.advertise<nav_msgs::OccupancyGrid>("/blur_map",10);
	marker_pub = 		nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_2_pub = 		nh_private.advertise<visualization_msgs::Marker>("visualization_marker_2", 10);
	markerarray_pub = 	nh_private.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
	border_pub =	 	nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("borders", 10);

	p1.pose.position.z = 1;
	p1.pose.orientation.w = 1;

	line_list.header.frame_id = "/occupancy_grid";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;

	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.2;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	rectangle_list.header.frame_id = "/occupancy_grid";
	rectangle_list.header.stamp = ros::Time::now();
	rectangle_list.action = visualization_msgs::Marker::ADD;

	rectangle_list.id = 0;
	rectangle_list.type = visualization_msgs::Marker::CUBE;
	rectangle_list.color.r = 1.0;
	rectangle_list.color.a = 1.0;

	ros::spin();	
}
