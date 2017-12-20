#include <ros/ros.h>
#include <ros/console.h>
/*#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

using namespace Eigen;
using namespace std;

ros::Subscriber map_sub;
ros::Subscriber pc_sub;
ros::Subscriber EKF_sub;

ros::Publisher OccupiedMap_pub;

nav_msgs::OccupancyGrid OccupancyGrid;
nav_msgs::OccupancyGrid OccupiedMap;

float OccupancyGrid_width = 0;
float OccupancyGrid_height = 0;
float OccupancyGrid_center_x = 0;
float OccupancyGrid_center_y = 0;
float resolution;

float BoatWidth = 3;
float BoatLength = 5;
float BoatWidthOffset = 0; 
float BoatLengthOffset = 0;
int Boat_pos_x = 0;
int Boat_pos_y = 0;

bool VoxelFound = true;

Matrix4f transformation1 = Eigen::Matrix4f::Identity();
Matrix4f transformation2 = Eigen::Matrix4f::Identity();

const int gridSize = 200;
int grid[gridSize][gridSize];

float x_pos, y_pos, z_pos;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	for (int i = 0; i < gridSize; i++){
		for (int j = 0; j < gridSize; j++){
			grid[i][j] = 0;
		}
	}

	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	pcl_conversions::toPCL(*cloud_msg, *cloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud, *temp_cloud);

	pcl::transformPointCloud (*temp_cloud, *transformed_cloud, transformation1);
	pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transformation2);

	for ( int i = 0; i < transformed_cloud->size(); i++)
	{
		x_pos = rint(transformed_cloud->points[i].x/resolution)*resolution;
		y_pos = rint(transformed_cloud->points[i].y/resolution)*resolution;
		z_pos = rint(transformed_cloud->points[i].z/resolution)*resolution;

		if (((x_pos > -(gridSize*resolution)/2) && (x_pos < (gridSize*resolution)/2) && (y_pos > -(gridSize*resolution)/2) && (y_pos < (gridSize*resolution)/2) && (z_pos < 1) && (z_pos > -1)) && not((x_pos > (BoatLengthOffset - BoatLength/2)) && (x_pos < (BoatLengthOffset + BoatLength/2)) && (y_pos > (BoatWidthOffset - BoatWidth/2)) && (y_pos < (BoatWidthOffset + BoatWidth/2))))
		{
			grid[(int)((x_pos+(gridSize*resolution)/2)/resolution)][(int)((y_pos+(gridSize*resolution)/2)/resolution)] = grid[(int)((x_pos+(gridSize*resolution)/2)/resolution)][(int)((y_pos+(gridSize*resolution)/2)/resolution)] + 1;
			VoxelFound = true;
		}
	}

	for ( int i = 0; i < gridSize; i++)
	{
		for ( int j = 0; j < gridSize; j++)
		{
			if (grid[i][j] > 5)
			{
				map.data[(floor((j-map_center_y)/resolution)-1) * map_width + floor((i-map_center_x)/resolution)] = 100;
			}	
		}
	}	
}

void EKF_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& ekf_msg)
{
	Boat_pos_x = rint(ekf_msg->x);
	Boat_pos_y = rint(ekf_msg->y);

	transformation1(0,3) = Boat_pos_x;
	transformation1(1,3) = Boat_pos_y;

	transformation1(0,0) = cos(ekf_msg->theta);
	transformation1(0,1) = sin(ekf_msg->theta);
	transformation1(1,0) = -sin(ekf_msg->theta);
	transformation1(1,1) = cos(ekf_msg->theta);

	transformation2(0,0) = cos(ekf_msg->theta);
	transformation2(0,1) = sin(ekf_msg->theta);
	transformation2(1,0) = -sin(ekf_msg->theta);
	transformation2(1,1) = cos(ekf_msg->theta);
}

void OccupancyGrid_cb (const nav_msgs::OccupancyGrid::ConstPtr& OccupancyGrid_msg)
{
	std::cout << "OccupancyGrid received" << std::endl;
	OccupancyGrid = *OccupancyGrid_msg;
	
	OccupancyGrid_width = (float)OccupancyGrid.info.width;
	OccupancyGrid_height = (float)OccupancyGrid.info.height;

	OccupancyGrid_center_x = (float)OccupancyGrid.info.origin.position.x;
	OccupancyGrid_center_y = (float)OccupancyGrid.info.origin.position.y;

	resolution = (float)OccupancyGrid.info.resolution;
}
*/
int main (int argc, char** argv)
{
	ros::init (argc, argv,"Obstacle_detection_using_OccupancyGrid");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
/*	pc_sub = 		nh.subscribe<sensor_msgs::PointCloud2>("/point_cloud",1,cloud_cb);
	EKF_sub = 		nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/Ekf/next_state",1,EKF_cb);
	map_sub = 		nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,OccupancyGrid_cb);

	OccupiedMap_sub = 	nh.subscribe<nav_msgs::OccupancyGrid>("OccupiedMap",10);
*/
	ros::spin();	
}
