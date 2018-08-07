#include <ros/ros.h>
#include <ros/console.h>

//PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <iostream>

#include <nav_msgs/OccupancyGrid.h>
#include <nautonomous_obstacle_detection/PVector.h>
#include <nautonomous_obstacle_detection/Blob.h>
#include <nautonomous_obstacle_detection/Eigen/Eigenvalues>
#include <nautonomous_obstacle_detection/Quaternion_conversion.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <nautonomous_pose_msgs/PointWithCovarianceStamped.h>

#include <nautonomous_mpc_msgs/StageVariable.h>
#include <tf/transform_listener.h>


// Namespaces
using namespace Eigen;
using namespace std;
using namespace cv;


// Parameters
Mat src, src_gray;
Mat dst, detected_edges;
int edgeThresh = 1;
int lowThreshold = 0;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";

float BoatWidth = 5;
float BoatLength = 10;
float BoatWidthOffset = 0; 
float BoatLengthOffset = -2.5;

bool UseJarvis = false;
bool VoxelFound = false;
float AvgX = -1;
float AvgY = -1;
float first_principle_axis = -1;
float second_principle_axis = -1;
double angle = -1;
float theta = 0;
float const voxelSize = 0.5;
int const gridSize = 1000; // In number of squares (gridSize(m)/voxelSize)
int const pointTreshold = 1;
int const origin_x = 628604;
int const origin_y = 5802730;
int Boat_pos_x = 0;
int Boat_pos_y = 0;

float map_width = 0;
float map_height = 0;
float map_center_x = 0;
float map_center_y = 0;
float resolution;
int weighted_map_border = 5;
double map_weight;

sensor_msgs::Imu Imu;

nav_msgs::OccupancyGrid grid_map;
nav_msgs::OccupancyGrid temp_map;
nav_msgs::OccupancyGrid weighted_map;

Matrix4f transformation1 = Eigen::Matrix4f::Identity();
Matrix4f transformation2 = Eigen::Matrix4f::Identity();
tf::StampedTransform transform_lidar_grid;

nautonomous_mpc_msgs::Obstacle ghost_obstacle;

// Publishers
ros::Publisher marker_pub;
ros::Publisher message_pub;
ros::Publisher Transformed_pcl_pub;
ros::Publisher map_pub;
// Subscribers
ros::Subscriber pc_sub;
ros::Subscriber EKF_sub;
ros::Subscriber map_sub;


tf::TransformListener *listener;


// Initialization
float x_pos, y_pos, z_pos, x_pos_origin, y_pos_origin;
float x_pos_to_map, y_pos_to_map;

int grid[gridSize][gridSize];

std::vector<Blob>* Blobs = new std::vector<Blob>();

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	ROS_DEBUG_STREAM( "Initialize grid" ); 
	for (int i = 0; i < gridSize; i++){
		for (int j = 0; j < gridSize; j++){
			grid[i][j] = 0;
		}
	}

	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	pcl_conversions::toPCL(*cloud_msg, *cloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud, *temp_cloud);

	*transformed_cloud = *temp_cloud;
	for (int i = 0; i < transformed_cloud->size(); i++)
	{
		transformed_cloud->points[i].x = (temp_cloud->points[i].x - transform_lidar_grid.getOrigin().x()) * cos(tf::getYaw(transform_lidar_grid.getRotation())) + (temp_cloud->points[i].y - transform_lidar_grid.getOrigin().y()) * sin(tf::getYaw(transform_lidar_grid.getRotation()));
		transformed_cloud->points[i].y = -(temp_cloud->points[i].x - transform_lidar_grid.getOrigin().x()) * sin(tf::getYaw(transform_lidar_grid.getRotation())) + (temp_cloud->points[i].y - transform_lidar_grid.getOrigin().y()) * cos(tf::getYaw(transform_lidar_grid.getRotation()));
	}

	transformed_cloud->header.frame_id = "occupancy_grid";

	*temp_2_cloud = *transformed_cloud;
	temp_2_cloud->points.clear();

	ROS_DEBUG_STREAM( "Create grid" ); 
	for ( int i = 0; i < transformed_cloud->size(); i++)
	{
		x_pos = rint(temp_cloud->points[i].x/voxelSize)*voxelSize;
		y_pos = rint(temp_cloud->points[i].y/voxelSize)*voxelSize;
		z_pos = rint(temp_cloud->points[i].z/voxelSize)*voxelSize;

                if (((x_pos > -(gridSize*voxelSize)/2) && (x_pos < (gridSize*voxelSize)/2) && (y_pos > -(gridSize*voxelSize)/2) && (y_pos < (gridSize*voxelSize)/2) && (z_pos < -0.5) && (z_pos > -3)) && not((x_pos > (BoatLengthOffset - BoatLength/2)) && (x_pos < (BoatLengthOffset + BoatLength/2)) && (y_pos > (BoatWidthOffset - BoatWidth/2)) && (y_pos < (BoatWidthOffset + BoatWidth/2))))
		{
			x_pos_to_map = rint(transformed_cloud->points[i].x/voxelSize)*voxelSize;
			y_pos_to_map = rint(transformed_cloud->points[i].y/voxelSize)*voxelSize;
			ROS_DEBUG_STREAM("Point: (" << x_pos << ", " << y_pos << ") on the map is (" << x_pos_to_map << ", " << y_pos_to_map << ") mapped to (" << x_pos_to_map-map_center_x << ", " << y_pos_to_map-map_center_y<< ")" );
			ROS_DEBUG_STREAM("Map value is: " << (int)grid_map.data[(floor((y_pos_to_map-map_center_y)/resolution)-1) * map_width + floor((x_pos_to_map-map_center_x)/resolution)] );
			if ((int)weighted_map.data[(floor((y_pos_to_map-map_center_y)/resolution)-1) * map_width + floor((x_pos_to_map-map_center_x)/resolution)] < 99)
			{
				ROS_DEBUG_STREAM("Map is not occupied");
				temp_2_cloud->push_back(transformed_cloud->points[i]);
				grid[(int)((x_pos+(gridSize*voxelSize)/2)/voxelSize)][(int)((y_pos+(gridSize*voxelSize)/2)/voxelSize)] = grid[(int)((x_pos+(gridSize*voxelSize)/2)/voxelSize)][(int)((y_pos+(gridSize*voxelSize)/2)/voxelSize)] + 1;
				VoxelFound = true;
			}
		}
	}

	Transformed_pcl_pub.publish(temp_2_cloud);

	ROS_DEBUG_STREAM( "Setup markers" ); 
	visualization_msgs::Marker points;
	points.header.frame_id = "lidar_link";
	points.header.stamp = ros::Time::now();
	points.ns = "points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = voxelSize;
	points.scale.y = voxelSize;
	points.color.g = 1.0;
	points.color.a = 1.0;

	visualization_msgs::MarkerArray Markers;
	visualization_msgs::Marker oval;
	oval.header.frame_id = "occupancy_grid";
	oval.header.stamp = ros::Time::now();
	oval.ns = "ovals";
	oval.action = visualization_msgs::Marker::ADD;
	oval.pose.orientation.w = 1.0;
	oval.id = 0;
	oval.type = visualization_msgs::Marker::CYLINDER;
	oval.color.b = 1.0;
	oval.color.a = 1.0;
	
	visualization_msgs::Marker Boat;
	Boat.header.frame_id = "lidar_link";
	Boat.header.stamp = ros::Time::now();
	Boat.ns = "Boat";
	Boat.action = visualization_msgs::Marker::ADD;
	Boat.pose.orientation.w = 1.0;
	Boat.id = 0;
	Boat.type = visualization_msgs::Marker::CUBE;
	Boat.scale.x = BoatLength;
	Boat.scale.y = BoatWidth;
	Boat.scale.z = 0.1;
	Boat.color.r = 1.0;
	Boat.color.a = 1.0;
	Boat.pose.position.x = BoatLengthOffset;
	Boat.pose.position.y = BoatWidthOffset;
	
	//Markers.markers.push_back(Boat);

	nautonomous_mpc_msgs::Obstacle obstacle;
	nautonomous_mpc_msgs::Obstacles obstacles;
	
	if (VoxelFound)
	{
		ROS_DEBUG_STREAM( "Create blobs" ); 
		for (int i = 0; i < gridSize ; i++)
		{
			for (int j = 0; j < gridSize ; j++)
			{
				if (grid[i][j] > pointTreshold)
				{	
					bool found = false;
					for (int k = 0; k < Blobs->size() ; k++)
					{
						ROS_DEBUG_STREAM( "Check if blob is close" ); 
						if (Blobs->at(k).isClose(i,j))
						{
							ROS_DEBUG_STREAM( "Add point to blob " << k ); 
							Blobs->at(k).add(i,j);
							found = true;
							break;
						}

					}
					if (!found)
					{
						ROS_DEBUG_STREAM( "Create new blob" ); 
						Blob* b = new Blob(i,j);
						Blobs->push_back(*b);
						ROS_DEBUG_STREAM( "New blob " << Blobs->size() << " created" ); 
					}
				}
			}
		}
		// Merge blobs
		int i = 0;
		int j = 0;
		bool BlobsMerged = false;
	
		ROS_DEBUG_STREAM( "Blobs before merging is: " << Blobs->size() ); 

		while (i < Blobs->size())
		{	
			ROS_DEBUG_STREAM( "iterator i: " << i ); 
			while ((j < i) && (i < Blobs->size()))
			{
				ROS_DEBUG_STREAM( "iterator j: " << j ); 
				std::vector<PVector>* MergePoints = new std::vector<PVector>(Blobs->at(i).getPoints());
				for (int k = 0; k < Blobs->at(i).getSize() ; k++)
				{
					if (Blobs->at(j).isClose(MergePoints->at(k).getX(),MergePoints->at(k).getY()))
					{
						ROS_DEBUG_STREAM( "Blobs are close" ); 
						for (int l = 0; l < Blobs->at(i).getSize() ; l++)

						{
							Blobs->at(j).add(MergePoints->at(l).getX(),MergePoints->at(l).getY());
						}
						ROS_DEBUG_STREAM( "Blobs merged" ); 
						Blobs->erase(Blobs->begin() + i);
						ROS_DEBUG_STREAM( "New size is: " << Blobs->size() ); 
						BlobsMerged = true;
						break;
					}
				}
				if (!BlobsMerged)
				{
					j++;
				}
				BlobsMerged = false;
			} 
			j = 0;			
			i++;
		}
	
		ROS_DEBUG_STREAM( "Blobs expended to: " << Blobs->size() ); 

		for (int i = 0; i < Blobs->size(); i++)
		{
			if (Blobs->at(i).getSize() > 2)
			{
				std::vector<PVector>* Points = new std::vector<PVector>(Blobs->at(i).getPoints());
				MatrixXd PointsMatrix = MatrixXd::Ones(Blobs->at(i).getSize(),2);

				for (int j = 0; j < Blobs->at(i).getSize(); j++)
				{
					geometry_msgs::Point p;
					p.x = Points->at(j).getX() * voxelSize - (gridSize * voxelSize)/2;
					p.y = Points->at(j).getY() * voxelSize - (gridSize * voxelSize)/2;
					p.z = 0;
					ROS_DEBUG_STREAM( "Blob: " << i << " Point: (" << p.x << "," << p.y << ")" ); 
					PointsMatrix(j,0) = Points->at(j).getX();
					PointsMatrix(j,1) = Points->at(j).getY();
					points.points.push_back(p);
				}


				// Use eigenvalue decomposition for the oval markers
				MatrixXd centered = PointsMatrix.rowwise() - PointsMatrix.colwise().mean();
				MatrixXd cov = (centered.adjoint() * centered)/ double(PointsMatrix.rows() -1);
			
				EigenSolver<MatrixXd> eigensolver(cov);
				MatrixXcd Eigenvalues = eigensolver.eigenvalues().asDiagonal();
				MatrixXcd Eigenvectors = eigensolver.eigenvectors();
	
				MatrixXd RealEigenvalues = Eigenvalues.real();
				MatrixXd RealEigenvectors = Eigenvectors.real();
		
				std::ptrdiff_t a,b,c,d;
				float smallest_eigenvalue = RealEigenvalues.diagonal().minCoeff(&a,&b);
				float largest_eigenvalue = RealEigenvalues.maxCoeff(&c,&d);
	
				VectorXd smallest_eigenvector = RealEigenvectors.col(a);
				VectorXd largest_eigenvector =  RealEigenvectors.col(c);

				angle = atan2((double)largest_eigenvector(1,0), (double)largest_eigenvector(0,0));
				if (angle < 0)
				{
					angle += 6.28;
				}
	
				float chisvalue = 3;
				AvgX = PointsMatrix.col(0).mean() - gridSize/2;
				AvgY = PointsMatrix.col(1).mean() - gridSize/2;
		
				first_principle_axis = chisvalue * sqrt(largest_eigenvalue);
				second_principle_axis = chisvalue * sqrt(smallest_eigenvalue);
			
				//oval.pose.position.x = AvgX * voxelSize;
				//oval.pose.position.y = AvgY * voxelSize;

				oval.pose.position.x = (AvgX * voxelSize - transform_lidar_grid.getOrigin().x()) * cos(tf::getYaw(transform_lidar_grid.getRotation())) + (AvgY * voxelSize - transform_lidar_grid.getOrigin().y()) * sin(tf::getYaw(transform_lidar_grid.getRotation()));
				oval.pose.position.y = -(AvgX * voxelSize - transform_lidar_grid.getOrigin().x()) * sin(tf::getYaw(transform_lidar_grid.getRotation())) + (AvgY * voxelSize - transform_lidar_grid.getOrigin().y()) * cos(tf::getYaw(transform_lidar_grid.getRotation()));


				oval.pose.position.z = 0;
				oval.pose.orientation = toQuaternion(0,0,angle - tf::getYaw(transform_lidar_grid.getRotation()));
				oval.scale.x = fmax(first_principle_axis * voxelSize * 2,voxelSize);
				oval.scale.y = fmax(second_principle_axis * voxelSize * 2,voxelSize);
				oval.scale.z = 0.5;
				oval.ns = i + 65;

				
				obstacle.pose.position.x = (AvgX * voxelSize - transform_lidar_grid.getOrigin().x()) * cos(tf::getYaw(transform_lidar_grid.getRotation())) + (AvgY * voxelSize - transform_lidar_grid.getOrigin().y()) * sin(tf::getYaw(transform_lidar_grid.getRotation()));
				obstacle.pose.position.y = -(AvgX * voxelSize - transform_lidar_grid.getOrigin().x()) * sin(tf::getYaw(transform_lidar_grid.getRotation())) + (AvgY * voxelSize - transform_lidar_grid.getOrigin().y()) * cos(tf::getYaw(transform_lidar_grid.getRotation()));
				obstacle.pose.position.z = 0;
				obstacle.pose.orientation = toQuaternion(0,0,angle - tf::getYaw(transform_lidar_grid.getRotation()));
				obstacle.major_semiaxis = fmax(first_principle_axis * voxelSize * 2,voxelSize);
				obstacle.minor_semiaxis = fmax(second_principle_axis * voxelSize * 2,voxelSize);

				ROS_DEBUG_STREAM( "Marker is FPA: " << first_principle_axis << " SPA: " << second_principle_axis << " angle: " << angle << " X: " << AvgX << " Y: " << AvgY ); 
				Markers.markers.push_back(oval);
				obstacles.obstacles.push_back(obstacle);
			}
		}

		Markers.markers.push_back(points);

		marker_pub.publish(Markers);

		ROS_INFO_STREAM( "The number of blobs is: " << Blobs->size() ); 

		Blobs->clear();
	}
	else
	{
		ROS_DEBUG_STREAM( "No voxels were found" ); 
	}
	
	while (obstacles.obstacles.size() < 10)
	{
		obstacles.obstacles.push_back(ghost_obstacle);
	}

	message_pub.publish(obstacles);
}

void EKF_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& ekf_msg)
{
	Boat_pos_x = rint(ekf_msg->x);
	Boat_pos_y = rint(ekf_msg->y);
/*
	transformation1(0,3) = Boat_pos_x;
	transformation1(1,3) = Boat_pos_y;

	transformation1(0,0) = cos(ekf_msg->theta);
	transformation1(0,1) = sin(ekf_msg->theta);
	transformation1(1,0) = -sin(ekf_msg->theta);
	transformation1(1,1) = cos(ekf_msg->theta);

	transformation2(0,0) = cos(ekf_msg->theta);
	transformation2(0,1) = sin(ekf_msg->theta);
	transformation2(1,0) = -sin(ekf_msg->theta);
	transformation2(1,1) = cos(ekf_msg->theta);*/
}

void make_weighted_map()
{
	weighted_map = grid_map;
	for (int i = 0; i < map_width; i++)
	{
		for (int j = 0; j < map_height; j++)
		{
			if ((int)grid_map.data[j * map_width + i] > 99)
			{
				weighted_map.data[j * map_width + i] = 100;
			}
		}
	}

	ROS_INFO_STREAM("Weighted map published");

	
	map_pub.publish(weighted_map);
}

void map_cb (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	ROS_DEBUG_STREAM( "map received" );
	grid_map = *map_msg;
	temp_map = grid_map;
	
	ROS_DEBUG_STREAM( "Data length is: " << grid_map.data.size() );
	map_width = (float)grid_map.info.width;
	map_height = (float)grid_map.info.height;
 
	map_center_x = (float)grid_map.info.origin.position.x;
	map_center_y = (float)grid_map.info.origin.position.y;

	resolution = (float)grid_map.info.resolution;
	ROS_DEBUG_STREAM( "Map center: " << map_center_x << ", " << map_center_y );
	ROS_DEBUG_STREAM( "Map size: " << map_width << " x " << map_height );

	make_weighted_map();
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Obstacle_detection");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	/*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 		{
	   ros::console::notifyLoggerLevelsChanged();
	}*/


	pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",1,cloud_cb);
	EKF_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/Ekf/next_state",1,EKF_cb);
	map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,map_cb);

	message_pub = nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",1);
	marker_pub = nh_private.advertise<visualization_msgs::MarkerArray>("markers",10);
	Transformed_pcl_pub = nh_private.advertise<sensor_msgs::PointCloud2>("transformed_pcl",10);
	map_pub = 	nh_private.advertise<nav_msgs::OccupancyGrid>("map_tree_opt",10);

	ROS_DEBUG_STREAM("Messages setup, waiting for tf");

	listener    = new tf::TransformListener();

	ROS_DEBUG_STREAM("Tf found");

	ros::Rate loop_rate(100);

	ROS_DEBUG_STREAM("Loop rate set");

	ghost_obstacle.pose.position.x = 1000;
	ghost_obstacle.pose.position.y = 1000;
	ghost_obstacle.pose.position.z = 0;
	ghost_obstacle.pose.orientation = toQuaternion(0,0,0);
	ghost_obstacle.major_semiaxis = 0.1;
	ghost_obstacle.minor_semiaxis = 0.1;

	ROS_DEBUG_STREAM("Ghost obstacle set");

	ros::Duration(1).sleep();

	ROS_INFO_STREAM("Build worked");
	while (ros::ok())
	{	
		ROS_DEBUG_STREAM("Lookup transform");
		listener->lookupTransform("/lidar_link", "/occupancy_grid", ros::Time(0), transform_lidar_grid);
		ROS_DEBUG_STREAM("Transform found");

		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_DEBUG_STREAM("ROS ended");
	return 0;
}
