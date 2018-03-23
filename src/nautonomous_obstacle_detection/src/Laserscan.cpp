#include <ros/ros.h>
#include <ros/console.h>

#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <iostream>

#include <nautonomous_obstacle_detection/PVector.h>
#include <nautonomous_obstacle_detection/Blob.h>
#include <nautonomous_obstacle_detection/Eigen/Eigenvalues>
#include <nautonomous_obstacle_detection/Quaternion_conversion.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <sensor_msgs/Imu.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <nautonomous_pose_msgs/PointWithCovarianceStamped.h>

#include <nautonomous_mpc_msgs/StageVariable.h>


// Namespaces
using namespace Eigen;
using namespace std;

// Parameters
int edgeThresh = 1;
int lowThreshold = 0;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";

float BoatWidth = 3;
float BoatLength = 5;
float BoatWidthOffset = 0; 
float BoatLengthOffset = 0;

bool UseJarvis = false;
bool VoxelFound = false;
float AvgX = -1;
float AvgY = -1;
float first_principle_axis = -1;
float second_principle_axis = -1;
double angle = -1;
float theta = 0;
float const voxelSize = 0.5;
int const gridSize = 100; // In number of squares (gridSize(m)/voxelSize)
int const pointTreshold = 1;
int const origin_x = 628604;
int const origin_y = 5802730;
float const resolution = 0.5;
float Boat_pos_x = 0;
float Boat_pos_y = 0;
float Boat_angle = 0;

sensor_msgs::Imu Imu;
sensor_msgs::LaserScan scan;

// Publishers
ros::Publisher marker_pub;
ros::Publisher message_pub;
ros::Publisher Transformed_pcl_pub;
// Subscribers
ros::Subscriber pc_sub;
ros::Subscriber EKF_sub;

visualization_msgs::MarkerArray Markers;
visualization_msgs::Marker points;
visualization_msgs::Marker oval;

// Initialization
float x_pos, y_pos, z_pos, x_pos_origin, y_pos_origin;
int grid[gridSize][gridSize];

std::vector<Blob>* Blobs = new std::vector<Blob>();

void scan_cb (const sensor_msgs::LaserScan::ConstPtr& Scan_msg)
{
	scan = *Scan_msg;
	Markers.markers.clear();
	points.points.clear();

	for (int i = 0; i < gridSize; i++){
		for (int j = 0; j < gridSize; j++){
			grid[i][j] = 0;
		}
	}

	for ( int i = 0; i < scan.ranges.size(); i++)
	{
		if (scan.ranges[i] < scan.range_max)
		{
			x_pos = rint(scan.ranges[i] * cos(Boat_angle + scan.angle_min + scan.angle_increment*i)/voxelSize)*voxelSize;
			y_pos = rint(scan.ranges[i] * sin(Boat_angle + scan.angle_min + scan.angle_increment*i)/voxelSize)*voxelSize;
			z_pos = 0;

			cout << "Pos: " << x_pos/voxelSize + gridSize/2*voxelSize << ", " << y_pos/voxelSize + gridSize/2*voxelSize << endl;
			if ((x_pos/voxelSize + gridSize/2*voxelSize) >= 0 && (x_pos/voxelSize + gridSize/2*voxelSize) < gridSize && (y_pos/voxelSize + gridSize/2*voxelSize) >= 0 && (y_pos/voxelSize + gridSize/2*voxelSize < gridSize))
			{
				if(!VoxelFound)
				{
					cout<< "point found"<<endl;
				}
				grid[(int)(x_pos/voxelSize + gridSize/2*voxelSize)][(int)(y_pos/voxelSize + gridSize/2*voxelSize)]++;
				VoxelFound = true;
			}
		}
	}

	cout << "Initialized grid" << endl;
	nautonomous_mpc_msgs::Obstacle obstacle;
	nautonomous_mpc_msgs::Obstacles obstacles;

	if (VoxelFound)
	{
		for (int i = 0; i < gridSize ; i++)
		{
			for (int j = 0; j < gridSize ; j++)
			{
				if (grid[i][j] > pointTreshold)
				{	
					bool found = false;
					for (int k = 0; k < Blobs->size() ; k++)
					{
						if (Blobs->at(k).isClose(i,j))
						{
							Blobs->at(k).add(i,j);
							found = true;
							break;
						}

					}
					if (!found)
					{
						Blob* b = new Blob(i,j);
						Blobs->push_back(*b);
					}
				}
			}
		}
		cout << "Blobs created, size is: " << Blobs->size() << endl;
		// Merge blobs
		int i = 0;
		int j = 0;
		bool BlobsMerged = false;
	
		while (i < Blobs->size())
		{	
			while ((j < i) && (i < Blobs->size()))
			{
				std::vector<PVector>* MergePoints = new std::vector<PVector>(Blobs->at(i).getPoints());
				for (int k = 0; k < Blobs->at(i).getSize() ; k++)
				{
					if (Blobs->at(j).isClose(MergePoints->at(k).getX(),MergePoints->at(k).getY()))
					{
						for (int l = 0; l < Blobs->at(i).getSize() ; l++)

						{
							Blobs->at(j).add(MergePoints->at(l).getX(),MergePoints->at(l).getY());
						}
						Blobs->erase(Blobs->begin() + i);
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
		cout << "Blobs merged, size is: " << Blobs->size() << endl;
		// Select 10 closest blobs

		// If Nblobs < 10, then duplicate the smallest blob (10-Nblob) times
		for (int i = Blobs->size(); i < 6 ;i++)
		{
			Blobs->push_back(Blobs->at(Blobs->size()-1));	
		}

		cout << "Blobs extended to " << Blobs->size() << endl;
		for (int i = 0; i < Blobs->size(); i++)
		{
			if (Blobs->at(i).getSize() > 1)
			{
				std::vector<PVector>* Points = new std::vector<PVector>(Blobs->at(i).getPoints());
				MatrixXd PointsMatrix = MatrixXd::Ones(Blobs->at(i).getSize(),2);
				cout << "Make pointmatrix" << endl;

				for (int j = 0; j < Blobs->at(i).getSize(); j++)
				{
					geometry_msgs::Point p;
					p.x = (Points->at(j).getX() - gridSize/2*voxelSize) * voxelSize + Boat_pos_x;
					p.y = (Points->at(j).getY() - gridSize/2*voxelSize) * voxelSize + Boat_pos_y;
					p.z = 0;
					PointsMatrix(j,0) = Points->at(j).getX();
					PointsMatrix(j,1) = Points->at(j).getY();
					points.points.push_back(p);
					cout << "Select point" << endl;
				}
				

				cout << "Points set" << endl;

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
	
				cout << "Eigenvalues found" << endl;
				float chisvalue = 3;
				AvgX = (PointsMatrix.col(0).mean() - gridSize/2*voxelSize) * voxelSize + Boat_pos_x;
				AvgY = (PointsMatrix.col(1).mean() - gridSize/2*voxelSize) * voxelSize + Boat_pos_y;
		
				first_principle_axis = chisvalue * sqrt(largest_eigenvalue);
				second_principle_axis = chisvalue * sqrt(smallest_eigenvalue);
			
				obstacle.state.pose.position.x = AvgX;
				obstacle.state.pose.position.y = AvgY;
				obstacle.state.pose.orientation.z = angle;
				obstacle.major_semiaxis = fmax(first_principle_axis * voxelSize + 2, 2);
				obstacle.minor_semiaxis = fmax(second_principle_axis * voxelSize + 2, 2);

				oval.pose.position.x = AvgX;
				oval.pose.position.y = AvgY;
				oval.pose.position.z = 1;
				oval.pose.orientation = toQuaternion(0,0,angle);
				oval.scale.x = obstacle.major_semiaxis * 2;
				oval.scale.y = obstacle.minor_semiaxis * 2;
				oval.scale.z = 0.5;
				oval.ns = i + 65;

				cout << "Marker is FPA: " << first_principle_axis << " SPA: " << second_principle_axis << " angle: " << angle << " X: " << AvgX << " Y: " << AvgY << endl;
				Markers.markers.push_back(oval);
				obstacles.obstacles.push_back(obstacle);
			}
		}

		Markers.markers.push_back(points);

		marker_pub.publish(Markers);
	
		//obstacles.Nobstacles = Blobs->size();
		message_pub.publish(obstacles);

		Blobs->clear();
	}
	else
	{
		cout << "No voxels were found" << endl;
	}
}

void odom_cb (const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	Boat_pos_x = rint(odom_msg->pose.pose.position.x);
	Boat_pos_y = rint(odom_msg->pose.pose.position.y);
	Boat_angle = toEulerAngle(odom_msg->pose.pose.orientation);
}


int main (int argc, char** argv)
{
	ros::init (argc, argv,"Obstacle_detection");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	points.header.frame_id = "odom";
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

	oval.header.frame_id = "odom";
	oval.header.stamp = ros::Time::now();
	oval.ns = "ovals";
	oval.action = visualization_msgs::Marker::ADD;
	oval.pose.orientation.w = 1.0;
	oval.id = 0;
	oval.type = visualization_msgs::Marker::CYLINDER;
	oval.color.b = 1.0;
	oval.color.a = 1.0;

	pc_sub = nh.subscribe<sensor_msgs::LaserScan>("/mybot/laser/scan",1,scan_cb);
	EKF_sub = nh.subscribe<nav_msgs::Odometry>("/odom",1,odom_cb);

	message_pub = nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",1);
	marker_pub = nh_private.advertise<visualization_msgs::MarkerArray>("markers",1);

	ros::spin();	
}
