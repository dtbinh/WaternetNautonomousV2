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

#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
//#include <Eigen/Eigenvalues>
#include <iostream>

#include <nautonomous_obstacle_detection/PVector.h>
#include <nautonomous_obstacle_detection/Blob.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <nautonomous_pose_msgs/PointWithCovarianceStamped.h>

#include <nautonomous_mpc_msgs/StageVariable.h>


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
int const gridSize = 160; // In number of squares (gridSize(m)/voxelSize)
int const pointTreshold = 5;
int const origin_x = 628604;
int const origin_y = 5802730;
float const resolution = 0.5;
int Boat_pos_x = 0;
int Boat_pos_y = 0;

sensor_msgs::Imu Imu;

Matrix4f transformation1 = Eigen::Matrix4f::Identity();
Matrix4f transformation2 = Eigen::Matrix4f::Identity();

// Publishers
ros::Publisher marker_pub;
ros::Publisher message_pub;
ros::Publisher Transformed_pcl_pub;
// Subscribers
ros::Subscriber pc_sub;
ros::Subscriber EKF_sub;

// Initialization
float x_pos, y_pos, z_pos, x_pos_origin, y_pos_origin;
int grid[gridSize][gridSize];

std::vector<Blob>* Blobs = new std::vector<Blob>();

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Create image
	src = imread("/home/waternet/programmeren/nautonomous/WaternetNautonomous/src/nautonomous_configuration/config/navigation/map/amsterdam_cropped_debug_rotated.png" , IMREAD_COLOR);
	cout << "Image size is " << src.cols << "x" << src.rows <<endl;

	cout << "Initialize grid" << endl;
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

	Transformed_pcl_pub.publish(transformed_cloud);

	cout << "Create grid" << endl;
	for ( int i = 0; i < transformed_cloud->size(); i++)
	{
		x_pos = rint(transformed_cloud->points[i].x/voxelSize)*voxelSize;
		y_pos = rint(transformed_cloud->points[i].y/voxelSize)*voxelSize;
		z_pos = rint(transformed_cloud->points[i].z/voxelSize)*voxelSize;

		if (((x_pos > -(gridSize*voxelSize)/2) && (x_pos < (gridSize*voxelSize)/2) && (y_pos > -(gridSize*voxelSize)/2) && (y_pos < (gridSize*voxelSize)/2) && (z_pos < 3) && (z_pos > 0)) && not((x_pos > (BoatLengthOffset - BoatLength/2)) && (x_pos < (BoatLengthOffset + BoatLength/2)) && (y_pos > (BoatWidthOffset - BoatWidth/2)) && (y_pos < (BoatWidthOffset + BoatWidth/2))))
		{
			grid[(int)((x_pos+(gridSize*voxelSize)/2)/voxelSize)][(int)((y_pos+(gridSize*voxelSize)/2)/voxelSize)] = grid[(int)((x_pos+(gridSize*voxelSize)/2)/voxelSize)][(int)((y_pos+(gridSize*voxelSize)/2)/voxelSize)] + 1;
			VoxelFound = true;
		}

		/*// ToDo fix frames
		cout << "Point in origin frame: (" << << "," << << ") and in lidar frame "
		if ((x_pos_origin > 0 ) && (x_pos_origin < src.cols) && (y_pos_origin > 0 ) && (y_pos_origin < src.rows))
		{
			if ((int)src.at<uchar>(y_pos_origin*2,x_pos_origin*2) > 1)
			{
				if (((x_pos > -(gridSize*voxelSize)/2) && (x_pos < (gridSize*voxelSize)/2) && (y_pos > -(gridSize*voxelSize)/2) && (y_pos < (gridSize*voxelSize)/2) && (z_pos < 0) && (z_pos > -5)) && not((x_pos > (BoatLengthOffset - BoatLength/2)) && (x_pos < (BoatLengthOffset + BoatLength/2)) && (y_pos > (BoatWidthOffset - BoatWidth/2)) && (y_pos < (BoatWidthOffset + BoatWidth/2))))
		{
				grid[(int)((x_pos+(gridSize*voxelSize)/2)/voxelSize)][(int)((y_pos+(gridSize*voxelSize)/2)/voxelSize)] = grid[(int)((x_pos+(gridSize*voxelSize)/2)/voxelSize)][(int)((y_pos+(gridSize*voxelSize)/2)/voxelSize)] + 1;
				VoxelFound = true;
		}
			}
		}*/
	}

	cout << "Setup markers" << endl;
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
	oval.header.frame_id = "lidar_link";
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
	
	Markers.markers.push_back(Boat);



	nautonomous_mpc_msgs::Obstacle obstacle;
	nautonomous_mpc_msgs::Obstacles obstacles;


		
	
	if (VoxelFound)
	{
		cout << "Create blobs" << endl;
		for (int i = 0; i < gridSize ; i++)
		{
			for (int j = 0; j < gridSize ; j++)
			{
				if (grid[i][j] > pointTreshold)
				{	
					bool found = false;
					for (int k = 0; k < Blobs->size() ; k++)
					{
						cout << "Check if blob is close" << endl;
						if (Blobs->at(k).isClose(i,j))
						{
							cout << "Add point to blob " << k << endl;
							Blobs->at(k).add(i,j);
							found = true;
							break;
						}

					}
					if (!found)
					{
						cout << "Create new blob" << endl;
						Blob* b = new Blob(i,j);
						Blobs->push_back(*b);
						cout << "New blob " << Blobs->size() << " created" << endl;
					}
				}
			}
		}
		// Merge blobs
		int i = 0;
		int j = 0;
		bool BlobsMerged = false;
	
		cout << "Blobs before merging is: " << Blobs->size() << endl;

		while (i < Blobs->size())
		{	
			cout << "iterator i: " << i << endl;
			while ((j < i) && (i < Blobs->size()))
			{
				cout << "iterator j: " << j << endl;
				std::vector<PVector>* MergePoints = new std::vector<PVector>(Blobs->at(i).getPoints());
				for (int k = 0; k < Blobs->at(i).getSize() ; k++)
				{
					if (Blobs->at(j).isClose(MergePoints->at(k).getX(),MergePoints->at(k).getY()))
					{
						cout << "Blobs are close" << endl;
						for (int l = 0; l < Blobs->at(i).getSize() ; l++)

						{
							Blobs->at(j).add(MergePoints->at(l).getX(),MergePoints->at(l).getY());
						}
						cout << "Blobs merged" << endl;
						Blobs->erase(Blobs->begin() + i);
						cout << "New size is: " << Blobs->size() << endl;
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

		// Select 10 closest blobs

		// If Nblobs < 10, then duplicate the smallest blob (10-Nblob) times
		for (int i = Blobs->size(); i < 6 ;i++)
		{
			Blobs->push_back(Blobs->at(Blobs->size()-1));	
		}
	
		cout << "Blobs expended to: " << Blobs->size() << endl;

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
					cout << "Blob: " << i << " Point: (" << p.x << "," << p.y << ")" << endl;
					PointsMatrix(j,0) = Points->at(j).getX();
					PointsMatrix(j,1) = Points->at(j).getY();
					points.points.push_back(p);
				}

				if (!UseJarvis)
				{
		
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
			
				}
				else
				{
					// Use Jarvis
					cout << "Start Jarvis" << endl;
					MatrixXd ConvexSet(1,3);
					MatrixXd TempSet(1,3);
					ConvexSet (0,0) = 500;
					ConvexSet (0,1) = 500;
					ConvexSet (0,2) = 0;

					// Find minimum corner point
					for (int j = 0; j < Blobs->at(i).getSize() ;j++)
					{
						if ((PointsMatrix(j,0) < ConvexSet (0,0)) && (PointsMatrix(j,1) < ConvexSet (0,1)))
						{
							ConvexSet (0,0) = PointsMatrix(j,0);
							ConvexSet (0,1) = PointsMatrix(j,1);
						}
					}			
					// Find second point
					float NewAngle = -6.28;
					float PrevAngle = -6.28;

					TempSet = ConvexSet;
					ConvexSet.resize(ConvexSet.rows()+1,3);
					ConvexSet.topRows(TempSet.rows()) = TempSet;
					TempSet.resize(ConvexSet.rows(),3);
					for (int j = 0; j < Blobs->at(i).getSize() ;j++)
					{
						if ((PointsMatrix(j,1) == ConvexSet(0,1)) && (PointsMatrix(j,0) == ConvexSet(0,0)))
						{
							continue;
						}
						else
						{
							NewAngle = atan2((double)(PointsMatrix(j,1) - ConvexSet(0,1)), (double)(PointsMatrix(j,0) - ConvexSet(0,0)));
							if (NewAngle > PrevAngle)
							{
								PrevAngle = NewAngle;
								ConvexSet (1,0) = PointsMatrix(j,0);
								ConvexSet (1,1) = PointsMatrix(j,1);
								ConvexSet (1,2) = NewAngle;
							}
						}
					}	
					PrevAngle = -6.28;
					int k = 1;
					do 
					{
						TempSet = ConvexSet;
						ConvexSet.resize(ConvexSet.rows()+1,3);
						ConvexSet.topRows(TempSet.rows()) = TempSet;
						TempSet.resize(ConvexSet.rows(),3);
						k++;
						for (int j = 0; j < Blobs->at(i).getSize() ; j++)
						{
							if (((PointsMatrix(j,1) == ConvexSet(k-1,1)) && (PointsMatrix(j,0) == ConvexSet(k-1,0))) || ((ConvexSet(k-2,1) == PointsMatrix(j,1) ) && (ConvexSet(k-2,0) == PointsMatrix(j,0))))
							{
								continue;
							}
							else
							{
								NewAngle = atan2((double)(PointsMatrix(j,1) - ConvexSet(k-1,1)), (double)(PointsMatrix(j,0) - ConvexSet(k-1,0))) - atan2((double)(ConvexSet(k-2,1)- ConvexSet(k-1,1)), (double)(ConvexSet(k-2,0) - ConvexSet(k-1,0)));
								
								if (NewAngle < 0)
								{
									NewAngle += 6.28;
								}

								if (NewAngle > PrevAngle)
								{
									PrevAngle = NewAngle;
									ConvexSet(k,0) = PointsMatrix(j,0);
									ConvexSet(k,1) = PointsMatrix(j,1);
									ConvexSet(k,2) = NewAngle;
								}
							}
						}
						PrevAngle = -6.28;
						
					} while (not((ConvexSet(k,0) == ConvexSet(0,0)) && (ConvexSet(k,1) == ConvexSet(0,1))));

					MatrixXd TempVec(ConvexSet.rows(),2);
					TempVec.leftCols(1) = ConvexSet(0,0)*MatrixXd::Ones(ConvexSet.rows(),1);
					TempVec.rightCols(1) = ConvexSet(0,1)*MatrixXd::Ones(ConvexSet.rows(),1);
					MatrixXd Transform(3,3);
					MatrixXd FullTransform(3,3);
					MatrixXd tempTransform(3,3);
					tempTransform = MatrixXd::Identity(3,3);
					MatrixXd TransformedConvexSet(ConvexSet.rows(),2) ;
					TransformedConvexSet.leftCols(2) = ConvexSet.leftCols(2) - TempVec ;
					TempVec.resize(3,1);
					Transform(2,0) = 0.0;
					Transform(2,1) = 0.0;
					Transform(2,2) = 1.0;

					float minArea = 10000.0;
					float minWidth = 0.0;
					float minHeight = 0.0;
					float minAngle = 0.0;
					float minCenterX = 0.0;
					float minCenterY = 0.0;
					float tempAngle = 0;
					float CenterX = 0;
					float CenterY = 0;
					for (int l = 1; l < ConvexSet.rows(); l++)
					{
						Transform(0,0) = cos(ConvexSet(l,2));
						Transform(0,1) = sin(ConvexSet(l,2));
						Transform(1,0) = -sin(ConvexSet(l,2));
						Transform(1,1) = cos(ConvexSet(l,2));
						Transform(0,2) = -(TransformedConvexSet(l,0) - TransformedConvexSet(l-1,0)) * cos(ConvexSet(l,2)) - (TransformedConvexSet(l,1) - TransformedConvexSet(l-1,1)) * sin(ConvexSet(l,2));
						Transform(1,2) = (TransformedConvexSet(l,0) - TransformedConvexSet(l-1,0)) * sin(ConvexSet(l,2)) - (TransformedConvexSet(l,1) - TransformedConvexSet(l-1,1)) * cos(ConvexSet(l,2));
						
						for (int j = 1; j < ConvexSet.rows(); j++)
						{
							TempVec(0,0) = TransformedConvexSet(j,0);
							TempVec(1,0) = TransformedConvexSet(j,1);
							TempVec(2,0) = 1.0;
							TempVec = Transform * TempVec;
							
							TransformedConvexSet(j,0) = TempVec(0,0);
							TransformedConvexSet(j,1) = TempVec(1,0);
						}

						tempTransform = Transform * tempTransform;
						tempAngle += ConvexSet(l,2);

						// Get max min
						float height = TransformedConvexSet.leftCols(1).maxCoeff() - TransformedConvexSet.leftCols(1).minCoeff();
						float width = TransformedConvexSet.rightCols(1).maxCoeff() - TransformedConvexSet.rightCols(1).minCoeff();		
						float relativeCenterX = (TransformedConvexSet.leftCols(1).maxCoeff() + TransformedConvexSet.leftCols(1).minCoeff())/2.0;
						float relativeCenterY = (TransformedConvexSet.rightCols(1).maxCoeff() + TransformedConvexSet.rightCols(1).minCoeff())/2.0;
						
						TempVec(0,0) = relativeCenterX;
						TempVec(1,0) = relativeCenterY;
						TempVec(2,0) = 1.0;
						TempVec = tempTransform.inverse() * TempVec;
						
						CenterX = TempVec(0,0);
						CenterY = TempVec(1,0);


						// Check greatest area
						if ((height * width) < minArea)
						{
							minArea = height * width;
							minWidth = width;
							minHeight = height;
							minAngle = tempAngle;
							minCenterX = CenterX;
							minCenterY = CenterY;
							FullTransform = tempTransform;
						}
					}
						
					AvgX = CenterX - gridSize/2 + ConvexSet(0,0);
					AvgY = CenterY - gridSize/2 + ConvexSet(0,1);

					// Get priciple axis	
					angle = fmod(minAngle,6.28);
					first_principle_axis = minHeight;
					second_principle_axis = minWidth;
				}
				oval.pose.position.x = AvgX * voxelSize;
				oval.pose.position.y = AvgY * voxelSize;
				oval.pose.position.z = 0;
				oval.pose.orientation.z = angle;
				oval.scale.x = first_principle_axis * voxelSize;
				oval.scale.y = second_principle_axis * voxelSize;
				oval.scale.z = 0.5;
				oval.ns = i + 65;

				
				obstacle.state.pose.position.x = AvgX * voxelSize;
				obstacle.state.pose.position.y = AvgY * voxelSize;
				obstacle.state.pose.position.z = 0;
				obstacle.state.pose.orientation.x = 0;
				obstacle.state.pose.orientation.y = 0;
				obstacle.state.pose.orientation.z = angle;
				obstacle.major_semiaxis = first_principle_axis * voxelSize;
				obstacle.minor_semiaxis = second_principle_axis * voxelSize;

				cout << "Marker is FPA: " << first_principle_axis << " SPA: " << second_principle_axis << " angle: " << angle << " X: " << AvgX << " Y: " << AvgY << endl;
				Markers.markers.push_back(oval);
				obstacles.obstacles.push_back(obstacle);
			}
		}

		Markers.markers.push_back(points);

		marker_pub.publish(Markers);
	
		obstacles.Nobstacles = Blobs->size();
		message_pub.publish(obstacles);

		cout << "The number of blobs is: " << Blobs->size() << endl;

		Blobs->clear();
	}
	else
	{
		cout << "No voxels were found" << endl;
	}
}

void EKF_cb (const nautonomous_mpc_msgs::StageVariable::ConstPtr& ekf_msg)
{
	Boat_pos_x = rint(ekf_msg->x);
	Boat_pos_y = rint(ekf_msg->y);

	transformation1(0,3) = -Boat_pos_x;
	transformation1(1,3) = -Boat_pos_y;

	transformation1(0,0) = cos(ekf_msg->theta);
	transformation1(0,1) = sin(ekf_msg->theta);
	transformation1(1,0) = -sin(ekf_msg->theta);
	transformation1(1,1) = cos(ekf_msg->theta);

	transformation2(0,0) = cos(ekf_msg->theta);
	transformation2(0,1) = sin(ekf_msg->theta);
	transformation2(1,0) = -sin(ekf_msg->theta);
	transformation2(1,1) = cos(ekf_msg->theta);
}


int main (int argc, char** argv)
{
	ros::init (argc, argv,"Obstacle_detection");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("/point_cloud",1,cloud_cb);
	EKF_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/Ekf/next_state",1,EKF_cb);

	message_pub = nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",1);
	marker_pub = nh_private.advertise<visualization_msgs::MarkerArray>("markers",10);
	Transformed_pcl_pub = nh_private.advertise<sensor_msgs::PointCloud2>("transformed_pcl",10);

	ros::spin();	
}
