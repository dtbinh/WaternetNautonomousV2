#include <ros/ros.h>
#include <ros/console.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <cmath>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <nautonomous_obstacle_detection/Quaternion_conversion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;
using namespace Eigen;

ros::Subscriber message_sub;

ros::Publisher message_pub;

bool previous_obstacles_received = false;
bool obstacle_is_new = false;

double dt = 0.1;
int obst_dist = 2;

double x_est, y_est, th_est, v_est;
double x_upd, y_upd, th_upd, v_upd;

double start_time=0;
double timed = 0;
int calls = 0;

nautonomous_mpc_msgs::Obstacles obstacles_set;
nautonomous_mpc_msgs::Obstacles obstacles_publish;
nautonomous_mpc_msgs::Obstacle ghost_obstacle;

geometry_msgs::Pose Obstacle_pose;
geometry_msgs::PoseArray Obstacles_pose;

Matrix<double,10,10> temp_block;
Matrix<double,1,9> temp_block_2;
Matrix<double,100,10> tracked_obstacles_x;
Matrix<double,100,10> tracked_obstacles_y;
std::vector<int>* obstacle_number_tracker = new std::vector<int>();

float dist( nautonomous_mpc_msgs::Obstacle obstacle1, double x_pos, double y_pos)
{
	return sqrt(pow(obstacle1.pose.position.x - x_pos,2) + pow(obstacle1.pose.position.y - y_pos,2));
}

float dist_2( double x_pos_1, double y_pos_1, double x_pos_2, double y_pos_2)
{
	return sqrt(pow(x_pos_1 - x_pos_2,2) + pow(y_pos_1 - y_pos_2,2));
}

float angle( double x_pos_1, double y_pos_1, double x_pos_2, double y_pos_2)
{
	double return_angle = atan2(y_pos_1 - y_pos_2, x_pos_1 - x_pos_2);
	return return_angle;
}

void obstacle_cb ( const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles_set = *obstacle_msg;

	for (int i = 0; i < obstacles_set.obstacles.size(); i++)
	{
		ROS_DEBUG_STREAM("Obstacle " << i << " position is (" << obstacles_set.obstacles.at(i).pose.position.x << ", " << obstacles_set.obstacles.at(i).pose.position.y << ")" );
	}

	for (int i = 0; i < obstacle_number_tracker->size(); i++)
	{
		obstacle_number_tracker->at(i)++;
	}

	ROS_DEBUG_STREAM(obstacles_set.obstacles.size() << " obstacles found");
	ROS_DEBUG_STREAM(obstacle_number_tracker->size() << " obstacles are being tracked");
	for (int i = 0; i < obstacles_set.obstacles.size(); i++)
	{
		obstacle_is_new = true;
		for (int j = 0; j < obstacle_number_tracker->size(); j++)
		{
			if (dist(obstacles_set.obstacles.at(i),tracked_obstacles_x(j,0),tracked_obstacles_y(j,0)) < obst_dist)
			{
				temp_block_2 = tracked_obstacles_x.block<1,9>(j,0);
				tracked_obstacles_x.block<1,9>(j,1) = temp_block_2;
				temp_block_2 = tracked_obstacles_y.block<1,9>(j,0);
				tracked_obstacles_y.block<1,9>(j,1) = temp_block_2;
				tracked_obstacles_x(j,0) = obstacles_set.obstacles.at(i).pose.position.x;
				tracked_obstacles_y(j,0) = obstacles_set.obstacles.at(i).pose.position.y;
				obstacle_is_new = false;
				obstacle_number_tracker->at(j) = 0;
			}
		}
		if (obstacle_is_new)
		{
			for (int j = 0; j < 10; j++)
			{
				tracked_obstacles_x(obstacle_number_tracker->size(),j) = obstacles_set.obstacles.at(i).pose.position.x;
				tracked_obstacles_y(obstacle_number_tracker->size(),j) = obstacles_set.obstacles.at(i).pose.position.y;
			}
			obstacle_number_tracker->push_back(0);
		}
	}

	for (int i = 0; i < obstacle_number_tracker->size(); i++)
	{
		ROS_DEBUG_STREAM("Obstacle " << i << " is not seen for " << obstacle_number_tracker->at(i) << " turns");
		if (obstacle_number_tracker->at(i) > 9)
		{
			for (int j = 0; j < (99-i); j++)
			{
				tracked_obstacles_x.block<1,10>(i+j,0) = tracked_obstacles_x.block<1,10>(i+j+1,0);
				tracked_obstacles_y.block<1,10>(i+j,0) = tracked_obstacles_y.block<1,10>(i+j+1,0);
			}
			obstacle_number_tracker->erase(obstacle_number_tracker->begin() + i);
			i--;
		}
	}
	ROS_DEBUG_STREAM(tracked_obstacles_x);
	ROS_DEBUG_STREAM(tracked_obstacles_y);
	Obstacles_pose.poses.clear();
	for (int i = 0; i < obstacle_number_tracker->size(); i++)
	{
		double velocity_x = 0;
		double velocity_y = 0;
		for (int j = 0; j < 9; j++)
		{
			velocity_x+= dist_2(tracked_obstacles_x(i,j), tracked_obstacles_y(i,j),tracked_obstacles_x(i,j+1), tracked_obstacles_y(i,j+1)) * cos(angle(tracked_obstacles_x(i,j), tracked_obstacles_y(i,j),tracked_obstacles_x(i,j+1), tracked_obstacles_y(i,j+1)));
			velocity_y+= dist_2(tracked_obstacles_x(i,j), tracked_obstacles_y(i,j),tracked_obstacles_x(i,j+1), tracked_obstacles_y(i,j+1)) * sin(angle(tracked_obstacles_x(i,j), tracked_obstacles_y(i,j),tracked_obstacles_x(i,j+1), tracked_obstacles_y(i,j+1)));
		}
		velocity_x/=(9 * dt);
		velocity_y/=(9 * dt);

		if ((velocity_x > 1) || (velocity_y > 1))
		{
			Obstacle_pose.position.x = tracked_obstacles_x(i,0);
			Obstacle_pose.position.y = tracked_obstacles_y(i,0);
			Obstacle_pose.orientation = toQuaternion(0,0,angle(velocity_x,velocity_y,0,0));
			Obstacles_pose.poses.push_back(Obstacle_pose);
		}
		message_pub.publish(Obstacles_pose);
		ROS_DEBUG_STREAM("Obstacle " << i << " has an x-velocity of " << velocity_x);
		ROS_DEBUG_STREAM("Obstacle " << i << " has an y-velocity of " << velocity_y);
	}
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Obstacle_tracking");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 		{
	   ros::console::notifyLoggerLevelsChanged();
        }

	Obstacles_pose.header.frame_id = "occupancy_grid";
	ghost_obstacle.major_semiaxis = 0.1;
	ghost_obstacle.minor_semiaxis = 0.1;
	ghost_obstacle.pose.position.x = 0;
	ghost_obstacle.pose.position.y = 0;
	ghost_obstacle.twist.linear.x = 0;
	ghost_obstacle.pose.orientation.z = 0;

	message_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles",1,obstacle_cb);
	message_pub = nh_private.advertise<geometry_msgs::PoseArray>("obstacles_poses",1);

	ros::spin();	
}
