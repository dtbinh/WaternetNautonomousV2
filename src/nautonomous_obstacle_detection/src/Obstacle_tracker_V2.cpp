#include <ros/ros.h>
#include <ros/console.h>

#include <Eigen/Dense>

#include <cmath>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>

using namespace std;
using namespace Eigen;


ros::Subscriber message_sub;

ros::Publisher message_pub;

bool previous_obstacles_received = false;
bool obstacle_is_new = false;

double dt = 0.1;
int obst_dist = 5;

double x_est, y_est, th_est, v_est;
double x_upd, y_upd, th_upd, v_upd;

double start_time=0;
double timed = 0;
int calls = 0;

nautonomous_mpc_msgs::Obstacles obstacles_set;
nautonomous_mpc_msgs::Obstacles obstacles_publish;
nautonomous_mpc_msgs::Obstacle ghost_obstacle;

std::vector<nautonomous_mpc_msgs::Obstacle>* available_obstacles = new std::vector<nautonomous_mpc_msgs::Obstacle>();
std::vector<int>* iterations = new std::vector<int>();
std::vector<nautonomous_mpc_msgs::Obstacle>* tracked_obstacles = new std::vector<nautonomous_mpc_msgs::Obstacle>();
std::vector<VectorXd>* obstacle_states = new std::vector<VectorXd>();

VectorXd Update_state( VectorXd current_state, VectorXd measured_pos)
{
	start_time = ros::Time::now().toSec();
	VectorXd estimated_state(4);
	VectorXd lon_lat_pos(2);
	VectorXd lon_lat_meas(2);
	VectorXd pos_error(2);
	VectorXd updated_state(4);
	MatrixXd K(4,2);

	estimated_state(0) = current_state(0) + dt * current_state(3) * cos(current_state(2));
	estimated_state(1) = current_state(1) + dt * current_state(3) * sin(current_state(2));
	estimated_state(2) = current_state(2);
	estimated_state(3) = current_state(3);

	ROS_DEBUG_STREAM("estimated_state: " << estimated_state);

	lon_lat_pos(0) = estimated_state(0) * cos(current_state(2)) + estimated_state(1) * sin(current_state(2));
	lon_lat_pos(1) = -estimated_state(0) * sin(current_state(2)) + estimated_state(1) * cos(current_state(2));

	ROS_DEBUG_STREAM("lon_lat_pos: " << lon_lat_pos);

	lon_lat_meas(0) = measured_pos(0) * cos(current_state(2)) + measured_pos(1) * sin(current_state(2));
	lon_lat_meas(1) = -measured_pos(0) * sin(current_state(2)) + measured_pos(1) * cos(current_state(2));

	ROS_DEBUG_STREAM("lon_lat_meas: " << lon_lat_meas);

	pos_error(0) = lon_lat_meas(0) - lon_lat_pos(0);
	pos_error(1) = lon_lat_meas(1) - lon_lat_pos(1);

	ROS_DEBUG_STREAM("pos_error: " << pos_error);

	K = MatrixXd::Zero(4,2);
	K(0,0) = 0.1 * cos(current_state(2));
	K(0,1) = -0.1 * sin(current_state(2));
	K(1,0) = 0.1 * sin(current_state(2));	
	K(1,1) = 0.1 * cos(current_state(2));
	K(2,1) = 0.1;
	K(3,0) = 0.1;

	updated_state(0) = estimated_state(0) + K(0,0) * pos_error(0) + K(0,1) * pos_error(1);
	updated_state(1) = estimated_state(1) + K(1,0) * pos_error(0) + K(1,1) * pos_error(1);
	updated_state(2) = estimated_state(2) + K(2,0) * pos_error(0) + K(2,1) * pos_error(1);
	updated_state(3) = estimated_state(3) + K(3,0) * pos_error(0) + K(3,1) * pos_error(1);

	ROS_DEBUG_STREAM("updated_state: " << updated_state);

	timed += ros::Time::now().toSec() - start_time;
	calls++;
	
	ROS_INFO_STREAM("Elapsed time per iteration: " << timed / calls );
	return updated_state;
}

float dist( nautonomous_mpc_msgs::Obstacle obstacle1, nautonomous_mpc_msgs::Obstacle obstacle2)
{
	return sqrt(pow(obstacle1.pose.position.x - obstacle2.pose.position.x,2) + pow(obstacle1.pose.position.y - obstacle2.pose.position.y,2));
}

float angle( nautonomous_mpc_msgs::Obstacle obstacle1, nautonomous_mpc_msgs::Obstacle obstacle2)
{
	double return_angle = atan2(obstacle2.pose.position.y - obstacle1.pose.position.y, obstacle2.pose.position.x - obstacle1.pose.position.x);
	while (return_angle < 0)
	{
		return_angle += 6.28;
	}
	return return_angle;
}

void obstacle_cb ( const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles_set = *obstacle_msg;

	for (int i = 0; i < obstacles_set.obstacles.size(); i++)
	{
		ROS_DEBUG_STREAM("Obstacle " << i << " position is (" << obstacles_set.obstacles.at(i).pose.position.x << ", " << obstacles_set.obstacles.at(i).pose.position.y << ")" );
	}

	for (int i = 0; i < tracked_obstacles->size(); i++)
	{
		tracked_obstacles->at(i).ns++;
	}

	if (!previous_obstacles_received)
	{
		previous_obstacles_received = true;
		for (int i = 0; i < obstacles_set.obstacles.size(); i++)
		{
			if ((obstacles_set.obstacles.at(i).pose.position.x == obstacles_set.obstacles.at(i).pose.position.y) && (obstacles_set.obstacles.at(i).pose.position.x == 100))
			{
				//
			}
			else 
			{	
				obstacles_set.obstacles.at(i).ns = 0;
				available_obstacles->push_back(obstacles_set.obstacles.at(i));
				iterations->push_back(1);
			}
			ROS_DEBUG_STREAM("First obstacles found");
		}
	}
	else
	{
		ROS_DEBUG_STREAM(obstacles_set.obstacles.size() << " obstacles found");
		ROS_DEBUG_STREAM(available_obstacles->size() << " obstacles are open");
		ROS_DEBUG_STREAM(tracked_obstacles->size() << " obstacles are being tracked");
		for (int i = 0; i < obstacles_set.obstacles.size(); i++)
		{
			if ((obstacles_set.obstacles.at(i).pose.position.x == obstacles_set.obstacles.at(i).pose.position.y) && (obstacles_set.obstacles.at(i).pose.position.x == 100))
			{
				//
			}
			else 
			{
				obstacle_is_new = true;
				for (int j = 0; j < tracked_obstacles->size(); j++)
				{
					if ( dist(obstacles_set.obstacles.at(i), tracked_obstacles->at(j)) < obst_dist)
					{
						tracked_obstacles->at(j).ns = 0;

						ROS_DEBUG_STREAM("Obstacle is being tracked");

						VectorXd current_state(4);
						VectorXd measured_pos(2);
						current_state = obstacle_states->at(j);
						measured_pos(0) = obstacles_set.obstacles.at(i).pose.position.x;
						measured_pos(1) = obstacles_set.obstacles.at(i).pose.position.y;

	    					obstacle_states->at(j) = Update_state( current_state, measured_pos);

						obstacle_is_new = false;
						tracked_obstacles->at(j).pose.position.x = obstacle_states->at(j)(0);
						tracked_obstacles->at(j).pose.position.y = obstacle_states->at(j)(1);
						tracked_obstacles->at(j).pose.orientation.z = obstacle_states->at(j)(2);
						tracked_obstacles->at(j).twist.linear.x = obstacle_states->at(j)(3);
						ROS_DEBUG_STREAM("Estimated tracked obstacle position is: (" << tracked_obstacles->at(j).pose.position.x << ", " << tracked_obstacles->at(j).pose.position.y << ") at a velocity of: " << tracked_obstacles->at(j).twist.linear.x << " with an angle of: " << tracked_obstacles->at(j).pose.orientation.z);
						break;	
					}
				}
				for (int j = 0; j < available_obstacles->size(); j++)
				{
					if ( dist(obstacles_set.obstacles.at(i), available_obstacles->at(j)) < obst_dist)
					{
						obstacle_is_new = false;
						ROS_DEBUG_STREAM("Close obstacle found");
						if (iterations->at(j) < 10)
						{
							available_obstacles->at(j).twist.linear.x = (available_obstacles->at(j).twist.linear.x * (iterations->at(j)-1) + dist(obstacles_set.obstacles.at(i), available_obstacles->at(j)) / dt ) / (iterations->at(j));
							available_obstacles->at(j).pose.orientation.z = (available_obstacles->at(j).pose.orientation.z * (iterations->at(j)-1) + angle(available_obstacles->at(j), obstacles_set.obstacles.at(i))) / (iterations->at(j));
							ROS_INFO_STREAM("Angle at iteration " << iterations->at(j) << " is : " << angle(available_obstacles->at(j), obstacles_set.obstacles.at(i)));
							iterations->at(j)++;
							available_obstacles->at(j).pose.position.x = obstacles_set.obstacles.at(i).pose.position.x;
							available_obstacles->at(j).pose.position.y = obstacles_set.obstacles.at(i).pose.position.y;
							ROS_INFO_STREAM("Estimated available obstacle position is: (" << available_obstacles->at(j).pose.position.x << ", " << available_obstacles->at(j).pose.position.y << ") at a velocity of: " << available_obstacles->at(j).twist.linear.x << " with an angle of: " << available_obstacles->at(j).pose.orientation.z);
						}
						else if (iterations->at(j) == 10)
						{
							ROS_DEBUG_STREAM("Obstacle is close at 10. Make obstacle tracked");
							available_obstacles->at(j).twist.linear.x = (available_obstacles->at(j).twist.linear.x * (iterations->at(j)-1) + dist(obstacles_set.obstacles.at(i), available_obstacles->at(j)) / dt ) / (iterations->at(j));
							available_obstacles->at(j).pose.orientation.z = (available_obstacles->at(j).pose.orientation.z * (iterations->at(j)-1) + angle(available_obstacles->at(j), obstacles_set.obstacles.at(i))) / (iterations->at(j));
							available_obstacles->at(j).pose.position.x = obstacles_set.obstacles.at(i).pose.position.x;
							available_obstacles->at(j).pose.position.y = obstacles_set.obstacles.at(i).pose.position.y;
							tracked_obstacles->push_back(available_obstacles->at(j));
							ROS_DEBUG_STREAM("Estimated available obstacle position is: (" << available_obstacles->at(j).pose.position.x << ", " << available_obstacles->at(j).pose.position.y << ") at a velocity of: " << available_obstacles->at(j).twist.linear.x << " with an angle of: " << available_obstacles->at(j).pose.orientation.z);
			
							VectorXd x(4);
							x(0) = available_obstacles->at(j).pose.position.x;
							x(1) = available_obstacles->at(j).pose.position.y;
							x(2) = available_obstacles->at(j).pose.orientation.z;
							x(3) = available_obstacles->at(j).twist.linear.x;
							obstacle_states->push_back(x);
							available_obstacles->erase(available_obstacles->begin() + j);
							iterations->erase(iterations->begin() + j);
						}
						break;
					}
				}
				if (obstacle_is_new)
				{
					ROS_DEBUG_STREAM("New obstacle found");
					obstacles_set.obstacles.at(i).ns = 0;
					available_obstacles->push_back(obstacles_set.obstacles.at(i));
					iterations->push_back(1);
				}
			}
		}	
	}

	for (int i = 0; i < tracked_obstacles->size(); i++)
	{
		ROS_DEBUG_STREAM("Counter for obstacle " << i << " is equal to " << tracked_obstacles->at(i).ns );
		if (tracked_obstacles->at(i).ns > 100)
		{
			ROS_DEBUG_STREAM("Obstacle " << i << " is not seen for too long and is erased" );
			tracked_obstacles->erase(tracked_obstacles->begin() + i);
			obstacle_states->erase(obstacle_states->begin() + i);
			i--;
			ROS_DEBUG_STREAM("i is set back to " << i );
		}
		else if (tracked_obstacles->at(i).ns > 0)
		{
			ROS_DEBUG_STREAM("Obstacle " << i << " is not seen for the " << tracked_obstacles->at(i).ns << "th time" );
		}
	
	}

	for (int i = 0; i < tracked_obstacles->size(); i++)
	{
		if (tracked_obstacles->at(i).ns > 0)
		{
			tracked_obstacles->at(i).pose.position.x += dt * tracked_obstacles->at(i).twist.linear.x * cos(tracked_obstacles->at(i).pose.orientation.z);
			tracked_obstacles->at(i).pose.position.y += dt * tracked_obstacles->at(i).twist.linear.x * sin(tracked_obstacles->at(i).pose.orientation.z);
			obstacle_states->at(i)(0) = tracked_obstacles->at(i).pose.position.x;
			obstacle_states->at(i)(1) = tracked_obstacles->at(i).pose.position.y;
		}
	}
	
	obstacles_publish.obstacles.clear();

	for (int i = 0; i < tracked_obstacles->size(); i++)
	{
		obstacles_publish.obstacles.push_back(tracked_obstacles->at(i));
		ROS_DEBUG_STREAM("Obstacle " << i << " published");
	}
	for (int i = 0; i < available_obstacles->size(); i++)
	{
		obstacles_publish.obstacles.push_back(available_obstacles->at(i));
	}	
	for (int i = obstacles_publish.obstacles.size(); i < 3; i++)
	{
		obstacles_publish.obstacles.push_back(ghost_obstacle);
	}
	message_pub.publish(obstacles_publish);
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Obstacle_tracking");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

        /*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 		{
	   ros::console::notifyLoggerLevelsChanged();
        }*/

	ghost_obstacle.major_semiaxis = 0.1;
	ghost_obstacle.minor_semiaxis = 0.1;
	ghost_obstacle.pose.position.x = 0;
	ghost_obstacle.pose.position.y = 0;
	ghost_obstacle.twist.linear.x = 0;
	ghost_obstacle.pose.orientation.z = 0;

	message_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles",1,obstacle_cb);
	message_pub = nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",1);

	ros::spin();	
}
