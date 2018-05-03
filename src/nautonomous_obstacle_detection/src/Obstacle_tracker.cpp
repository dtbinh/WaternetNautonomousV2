#include <ros/ros.h>
#include <ros/console.h>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;

typedef boost::array<double, 20> state_type;

ros::Subscriber message_sub;

ros::Publisher message_pub;

bool previous_obstacles_received = false;
bool obstacle_is_new = false;

double dt = 1;
int obst_dist = 5;

double x_est, y_est, th_est, v_est;
double x_upd, y_upd, th_upd, v_upd;

nautonomous_mpc_msgs::Obstacles obstacles_set;
nautonomous_mpc_msgs::Obstacles obstacles_publish;

std::vector<nautonomous_mpc_msgs::Obstacle>* available_obstacles = new std::vector<nautonomous_mpc_msgs::Obstacle>();
std::vector<int>* iterations = new std::vector<int>();
std::vector<nautonomous_mpc_msgs::Obstacle>* tracked_obstacles = new std::vector<nautonomous_mpc_msgs::Obstacle>();
std::vector<state_type>* obstacle_states = new std::vector<state_type>();

MatrixXd A(4,4);
MatrixXd P(4,4);
MatrixXd X(4,1);
MatrixXd X_meas(2,1);
MatrixXd R = MatrixXd::Identity(2,2);
MatrixXd Q = MatrixXd::Identity(4,4);
MatrixXd I = MatrixXd::Identity(4,4);
MatrixXd K(4,2);
MatrixXd B = MatrixXd::Identity(2,4);
MatrixXd S(2,2);
MatrixXd DP(4,4);

VectorXd x_(4);

void ODE_constant_obstacle_velocity( const state_type &x , state_type &dxdt , double t )
{
	for (int i = 0; i < 4; i++)
	{
		x_(i) = x[i];
	}
	for (int i = 4; i < 20; i++)
	{
		P(floor((i-4)/4),fmod(i,4)) = x[i];
	}

	A = MatrixXd::Zero(4,4);

	dxdt[0] = x_(3) * cos(x_(2));
	dxdt[1] = x_(3) * sin(x_(2));
	dxdt[2] = 0;
	dxdt[3] = 0;

	A(0,2) = -x_(3) * sin(x_(2));
	A(0,3) = cos(x_(2));
	A(1,2) = x_(3) * cos(x_(2));
	A(1,3) = sin(x_(2));

	DP = A * P + P * A.transpose() + Q;

	for (int i = 4; i < 20; i++)
	{
		dxdt[i] = DP(floor((i-4)/4),fmod(i,4));
	}
}

void write_ODE( const state_type &x , const double t )
{
	// Do nothing
}

float dist( nautonomous_mpc_msgs::Obstacle obstacle1, nautonomous_mpc_msgs::Obstacle obstacle2)
{
	return sqrt(pow(obstacle1.pose.position.x - obstacle2.pose.position.x,2) + pow(obstacle1.pose.position.y - obstacle2.pose.position.y,2));
}

float angle( nautonomous_mpc_msgs::Obstacle obstacle1, nautonomous_mpc_msgs::Obstacle obstacle2)
{
	return atan2(obstacle2.pose.position.y - obstacle1.pose.position.y, obstacle2.pose.position.x - obstacle1.pose.position.x);
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

						state_type x;
						x = obstacle_states->at(j);

	    					integrate( ODE_constant_obstacle_velocity , x , 0.0 , dt , dt , write_ODE );

						for (int k = 0; k < 4; k++)
						{
							X(k) = x[k];
						}
						for (int k = 4; k < 20; k++)
						{
							P(floor((k-4)/4),fmod(k,4)) = x[k];
						}

						X_meas(0) = obstacles_set.obstacles.at(i).pose.position.x - X(0);
						X_meas(1) = obstacles_set.obstacles.at(i).pose.position.y - X(1);

						S = B * P * B.transpose() + R;
						K = P * B.transpose() * S.inverse();
						X = X + K * X_meas;
						P = (I - K * B) * P;

						for (int k = 0; k < 4; k++)
						{
							x[k] = X(k);
						}
						for (int k = 4; k < 20; k++)
						{
							x[k] = P(floor((k-4)/4),fmod(k,4));
						}
						obstacle_states->at(j) = x;
						obstacle_is_new = false;
						tracked_obstacles->at(j).pose.position.x = X(0);
						tracked_obstacles->at(j).pose.position.y = X(1);
						tracked_obstacles->at(j).pose.orientation.z = X(2);
						tracked_obstacles->at(j).twist.linear.x = X(3);
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
							iterations->at(j)++;
							available_obstacles->at(j).pose.position.x = obstacles_set.obstacles.at(i).pose.position.x;
							available_obstacles->at(j).pose.position.y = obstacles_set.obstacles.at(i).pose.position.y;
							ROS_DEBUG_STREAM("Estimated available obstacle position is: (" << available_obstacles->at(j).pose.position.x << ", " << available_obstacles->at(j).pose.position.y << ") at a velocity of: " << available_obstacles->at(j).twist.linear.x << " with an angle of: " << available_obstacles->at(j).pose.orientation.z);
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
			
							state_type x;
							x[0] = available_obstacles->at(j).pose.position.x;
							x[1] = available_obstacles->at(j).pose.position.y;
							x[2] = available_obstacles->at(j).pose.orientation.z ;
							x[3] = available_obstacles->at(j).twist.linear.x;
							for (int k = 4; k < 20; k++)
							{
								x[k] = 0;
							}
							x[4] = 1;
							x[9] = 1;
							x[14] = 1;
							x[19] = 1;
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
		if (tracked_obstacles->at(i).ns > 10)
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

	obstacles_publish.obstacles.clear();
	for (int i = 0; i < available_obstacles->size(); i++)
	{
		obstacles_publish.obstacles.push_back(available_obstacles->at(i));
	}
	for (int i = 0; i < tracked_obstacles->size(); i++)
	{
		obstacles_publish.obstacles.push_back(tracked_obstacles->at(i));
	}
	message_pub.publish(obstacles_publish);
}

int main (int argc, char** argv)
{
//	Q(2,2) = 0.0001;
//	Q(3,3) = 0.0001;

	ros::init (argc, argv,"Obstacle_tracking");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }

	message_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles",1,obstacle_cb);
	message_pub = nh_private.advertise<nautonomous_mpc_msgs::Obstacles>("obstacles",1);

	ros::spin();	
}
