#include <ros/ros.h>
#include <ros/console.h>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include <cmath>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;

typedef Eigen::Matrix<double, 5, 4> state_type;

ros::Subscriber message_sub;

bool previous_obstacles_received = false;
bool obstacle_is_new = false;

float dt = 0.1;

double x_est, y_est, th_est, v_est;
double x_upd, y_upd, th_upd, v_upd;

nautonomous_mpc_msgs::Obstacles obstacles_set;

std::vector<nautonomous_mpc_msgs::Obstacle>* available_obstacles = new std::vector<nautonomous_mpc_msgs::Obstacle>();
std::vector<int>* iterations = new std::vector<int>();
std::vector<nautonomous_mpc_msgs::Obstacle>* tracked_obstacles = new std::vector<nautonomous_mpc_msgs::Obstacle>();

MatrixXd A(4,4);
MatrixXd P(4,4);
MatrixXd X(4,1);
MatrixXd X_meas(2,1);
MatrixXd R(2,2);
MatrixXd Q(4,4);
MatrixXd I(4,4);
MatrixXd K(4,2);
MatrixXd B(2,4);
MatrixXd S(2,2);

void ODE_constant_obstacle_velocity( const state_type &x , state_type &dxdt , double t )
{
	A = MatrixXd::Zero(4,4);
	dxdt = MatrixXd::Zero(5,4);

	dxdt[0,0] = x[0,3] * cos(x[0,2]);
	dxdt[0,1] = x[0,3] * sin(x[0,2]);
	dxdt[0,2] =  0;
	dxdt[0,3] =  0;

	A(0,2) = -x[0,3] * sin(x[0,2]);
	A(0,3) = cos(x[0,2]);
	A(1,2) = x[0,3] * cos(x[0,2]);
	A(1,3) = sin(x[0,2]);

	dxdt.block<4,4>(1,0) = A * x.block<4,4>(1,0) + x.block<4,4>(1,0) * A.transpose() + Q;
}

void write_ODE( const state_type &x , const double t )
{
	cout << t << '\t' << x[0,0] << '\t' << x[0,1] << '\t' << x[0,2] << '\t' << x[0,3] << endl;
}

float dist( nautonomous_mpc_msgs::Obstacle obstacle1, nautonomous_mpc_msgs::Obstacle obstacle2)
{
	return sqrt(pow(obstacle1.state.pose.position.x - obstacle2.state.pose.position.x,2) + pow(obstacle1.state.pose.position.y - obstacle2.state.pose.position.y,2));
}

float angle( nautonomous_mpc_msgs::Obstacle obstacle1, nautonomous_mpc_msgs::Obstacle obstacle2)
{
	return atan2(obstacle2.state.pose.position.y - obstacle1.state.pose.position.y, obstacle2.state.pose.position.x - obstacle1.state.pose.position.x);
}

void obstacle_cb ( const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	obstacles_set = *obstacle_msg;
	if (!previous_obstacles_received)
	{
		previous_obstacles_received = true;
		for (int i = 0; i < obstacles_set.obstacles.size(); i++)
		{
			available_obstacles->push_back(obstacles_set.obstacles.at(i));
			iterations->push_back(1);
			ROS_INFO_STREAM("First obstacles found");
		}
	}
	else
	{
		ROS_INFO_STREAM(obstacles_set.obstacles.size() << " obstacles found");
		ROS_INFO_STREAM(available_obstacles->size() << " obstacles are open");
		ROS_INFO_STREAM(tracked_obstacles->size() << " obstacles are being tracked");
		for (int i = 0; i < obstacles_set.obstacles.size(); i++)
		{
			obstacle_is_new = true;
			for (int j = 0; j < tracked_obstacles->size(); j++)
			{
				if ( dist(obstacles_set.obstacles.at(i), tracked_obstacles->at(j)) < 1)
				{
					ROS_INFO_STREAM("Obstacle is being tracked");
					
					state_type x;
					x[0,0] = X[0];
					x[0,1] = X[1];
					x[0,2] = X[2];
					x[0,3] = X[3];
					x.block<4,4>(1,0) = P;
    					integrate( ODE_constant_obstacle_velocity , x , 0 , dt , dt , write_ODE );
					X = x.row(0).transpose();
					P = x.block<4,4>(1,0);
					X_meas[0] = obstacles_set.obstacles.at(i).x;
					X_meas[1] = obstacles_set.obstacles.at(i).y;
					S = B * P * B.transpose() + R;
					K = P * B.transpose() * S.inverse();
					X = X + K * X_meas;
					P = (I - K * B) * P;
					obstacle_is_new = false;
					break;
				}
			}
			for (int j = 0; j < available_obstacles->size(); j++)
			{
				if ( dist(obstacles_set.obstacles.at(i), available_obstacles->at(j)) < 1)
				{
					obstacle_is_new = false;
					ROS_INFO_STREAM("Close obstacle found");
					if (iterations->at(j) < 10)
					{
						available_obstacles->at(j).state.twist.angular.z = (available_obstacles->at(j).state.twist.angular.z * (iterations->at(j)-1) + dist(obstacles_set.obstacles.at(i), available_obstacles->at(j)) / dt ) / (iterations->at(j));
						available_obstacles->at(j).state.pose.orientation.z = (available_obstacles->at(j).state.pose.orientation.z * (iterations->at(j)-1) + angle(available_obstacles->at(j), obstacles_set.obstacles.at(i))) / (iterations->at(j));
						iterations->at(j)++;
						available_obstacles->at(j).state.pose.position.x = obstacles_set.obstacles.at(i).state.pose.position.x;
						available_obstacles->at(j).state.pose.position.y = obstacles_set.obstacles.at(i).state.pose.position.y;
						ROS_INFO_STREAM("Estimated obstacle position is: (" << available_obstacles->at(j).state.pose.position.x << ", " << available_obstacles->at(j).state.pose.position.y << ") at a velocity of: " << available_obstacles->at(j).state.twist.angular.z << " with an angle of: " << available_obstacles->at(j).state.pose.orientation.z);
					}
					else if (iterations->at(j) == 10)
					{
						ROS_INFO_STREAM("Obstacle is close at 10");
						available_obstacles->at(j).state.twist.angular.z = (available_obstacles->at(j).state.twist.angular.z * (iterations->at(j)-1) + dist(obstacles_set.obstacles.at(i), available_obstacles->at(j)) / dt ) / (iterations->at(j));
						available_obstacles->at(j).state.pose.orientation.z = (available_obstacles->at(j).state.pose.orientation.z * (iterations->at(j)-1) + angle(available_obstacles->at(j), obstacles_set.obstacles.at(i))) / (iterations->at(j));
						available_obstacles->at(j).state.pose.position.x = obstacles_set.obstacles.at(i).state.pose.position.x;
						available_obstacles->at(j).state.pose.position.y = obstacles_set.obstacles.at(i).state.pose.position.y;
						tracked_obstacles->push_back(available_obstacles->at(j));
						ROS_INFO_STREAM("Estimated obstacle position is: (" << available_obstacles->at(j).state.pose.position.x << ", " << available_obstacles->at(j).state.pose.position.y << ") at a velocity of: " << available_obstacles->at(j).state.twist.angular.z << " with an angle of: " << available_obstacles->at(j).state.pose.orientation.z);
						available_obstacles->erase(available_obstacles->begin() + j);
						iterations->erase(iterations->begin() + j);
					}
					break;
				}
			}
			if (obstacle_is_new)
			{
				ROS_INFO_STREAM("New obstacle found");
				available_obstacles->push_back(obstacles_set.obstacles.at(i));
				iterations->push_back(1);
			}
		}	
	}
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"Obstacle_tracking");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	message_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles",1,obstacle_cb);

	ros::spin();	
}
