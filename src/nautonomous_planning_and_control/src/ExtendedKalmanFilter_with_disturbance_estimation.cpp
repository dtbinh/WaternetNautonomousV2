#include <iostream>
#include <boost/array.hpp>

#include <boost/numeric/odeint.hpp>

#include <ros/ros.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <cmath>

#include <Eigen/Dense>

#include "sensor_msgs/Imu.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <nautonomous_planning_and_control/Quaternion_conversion.h>

#define PI 3.141592653589793238462643383279502884197169399375105820974

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;

typedef boost::array<double, 56> state_type;

ros::Subscriber state_sub;
ros::Subscriber start_sub;

ros::Publisher state_pub;
ros::Publisher odom_pub;
ros::Publisher forward_disturbance_pub;
ros::Publisher yaw_disturbance_pub;


double D_x = 38;
double D_y = 5280;
double D_t = 104;
double m = 200;
double l = 0.8;
double I_z = 100;

double f1 = 0;
double f2 = 0;

double dist_x1;
double dist_x2;
double dist_y1;
double dist_y2;

nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable next_state;

MatrixXd A = MatrixXd::Zero(7,7);
MatrixXd C = MatrixXd::Identity(3,7);
MatrixXd Q = MatrixXd::Identity(7,7);
MatrixXd R = MatrixXd::Identity(3,3);
MatrixXd P = MatrixXd::Identity(7,7);
MatrixXd DP = MatrixXd::Identity(7,7);
MatrixXd I = MatrixXd::Identity(7,7);
MatrixXd S = MatrixXd::Zero(3,3);
MatrixXd K = MatrixXd::Zero(7,3);

VectorXd x_upd(7);
VectorXd x_est(7);
VectorXd x_(7);
VectorXd y_error(3);
VectorXd y_meas(3);

state_type x;

nav_msgs::Odometry Odom;
sensor_msgs::Imu Imu;

int angle_it = 0;
float prev_angle = 0;

bool first_time_initialization = true;

std_msgs::Float64 Data;

void Simulate_motion( const state_type &x , state_type &dxdt , double t )
{
	for (int i = 0; i < 7; i++)
	{
		x_(i) = x[i];
	}
	for (int i = 7; i < 56; i++)
	{
		P(floor((i-7)/7),fmod(i,7)) = x[i];
	}

	A(0,2) = -x_(3) * sin(x_(2));
	A(1,2) =  x_(3) * cos(x_(2));
	A(0,3) = cos(x_(2));
	A(1,3) = sin(x_(2));	
    	A(2,4) = -1;
	A(3,3) = -D_x / m;
    	A(3,5) = 1 / m;
	A(4,4) = -D_t / I_z;  
	A(4,6) = l / I_z;

	dxdt[0] = x_(3) * cos(x_(2));
	dxdt[1] = x_(3) * sin(x_(2));
	dxdt[2] = -x_(4);
	dxdt[3] = (f1 + f2 + x_(5) - D_x * x_(3)) / m;
	dxdt[4] = (l * (f1 - f2 + x_(6) ) - D_t * x_(4)) / I_z; 	
	dxdt[5] = 0;
	dxdt[6] = 0;

	DP = A * P + P * A.transpose() + Q;

	for (int i = 7; i < 56; i++)
	{
		dxdt[i] = DP(floor((i-7)/7),fmod(i,7));
	}

}

void write_states( const state_type &x , const double t )
{
	cout << "State at (" << t << ") :" << ", " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << ", " << x[4] << ", " << x[5] << ", " << x[6] << endl << endl << endl;
}

void start_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& start_msg )
{
	f1 = start_msg->T_l;
	f2 = start_msg->T_r;
	if (first_time_initialization)
	{
		x[0] = start_msg->x;
		x[1] = start_msg->y; 
		x[2] = start_msg->theta;
		x[3] = start_msg->u;
		x[4] = start_msg->omega;
		x[5] = 0;
		x[6] = 0;
		first_time_initialization = false;
	
		prev_angle = start_msg->theta;

		Q = 0.0001 * MatrixXd::Identity(7,7);
		Q(5,5) = 10;
		Q(6,6) = 0.1;

		R = 0.001 * MatrixXd::Identity(3,3);

		P = Q;

		for (int i = 7; i < 56; i++)
		{
			x[i] = P(floor((i-7)/7),fmod(i,7));
		}

		cout << "State initialized at: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << ", " << x[4] << ", " << x[5] << ", " << x[6] << endl << endl << endl;
	}
}

void state_cb( const nav_msgs::Odometry::ConstPtr& state_msg )
{
	// Define initial values:
	// ----------------------
	double t_start = 0.0;
	double t_end = 1;
 
    	integrate( Simulate_motion , x , t_start , t_end , 0.001 , write_states );

	for (int i = 7; i < 56; i++)
	{
		P(floor((i-7)/7),fmod(i,7)) = x[i];
	}

	cout << "P_before_correction: " << endl;	
	cout << P << endl << endl;

	x_est(0) = x[0];
	x_est(1) = x[1];
	x_est(2) = x[2];
	x_est(3) = x[3];
	x_est(4) = x[4];
	x_est(5) = x[5];
	x_est(6) = x[6];

	y_meas(0) = state_msg->pose.pose.position.x;
	y_meas(1) = state_msg->pose.pose.position.y;
	y_meas(2) = toEulerAngle(state_msg->pose.pose.orientation) + 2 * PI * angle_it;

	if ((y_meas(2)-prev_angle) < -3)
	{
		angle_it++;
		y_meas(2) = toEulerAngle(state_msg->pose.pose.orientation) + 2 * PI * angle_it;
	}
	else if ((y_meas(2)-prev_angle) > 3)
	{
		angle_it--;
		y_meas(2) = toEulerAngle(state_msg->pose.pose.orientation) + 2 * PI * angle_it;
	}
	prev_angle = y_meas(2);

	y_error = y_meas - x_est.head(3);

	cout << "State error: " << y_error(0) << ", " << y_error(1) << ", " << y_error(2) << endl << endl << endl;

	S = C * P * C.transpose() + R;

	K = P * C.transpose() * S.inverse();

	x_upd = x_est + K * y_error;

	P = (I - K * C) * P;

	x[0] = x_upd(0);
	x[1] = x_upd(1);
	x[2] = x_upd(2);
	x[3] = x_upd(3);
	x[4] = x_upd(4);
	x[5] = x_upd(5);
	x[6] = x_upd(6);

	next_state.x = x_upd(0);
	next_state.y = x_upd(1);
	next_state.theta = x_upd(2);
	next_state.u = x_upd(3);
	next_state.v = 0;
	next_state.omega = x_upd(4);
	next_state.T_l = f1;
	next_state.T_r = f2;

	state_pub.publish(next_state);

	Odom.pose.pose.position.x = next_state.x;
	Odom.pose.pose.position.y = next_state.y;
	Odom.pose.pose.orientation = toQuaternion(0, 0, next_state.theta);
	Odom.twist.twist.linear.x = next_state.u;
	Odom.twist.twist.angular.z = next_state.omega;

	Odom.header.stamp = ros::Time::now();
	Odom.header.frame_id = "/map";
	odom_pub.publish(Odom);

	Data.data = x_upd(5);
	forward_disturbance_pub.publish(Data);

	Data.data = x_upd(6);
	yaw_disturbance_pub.publish(Data);
}

int main(int argc, char **argv)
{
	ros::init (argc, argv,"Ekf");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	start_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start_ekf",10,start_cb);
	state_sub = nh.subscribe<nav_msgs::Odometry>("/mission_coordinator/current_state",10,state_cb);

	state_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("next_state",10);
	odom_pub = nh_private.advertise<nav_msgs::Odometry>("Odom" ,10);

	forward_disturbance_pub = nh_private.advertise<std_msgs::Float64>("Forward_dist" ,10);
	yaw_disturbance_pub = nh_private.advertise<std_msgs::Float64>("Yaw_dist" ,10);

	ros::spin();

	return 0;
}
