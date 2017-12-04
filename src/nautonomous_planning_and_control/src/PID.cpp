#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <Eigenvalues>

using namespace Eigen;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable reference_state;
nautonomous_mpc_msgs::StageVariable temp_state;

ros::Publisher action_pub;

float a1 = 0.0;
float a2 = 0.0;
float theta_error = 0.0;
float uf = 0.0;
float ut = 0.0;

float KKT_var;

double uf_gain;
double ut_gain;
double u_gain;
double max_theta_error;

double max_ut;
double max_uf;
double min_ut;
double min_uf;

MatrixXd Transformation(4,4);

VectorXd temp(4);

/* A template for testing of the solver. */
void state_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg )
{
	current_state = *state_msg;
	
	temp[0] = reference_state.x;
	temp[1] = reference_state.y;
	temp[2] = 0;
	temp[3] = 1;

	Transformation(0,0) = cos(current_state.theta);
	Transformation(0,1) = -sin(current_state.theta);
	Transformation(1,0) = sin(current_state.theta);
	Transformation(1,1) = cos(current_state.theta);
	Transformation(2,2) = 1;
	Transformation(3,3) = 1;
	Transformation(0,3) = current_state.x;
	Transformation(1,3) = current_state.y;

	temp = Transformation.inverse() * temp;

	a1 = temp[0];
	a2 = temp[1];

	theta_error = atan2(a2,a1);

	if (a1 < 0)
	{
		uf = 0;
		ut = copysign(1,a2);
		std::cout << "Facing the wrong way" << std::endl;
	}
	else if (fabs(theta_error) > max_theta_error)
	{
		uf = 0;
		ut = copysign(1,a2);
		std::cout << "Angle error is too large" << std::endl;
	}
	else
	{
		uf = fmax(fmin(uf_gain*a1 + u_gain*(reference_state.u - current_state.u),max_uf),min_uf);
		ut = fmax(fmin(ut_gain*theta_error,max_ut),min_ut);
		std::cout << "Calculating actions" << std::endl;
	}

	std::cout << "Parallel error: " << a1 << " Perpendicular error: " << a2 << " Forward action: " << uf << " Turning action: " << ut << " Theta_error: " << theta_error << std::endl;

	temp_state.T_l = uf;
	temp_state.T_r = ut;

	action_pub.publish(temp_state);
}

void ref_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& ref_msg )
{
	reference_state = *ref_msg;
}


int main (int argc, char** argv)
{
	ros::init (argc, argv,"PID");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	nh_private.getParam("parallel_error_gain", uf_gain);
	nh_private.getParam("perpendicular_error_gain", ut_gain);
	nh_private.getParam("velocity_error_gain", u_gain);
	nh_private.getParam("max_theta_error", max_theta_error);

	nh_private.getParam("max_ut", max_ut);
	nh_private.getParam("max_uf", max_uf);
	nh_private.getParam("min_ut", min_ut);
	nh_private.getParam("min_uf", min_uf);

	ros::Subscriber state_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/current_state",10,state_cb);
	ros::Subscriber ref_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/reference_state",1,ref_cb);
	
	action_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("next_state",10);

	ros::spin();	
}
