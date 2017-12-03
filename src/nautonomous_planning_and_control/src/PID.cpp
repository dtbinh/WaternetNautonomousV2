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
	else if (fabs(theta_error) > 1)
	{
		uf = 0;
		ut = copysign(1,a2);
		std::cout << "Angle error is too large" << std::endl;
	}
	else
	{
		uf = fmax(fmin(0.25*a1,2),0);
		ut = fmax(fmin(1*theta_error,1),-1);
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
	
	ros::Subscriber state_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/current_state",10,state_cb);
	ros::Subscriber ref_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/reference_state",1,ref_cb);
	
	action_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("next_state",10);

	ros::spin();	
}
