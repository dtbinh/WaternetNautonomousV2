#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/Waypoint.h>
#include <nautonomous_mpc_msgs/WaypointList.h>
#include <Eigenvalues>

using namespace Eigen;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable temp_state;

nautonomous_mpc_msgs::Waypoint reference_state;
nautonomous_mpc_msgs::WaypointList reference_states;

ros::Publisher action_pub;

float a1 = 0.0;
float a2 = 0.0;
float theta_error = 0.0;
float uf = 0.0;
float ut = 0.0;
double integral = 0.0;

float KKT_var;

double uf_gain;
double ut_gain;
double u_gain;
double I_gain;
double max_theta_error;
double sampling_time;

double max_ut;
double max_uf;
double min_ut;
double min_uf;

bool use_fuzzy;
int number_of_fuzzy_waypoints;
double fuzzy_min_distance;
double fuzzy_max_distance;
double fuzzy_partition;
double fuzzy_total_partition;
double fuzzy_cut_distance;

double temp_ut;
double temp_uf;

MatrixXd Transformation(4,4);

VectorXd temp(4);

/* A template for testing of the solver. */
void state_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& state_msg )
{
	std::cout << "Start PID" << std::endl;

	current_state = *state_msg;
	
	Transformation(0,0) = cos(current_state.theta);
	Transformation(0,1) = -sin(current_state.theta);
	Transformation(1,0) = sin(current_state.theta);
	Transformation(1,1) = cos(current_state.theta);
	Transformation(2,2) = 1;
	Transformation(3,3) = 1;
	Transformation(0,3) = current_state.x;
	Transformation(1,3) = current_state.y;


	if (not(use_fuzzy))
	{
		std::cout << "Don't use fuzzy" << std::endl;
		temp[0] = reference_state.stage.x;
		temp[1] = reference_state.stage.y;
		temp[2] = 0;
		temp[3] = 1;

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
			integral += (reference_state.stage.u - current_state.u) * sampling_time;
			uf = fmax(fmin(uf_gain*a1 + u_gain*(reference_state.stage.u - current_state.u) + I_gain * integral,max_uf),min_uf);
			ut = fmax(fmin(ut_gain*theta_error,max_ut),min_ut);
			std::cout << "Calculating actions" << std::endl;
		}

		std::cout << "Parallel error: " << a1 << " Perpendicular error: " << a2 << " Forward action: " << uf << " Turning action: " << ut << " Theta_error: " << theta_error << std::endl;
	}

	else
	{
		if (!reference_states.stages.empty())
		{
			std::cout << "Use fuzzy" << std::endl;
			uf = 0;
			ut = 0;
			fuzzy_total_partition = 0;

			for (int i = 0; i < fmin(number_of_fuzzy_waypoints,reference_states.stages.size()); i++)
			{
				temp[0] = reference_states.stages[i].x;
				temp[1] = reference_states.stages[i].y;
				temp[2] = 0;
				temp[3] = 1;

				temp = Transformation.inverse() * temp;
				a1 = temp[0];
				a2 = temp[1];

				theta_error = atan2(a2,a1);

				if (i == 0)
				{
					fuzzy_partition = fmax(fmin(a1 / fuzzy_min_distance,1),0);
				}
				else
				{
					fuzzy_partition = fmax(fmin(fmin(a1 / fuzzy_min_distance, 1-(a1 - fuzzy_cut_distance)/(fuzzy_max_distance - fuzzy_cut_distance)),1),0);
				}

				fuzzy_total_partition += fuzzy_partition;

				if (a1 < 0)
				{
					temp_uf = 0;
					temp_ut = copysign(1,a2);
					uf += fuzzy_partition * temp_uf;
					ut += fuzzy_partition * temp_ut;

					std::cout << "Facing the wrong way, ut = " << temp_ut  << std::endl;
				}
				else if (fabs(theta_error) > max_theta_error)
				{
					temp_uf = 0;
					temp_ut = copysign(1,a2);
					uf += fuzzy_partition * temp_uf;
					ut += fuzzy_partition * temp_ut;

					std::cout << "Angle error is too large, ut = " << temp_ut << std::endl;
				}
				else
				{	
					temp_uf = uf_gain*a1 + u_gain*(reference_states.stages[i].u - current_state.u);
					temp_ut = ut_gain*theta_error;
					uf += fuzzy_partition * temp_uf;
					ut += fuzzy_partition * temp_ut;

					std::cout << "Calculating actions" << std::endl;
					std::cout << "Parallel error: " << a1 << " Perpendicular error: " << a2 << " Forward action: " << temp_uf << " Turning action: " << temp_ut << " Theta_error: " << theta_error << " Fuzzy partition: " << fuzzy_partition <<  std::endl;
				}			
			}
		}
		else
		{
			std::cout << "Reference states is empty" <<std::endl;
			uf = 0;
			ut = 0;
			fuzzy_total_partition = 1;
		}
		uf /= fuzzy_total_partition;
		ut /= fuzzy_total_partition;
	}
	std::cout << "Publish action" << std::endl;
	temp_state.T_l = fmax(fmin(uf,max_uf),min_uf);
	temp_state.T_r = fmax(fmin(ut,max_ut),min_ut);

	action_pub.publish(temp_state);

}

void ref_cb( const nautonomous_mpc_msgs::Waypoint::ConstPtr& ref_msg )
{
	reference_state = *ref_msg;
}

void refs_cb( const nautonomous_mpc_msgs::WaypointList::ConstPtr& refs_msg )
{
	reference_states = *refs_msg;
	std::cout << "Received references" << std::endl;
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"PID");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	nh_private.getParam("parallel_error_gain", uf_gain);
	nh_private.getParam("perpendicular_error_gain", ut_gain);
	nh_private.getParam("velocity_error_gain", u_gain);
	nh_private.getParam("integral_gain", I_gain);
	nh_private.getParam("max_theta_error", max_theta_error);

	nh_private.getParam("max_ut", max_ut);
	nh_private.getParam("max_uf", max_uf);
	nh_private.getParam("min_ut", min_ut);
	nh_private.getParam("min_uf", min_uf);

	nh_private.getParam("sampling_time", sampling_time);

	nh_private.getParam("fuzzy/use_fuzzy", use_fuzzy);
	nh_private.getParam("fuzzy/number_of_fuzzy_waypoints", number_of_fuzzy_waypoints);
	nh_private.getParam("fuzzy/fuzzy_min_distance", fuzzy_min_distance);
	nh_private.getParam("fuzzy/fuzzy_max_distance", fuzzy_max_distance);
	nh_private.getParam("fuzzy/fuzzy_cut_distance", fuzzy_cut_distance);

	ros::Subscriber state_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/current_state",10,state_cb);
	ros::Subscriber ref_sub = nh.subscribe<nautonomous_mpc_msgs::Waypoint>("/mission_coordinator/reference_state",1,ref_cb);
	ros::Subscriber refs_sub = nh.subscribe<nautonomous_mpc_msgs::WaypointList>("/mission_coordinator/reference_states",1,refs_cb);
	
	action_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("next_state",10);

	ros::spin();	
}
