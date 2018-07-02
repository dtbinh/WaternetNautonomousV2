#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nautonomous_planning_and_control/Quaternion_conversion.h>
#include <gazebo_msgs/ModelState.h>

#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   100        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

int acado_initializeSolver(  );

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::StageVariable temp_state;
nautonomous_mpc_msgs::StageVariable gazebo_state;
nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacle ghost_obstacle;
nautonomous_mpc_msgs::Obstacles obstacles;
nautonomous_mpc_msgs::Obstacles obstacles_sorted_by_distance;
nautonomous_mpc_msgs::Obstacle border;
nautonomous_mpc_msgs::Obstacles borders;
nautonomous_mpc_msgs::Obstacles borders_sorted_by_distance;

geometry_msgs::PoseStamped p;
nav_msgs::Path reference_path;
nav_msgs::Path route_list;

geometry_msgs::Twist action;

ros::Publisher position_pub;
ros::Publisher action_pub;
ros::Publisher control_horizon_pub;
ros::Publisher gazebo_pub;

float KKT_var;
int Path_point = 0;
float dt = 0.1;

std::vector<float>* Distances = new std::vector<float>();

void border_cb (const nautonomous_mpc_msgs::Obstacles::ConstPtr& border_msg )
{
	std::cout << border_msg->obstacles.size() << " Borders received" << std::endl;
	borders = *border_msg;
	std::cout << "Borders processed" << std::endl;
}

void obstacle_cb (const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg )
{
	std::cout << obstacle_msg->obstacles.size() << " Obstacles received" << std::endl;
	obstacles = *obstacle_msg;
	while (obstacles.obstacles.size() < 4)
	{
		obstacles.obstacles.push_back(ghost_obstacle);
		std::cout << "Added ghost obstacle" << std::endl;
	}
	obstacles_sorted_by_distance = obstacles;
	std::cout << "Obstacles processed" << std::endl;
}

/* A template for testing of the solver. */
void gps_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& twist_msg )
{
	/* Some temporary variables. */
	int    i = 0;
	int    iter = 0;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	current_state = *twist_msg;

	Distances->clear();
	for (int j = 0; j < borders.obstacles.size(); j++)
	{
		float dist_cp_1 = sqrt(pow(current_state.x-borders.obstacles[j].pose.position.x + borders.obstacles[j].major_semiaxis * cos(toEulerAngle(borders.obstacles[j].pose.orientation)) + borders.obstacles[j].minor_semiaxis * sin(toEulerAngle(borders.obstacles[j].pose.orientation)),2) + pow(current_state.y-borders.obstacles[j].pose.position.y + borders.obstacles[j].major_semiaxis * sin(toEulerAngle(borders.obstacles[j].pose.orientation)) - borders.obstacles[j].minor_semiaxis * cos(toEulerAngle(borders.obstacles[j].pose.orientation)),2));
		float dist_cp_2 = sqrt(pow(current_state.x-borders.obstacles[j].pose.position.x + borders.obstacles[j].major_semiaxis * cos(toEulerAngle(borders.obstacles[j].pose.orientation)) - borders.obstacles[j].minor_semiaxis * sin(toEulerAngle(borders.obstacles[j].pose.orientation)),2) + pow(current_state.y-borders.obstacles[j].pose.position.y + borders.obstacles[j].major_semiaxis * sin(toEulerAngle(borders.obstacles[j].pose.orientation)) + borders.obstacles[j].minor_semiaxis * cos(toEulerAngle(borders.obstacles[j].pose.orientation)),2));
		float dist_cp_3 = sqrt(pow(current_state.x-borders.obstacles[j].pose.position.x - borders.obstacles[j].major_semiaxis * cos(toEulerAngle(borders.obstacles[j].pose.orientation)) + borders.obstacles[j].minor_semiaxis * sin(toEulerAngle(borders.obstacles[j].pose.orientation)),2) + pow(current_state.y-borders.obstacles[j].pose.position.y - borders.obstacles[j].major_semiaxis * sin(toEulerAngle(borders.obstacles[j].pose.orientation)) - borders.obstacles[j].minor_semiaxis * cos(toEulerAngle(borders.obstacles[j].pose.orientation)),2));
		float dist_cp_4 = sqrt(pow(current_state.x-borders.obstacles[j].pose.position.x - borders.obstacles[j].major_semiaxis * cos(toEulerAngle(borders.obstacles[j].pose.orientation)) - borders.obstacles[j].minor_semiaxis * sin(toEulerAngle(borders.obstacles[j].pose.orientation)),2) + pow(current_state.y-borders.obstacles[j].pose.position.y - borders.obstacles[j].major_semiaxis * sin(toEulerAngle(borders.obstacles[j].pose.orientation)) + borders.obstacles[j].minor_semiaxis * cos(toEulerAngle(borders.obstacles[j].pose.orientation)),2));
		Distances->push_back(fmin(fmin(fmin(dist_cp_1,dist_cp_2),dist_cp_3),dist_cp_4));
		ROS_DEBUG_STREAM("Distance to border: " << Distances->at(i));
	}
	borders_sorted_by_distance.obstacles.clear();
	for (int j = 0; j < 5; j++)
	{	
		float minimum = Distances->at(0);
		float it = 0;
		for (int k = 1; k < Distances->size(); k++)
		{
			if (Distances->at(k) < minimum)
			{
				minimum = Distances->at(k);
				it = k;
			}
		}
		borders_sorted_by_distance.obstacles.push_back(borders.obstacles[it]);
		Distances->at(it) = 1000000;
	}
	ROS_DEBUG_STREAM("Closest borders: " << borders_sorted_by_distance);

	Distances->clear();
	if (obstacles.obstacles.size() > 0)
	{
		for (int j = 0; j < obstacles.obstacles.size(); j++)
		{
			float dist = sqrt(pow(current_state.x-obstacles.obstacles[j].pose.position.x,2) + pow(current_state.y-obstacles.obstacles[j].pose.position.y,2));
			Distances->push_back(dist);
			ROS_DEBUG_STREAM("Distance to border: " << Distances->at(i));
		}
		obstacles_sorted_by_distance.obstacles.clear();
		for (int j = 0; j < 5; j++)
		{	
			float minimum = Distances->at(0);
			float it = 0;
			for (int k = 1; k < Distances->size(); k++)
			{
				if (Distances->at(k) < minimum)
				{
					minimum = Distances->at(k);
					it = k;
				}
			}
			obstacles_sorted_by_distance.obstacles.push_back(obstacles.obstacles[it]);
			Distances->at(it) = 1000000;
		}
	}

	ROS_DEBUG_STREAM("Closest obstacles: " << obstacles_sorted_by_distance);
	ROS_DEBUG_STREAM("State received");
	ROS_DEBUG_STREAM("Path point set to " << Path_point);	
	ROS_DEBUG_STREAM("reference path length " << reference_path.poses.size());	

	while ((sqrt(pow(reference_path.poses[Path_point + i].pose.position.x - current_state.x,2) - pow(reference_path.poses[Path_point + i].pose.position.y - current_state.y,2)) < 1) && (Path_point > 0))
	{
		Path_point++;
		ROS_DEBUG_STREAM("Path point set to" << Path_point);	
	}

	if (Path_point == 0 )
	{
		ROS_DEBUG_STREAM("Start of the first iterations");

		for (i = 0; i < (N + 1); ++i) 
		{
			if ((Path_point + i) < (reference_path.poses.size()))
			{
				acadoVariables.x[ (NX * i) + 0 ] = reference_path.poses[Path_point + i].pose.position.x;
				acadoVariables.x[ (NX * i) + 1 ] = reference_path.poses[Path_point + i].pose.position.y;
			}
			else
			{
				acadoVariables.x[ (NX * i) + 0 ] = reference_path.poses[reference_path.poses.size() - 1].pose.position.x;
				acadoVariables.x[ (NX * i) + 1 ] = reference_path.poses[reference_path.poses.size() - 1].pose.position.y;
				
			}			
			acadoVariables.x[ (NX * i) + 2 ] = 0.0;
			acadoVariables.x[ (NX * i) + 3 ] = 0.0;
			acadoVariables.x[ (NX * i) + 4 ] = 0.0;
			acadoVariables.x[ (NX * i) + 5 ] = 0.0;

			ROS_DEBUG_STREAM("Initialization of all " << NX << " elements x at [" << i << "] [" <<  acadoVariables.x[ (NX * i) + 0 ] << ", "<<  acadoVariables.x[ (NX * i) + 1 ] << ", "<<  acadoVariables.x[ (NX * i) + 2 ] << ", "<<  acadoVariables.x[ (NX * i) + 3 ] << ", "<<  acadoVariables.x[ (NX * i) + 4 ] << ", "<<  acadoVariables.x[ (NX * i) + 5 ] << "]" );
		}
		for (i = 0; i < N; ++i)
		{
			acadoVariables.u[ (NU * i) + 0 ] = 100.0;
			acadoVariables.u[ (NU * i) + 1 ] = 100.0;
			ROS_DEBUG_STREAM("Initialization of all " << NU << " elements u at [" << i << "] [" <<  acadoVariables.u[ (NU * i) + 0 ] << ", "<<  acadoVariables.u[ (NU * i) + 1 ] << "]" );
		}
		for (i = 0; i < N ; ++i) 
		{
			if (obstacles_sorted_by_distance.obstacles.size() < 4)
			{
				ROS_FATAL_STREAM("Not enough obstacles registered");
			}
			else
			{
				for (int j = 0; j < 5; j++)
				{	
					acadoVariables.od[ (NOD * i) + (5 * j) + 0 ] = obstacles_sorted_by_distance.obstacles[j].pose.position.x + obstacles_sorted_by_distance.obstacles[j].twist.linear.x * cos(obstacle.pose.orientation.z) * i * dt;
					acadoVariables.od[ (NOD * i) + (5 * j) + 1 ] = obstacles_sorted_by_distance.obstacles[j].pose.position.y + obstacles_sorted_by_distance.obstacles[j].twist.linear.x * sin(obstacle.pose.orientation.z) * i * dt;
					acadoVariables.od[ (NOD * i) + (5 * j) + 2 ] = toEulerAngle(obstacles_sorted_by_distance.obstacles[j].pose.orientation);
					acadoVariables.od[ (NOD * i) + (5 * j) + 3 ] = 1/obstacles_sorted_by_distance.obstacles[j].major_semiaxis;
					acadoVariables.od[ (NOD * i) + (5 * j) + 4 ] = 1/obstacles_sorted_by_distance.obstacles[j].minor_semiaxis;
					ROS_DEBUG_STREAM("Initialization of all " << NOD << " elements nod at [" << i << ", " << j << "] [" <<  acadoVariables.od[ (NOD * i) +  (5 * j) + 0 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 1 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 2 ] << ", "<<  acadoVariables.od[ (NOD * i) +  (5 * j) + 3 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 4 ] << "]" );
				}
			}

			if (obstacles_sorted_by_distance.obstacles.size() < 4)
			{
				ROS_FATAL_STREAM("Not enough borders registered");
			}
			else
			{
				for (int j = 0; j < 5; j++)
				{
					acadoVariables.od[ (NOD * i) + (5 * j) + 25 ] = borders_sorted_by_distance.obstacles[j].pose.position.x;
					acadoVariables.od[ (NOD * i) + (5 * j) + 26 ] = borders_sorted_by_distance.obstacles[j].pose.position.y;
					acadoVariables.od[ (NOD * i) + (5 * j) + 27 ] = toEulerAngle(borders_sorted_by_distance.obstacles[j].pose.orientation);
					acadoVariables.od[ (NOD * i) + (5 * j) + 28 ] = 1/borders_sorted_by_distance.obstacles[j].major_semiaxis;
					acadoVariables.od[ (NOD * i) + (5 * j) + 29 ] = 1/borders_sorted_by_distance.obstacles[j].minor_semiaxis;
					ROS_DEBUG_STREAM("Initialization of all " << NOD << " elements nod at [" << i << ", " << j << "] [" <<  acadoVariables.od[ (NOD * i) + (5 * j) + 25 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 26 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 27 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 28 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 29 ] << "]" );
				}
			}
		}
		Path_point++;
	}
	else
	{
		ROS_DEBUG_STREAM("Start of the iteration");
		for (i = 0; i < N; ++i) 
		{
			acadoVariables.x[ (NX * i) + 0 ] = acadoVariables.x[ (NX * (i+1)) + 0 ];
			acadoVariables.x[ (NX * i) + 1 ] = acadoVariables.x[ (NX * (i+1)) + 1 ];
			acadoVariables.x[ (NX * i) + 2 ] = acadoVariables.x[ (NX * (i+1)) + 2 ];
			acadoVariables.x[ (NX * i) + 3 ] = acadoVariables.x[ (NX * (i+1)) + 3 ];
			acadoVariables.x[ (NX * i) + 4 ] = acadoVariables.x[ (NX * (i+1)) + 4 ];
			acadoVariables.x[ (NX * i) + 5 ] = acadoVariables.x[ (NX * (i+1)) + 5 ];
			acadoVariables.x[ (NX * i) + 6 ] = acadoVariables.x[ (NX * (i+1)) + 6 ];
			acadoVariables.x[ (NX * i) + 7 ] = acadoVariables.x[ (NX * (i+1)) + 7 ];

			ROS_DEBUG_STREAM("Initialization of all " << NX << " elements x at [" << i << "] [" <<  acadoVariables.x[ (NX * i) + 0 ] << ", "<<  acadoVariables.x[ (NX * i) + 1 ] << ", "<<  acadoVariables.x[ (NX * i) + 2 ] << ", "<<  acadoVariables.x[ (NX * i) + 3 ] << ", "<<  acadoVariables.x[ (NX * i) + 4 ] << ", "<<  acadoVariables.x[ (NX * i) + 5 ] << "]" );
		}

		if ((Path_point + i) < (reference_path.poses.size()))
		{
			acadoVariables.x[ (NX * N) + 0 ] = reference_path.poses[Path_point + i].pose.position.x;
			acadoVariables.x[ (NX * N) + 1 ] = reference_path.poses[Path_point + i].pose.position.y;
		}
		else
		{
			acadoVariables.x[ (NX * N) + 0 ] = reference_path.poses[reference_path.poses.size() - 1 ].pose.position.x;
			acadoVariables.x[ (NX * N) + 1 ] = reference_path.poses[reference_path.poses.size() - 1].pose.position.y;
		}


		acadoVariables.x[ (NX * N) + 2 ] = acadoVariables.x[ (NX * N) + 2 ];
		acadoVariables.x[ (NX * N) + 3 ] = acadoVariables.x[ (NX * N) + 3 ];
		acadoVariables.x[ (NX * N) + 4 ] = acadoVariables.x[ (NX * N) + 4 ];
		acadoVariables.x[ (NX * N) + 5 ] = acadoVariables.x[ (NX * N) + 5 ];
		
		ROS_DEBUG_STREAM("Initialization of all " << NX << " elements x at [" << i << "] [" <<  acadoVariables.x[ (NX * i) + 0 ] << ", "<<  acadoVariables.x[ (NX * i) + 1 ] << ", "<<  acadoVariables.x[ (NX * i) + 2 ] << ", "<<  acadoVariables.x[ (NX * i) + 3 ] << ", "<<  acadoVariables.x[ (NX * i) + 4 ] << ", "<<  acadoVariables.x[ (NX * i) + 5 ] << "]" );
		
		for (i = 0; i < (N-1); ++i)
		{
			acadoVariables.u[ (NU * i) + 0 ] = acadoVariables.u[ (NU * (i+1)) + 0 ];
			acadoVariables.u[ (NU * i) + 1 ] = acadoVariables.u[ (NU * (i+1)) + 1 ];
			ROS_DEBUG_STREAM("Initialization of all " << NU << " elements u at [" << i << "] [" <<  acadoVariables.u[ (NU * i) + 0 ] << ", "<<  acadoVariables.u[ (NU * i) + 1 ]  << "]" );

		}

		acadoVariables.u[ NU * (N-1) + 0 ] = 100.0;
		acadoVariables.u[ NU * (N-1) + 1 ] = 100.0;
		ROS_DEBUG_STREAM("Initialization of all " << NU << " elements u at [" << i << "] [" <<  acadoVariables.u[ (NU * (N-1)) + 0 ] << ", "<<  acadoVariables.u[ (NU * (N-1)) + 1 ] <<  "]" );

		for (i = 0; i < N ; ++i) 
		{
			for (int j = 0; j < 5; j++)
			{
				acadoVariables.od[ (NOD * i) + (5 * j) + 0 ] = obstacles_sorted_by_distance.obstacles[j].pose.position.x + obstacles_sorted_by_distance.obstacles[j].twist.linear.x * cos(obstacle.pose.orientation.z) * i * dt;
				acadoVariables.od[ (NOD * i) + (5 * j) + 1 ] = obstacles_sorted_by_distance.obstacles[j].pose.position.y + obstacles_sorted_by_distance.obstacles[j].twist.linear.x * sin(obstacle.pose.orientation.z) * i * dt;
				acadoVariables.od[ (NOD * i) + (5 * j) + 2 ] = toEulerAngle(obstacles_sorted_by_distance.obstacles[j].pose.orientation);
				acadoVariables.od[ (NOD * i) + (5 * j) + 3 ] = 1/obstacles_sorted_by_distance.obstacles[j].major_semiaxis;
				acadoVariables.od[ (NOD * i) + (5 * j) + 4 ] = 1/obstacles_sorted_by_distance.obstacles[j].minor_semiaxis;
				ROS_DEBUG_STREAM("Initialization of all " << NOD << " elements nod at [" << i << ", " << j << "] [" <<  acadoVariables.od[ (NOD * i) +  (5 * j) + 0 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 1 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 2 ] << ", "<<  acadoVariables.od[ (NOD * i) +  (5 * j) + 3 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 4 ] << "]" );
			}
			for (int j = 0; j < 5; j++)
			{
				acadoVariables.od[ (NOD * i) + (5 * j) + 25 ] = borders_sorted_by_distance.obstacles[j].pose.position.x;
				acadoVariables.od[ (NOD * i) + (5 * j) + 26 ] = borders_sorted_by_distance.obstacles[j].pose.position.y;
				acadoVariables.od[ (NOD * i) + (5 * j) + 27 ] = toEulerAngle(borders_sorted_by_distance.obstacles[j].pose.orientation);
				acadoVariables.od[ (NOD * i) + (5 * j) + 28 ] = 1/borders_sorted_by_distance.obstacles[j].major_semiaxis;
				acadoVariables.od[ (NOD * i) + (5 * j) + 29 ] = 1/borders_sorted_by_distance.obstacles[j].minor_semiaxis;
				ROS_DEBUG_STREAM("Initialization of all " << NOD << " elements nod at [" << i << ", " << j << "] [" <<  acadoVariables.od[ (NOD * i) + (5 * j) + 25 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 26 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 27 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 28 ] << ", "<<  acadoVariables.od[ (NOD * i) + (5 * j) + 29 ] << "]" );
			}
		}
	}

	/* Initialize the measurements/reference. */
	for (i = 0; i < N; ++i) 
	{
		if ((Path_point + i) < (reference_path.poses.size() - 1))
		{
			acadoVariables.y[ (NY * i) + 0 ] = reference_path.poses[Path_point + i + 1].pose.position.x;
			acadoVariables.y[ (NY * i) + 1 ] = reference_path.poses[Path_point + i + 1].pose.position.y;
		}
		else
		{
			acadoVariables.y[ (NY * i) + 0 ] = reference_path.poses[reference_path.poses.size() - 1 ].pose.position.x;
			acadoVariables.y[ (NY * i) + 1 ] = reference_path.poses[reference_path.poses.size() - 1 ].pose.position.y;
		}

		acadoVariables.y[ (NY * i) + 2 ] = 0.0;
		acadoVariables.y[ (NY * i) + 3 ] = 1.0;
		acadoVariables.y[ (NY * i) + 4 ] = 0.0;
		acadoVariables.y[ (NY * i) + 5 ] = 0.0;
		acadoVariables.y[ (NY * i) + 6 ] = 0.0;
		acadoVariables.y[ (NY * i) + 7 ] = 0.0;
		ROS_DEBUG_STREAM("Initialization of all " << NY << " elements y at [" << i << "] [" <<  acadoVariables.y[ (NY * i) + 0 ] << ", "<<  acadoVariables.y[ (NY * i) + 1 ] << ", "<<  acadoVariables.y[ (NY * i) + 2 ] << ", "<<  acadoVariables.y[ (NY * i) + 3 ] << ", "<<  acadoVariables.y[ (NY * i) + 4 ] << ", "<<  acadoVariables.y[ (NY * i) + 5 ] << ", "<<  acadoVariables.y[ (NY * i) + 6 ] << "]" );
	}

	if ((Path_point + i) < (reference_path.poses.size() - 1))
	{
		acadoVariables.yN[ 0 ] = reference_path.poses[Path_point + N ].pose.position.x;
		acadoVariables.yN[ 1 ] = reference_path.poses[Path_point + N ].pose.position.y;
	}
	else
	{
		acadoVariables.yN[ 0 ] = reference_path.poses[reference_path.poses.size() - 1 ].pose.position.x;
		acadoVariables.yN[ 1 ] = reference_path.poses[reference_path.poses.size() - 1 ].pose.position.y;
	}

	acadoVariables.yN[ 2 ] = 0.0;
	acadoVariables.yN[ 3 ] = 0.0;
	acadoVariables.yN[ 4 ] = 0.0;
	acadoVariables.yN[ 5 ] = 0.0;

	ROS_DEBUG_STREAM("Initialization of all " << NYN << " elements yN [" <<  acadoVariables.yN[ 0 ] << ", "<<  acadoVariables.yN[ 1 ] << ", "<<  acadoVariables.yN[ 2 ] << ", "<<  acadoVariables.yN[ 3 ] << ", "<<  acadoVariables.yN[ 4 ] << ", "<<  acadoVariables.yN[ 5 ] << ", "<< "]" );


	acadoVariables.x0[ 0 ] = current_state.x;
	acadoVariables.x0[ 1 ] = current_state.y;
	acadoVariables.x0[ 2 ] = current_state.theta;
	acadoVariables.x0[ 3 ] = current_state.u;
	acadoVariables.x0[ 4 ] = current_state.v;
	acadoVariables.x0[ 5 ] = current_state.omega;

	ROS_DEBUG_STREAM("Initialization of all " << NX << " elements X0 [" <<  acadoVariables.x0[ 0 ] << ", "<<  acadoVariables.x0[ 1 ] << ", "<<  acadoVariables.x0[ 2 ] << ", "<<  acadoVariables.x0[ 3 ] << ", "<<  acadoVariables.x0[ 4 ] << ", "<<  acadoVariables.x0[ 5 ] << "]" );

//	Path_point++;

	//if( VERBOSE ) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );
	
	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{
        	/* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		//if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e, Objective = %.3e\n\n", iter, acado_getKKT(), acado_getObjective() );

		/* Prepare for the next step. */
		acado_preparationStep();
	}

	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	if( VERBOSE ) printf("\n\nEnd of the RTI loop at t: %.3e. \n\n\n", te);

	/* Eye-candy. */

	if( !VERBOSE )
	printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	acado_printDifferentialVariables();
	acado_printControlVariables();

	double* states;
	double* actions;
	states = acado_getVariablesX( );
	actions = acado_getVariablesU( );

	temp_state.x = *(states+NX);
	temp_state.y = *(states+NX+1);
	temp_state.theta = *(states+NX+2);
	temp_state.u = *(states+NX+3);
	temp_state.v = *(states+NX+4);
	temp_state.omega = *(states+NX+5); 

	KKT_var = acado_getKKT();

	for (int i = 1; i <= N; i++)
	{
		p.pose.position.x = *(states+(NX*i));
		p.pose.position.y = *(states+(NX*i)+1);
		p.pose.orientation = toQuaternion(0.0, 0.0, *(states+(NX*i) + 2));
		route_list.poses.push_back(p);
	}

	printf("\n\n Action variables are Tl:   %.3e and Tr: %.3e with KKT: %.3e in %.3g [ms] \n\n ", *(actions), *(actions+1), KKT_var, 1e3 * te / NUM_STEPS);
	temp_state.T_l = *(actions);
	temp_state.T_r = *(actions+1); 

	position_pub.publish(temp_state);
	control_horizon_pub.publish(route_list);
	route_list.poses.clear();

	gazebo_state.T_r = *(actions);
	gazebo_state.T_l = *(actions+1);  
	gazebo_state.x = *(states+NX);
	gazebo_state.y = *(states+NX+1);
	gazebo_state.theta = *(states+NX+2);
	gazebo_state.u = *(states+NX+3);
	gazebo_state.v = *(states+NX+4);
	gazebo_state.omega = *(states+NX+5);

	gazebo_pub.publish(gazebo_state);

	action.linear.x = *(actions) + *(actions+1);
	action.angular.z = *(actions) - *(actions+1);

	action_pub.publish(action);
}

void ref_cb( const nav_msgs::Path::ConstPtr& reference_msg )
{
	reference_path = *reference_msg;
	ROS_INFO_STREAM("Path of length " << reference_path.poses.size() << " received");
	Path_point = 0;
}

void Initialize()
{
	ghost_obstacle.pose.position.x = 1000;
	ghost_obstacle.pose.position.y = 1000;
	ghost_obstacle.pose.position.z = 0;
	ghost_obstacle.pose.orientation = toQuaternion(0,0,0);
	ghost_obstacle.major_semiaxis = 0.1;
	ghost_obstacle.minor_semiaxis = 0.1;

	borders.obstacles.push_back(ghost_obstacle);
}

int main (int argc, char** argv)
{
	ros::init (argc, argv,"MPC");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	Initialize();

	/*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}*/

	ros::Subscriber gps_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/current_state",10,gps_cb);
	ros::Subscriber obstacle_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/mission_coordinator/obstacles",1,obstacle_cb);
	ros::Subscriber border_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Map_modifier/borders",1,border_cb);
	ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("/Local_planner/route",1,ref_cb);
	
	position_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("next_state",10);
	control_horizon_pub = nh_private.advertise<nav_msgs::Path>("control_horizon",10);
	gazebo_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("set_gazebo_control",10);
	action_pub = nh_private.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	route_list.header.frame_id = "/occupancy_grid";
	route_list.header.stamp = ros::Time::now();

	p.pose.orientation.z = 1;

	ros::spin();	
}
