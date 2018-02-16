/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */



/*

IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

#include <nautonomous_mpc_msgs/StageVariable.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

int acado_initializeSolver(  );

nautonomous_mpc_msgs::StageVariable current_state;
nav_msgs::Path reference_path;
nautonomous_mpc_msgs::StageVariable temp_state;

ros::Publisher position_pub;
ros::Publisher action_pub;

float KKT_var;
int Path_point = 0;

/* A template for testing of the solver. */
void gps_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& twist_msg )
{

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}
	/* Some temporary variables. */
	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	current_state = *twist_msg;

	if (Path_point ==0 )
	{
		for (i = 0; i < (N + 1); ++i) 
		{
			acadoVariables.x[ (NX * i) + 0 ] = reference_path.poses[Path_point + i].pose.position.x;
			acadoVariables.x[ (NX * i) + 1 ] = reference_path.poses[Path_point + i].pose.position.y;
			acadoVariables.x[ (NX * i) + 2 ] = 0.0;
			acadoVariables.x[ (NX * i) + 3 ] = 0.0;
			acadoVariables.x[ (NX * i) + 4 ] = 0.0;
			acadoVariables.x[ (NX * i) + 5 ] = 0.0;
			acadoVariables.x[ (NX * i) + 6 ] = 0.0;
			acadoVariables.x[ (NX * i) + 7 ] = 0.0;

			ROS_DEBUG_STREAM("Initialization of all " << NX << " elements x at [" << i << "] [" <<  acadoVariables.x[ (NX * i) + 0 ] << ", "<<  acadoVariables.x[ (NX * i) + 1 ] << ", "<<  acadoVariables.x[ (NX * i) + 2 ] << ", "<<  acadoVariables.x[ (NX * i) + 3 ] << ", "<<  acadoVariables.x[ (NX * i) + 4 ] << ", "<<  acadoVariables.x[ (NX * i) + 5 ] << ", "<<  acadoVariables.x[ (NX * i) + 6 ] << ", "<<  acadoVariables.x[ (NX * i) + 7 ] << "]" );
		}
		for (i = 0; i < N; ++i)
		{
			acadoVariables.u[ (NU * i) + 0 ] = 100.0;
			acadoVariables.u[ (NU * i) + 1 ] = 100.0;
			ROS_DEBUG_STREAM("Initialization of all " << NU << " elements u at [" << i << "] [" <<  acadoVariables.u[ (NU * i) + 0 ] << ", "<<  acadoVariables.u[ (NU * i) + 1 ] << "]" );
		}
		for (i = 0; i < N * NOD; ++i) 
		{
			acadoVariables.od[ i ] = 0.5;
		}
	
	}
	else
	{
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

			ROS_DEBUG_STREAM("Initialization of all " << NX << " elements x at [" << i << "] [" <<  acadoVariables.x[ (NX * i) + 0 ] << ", "<<  acadoVariables.x[ (NX * i) + 1 ] << ", "<<  acadoVariables.x[ (NX * i) + 2 ] << ", "<<  acadoVariables.x[ (NX * i) + 3 ] << ", "<<  acadoVariables.x[ (NX * i) + 4 ] << ", "<<  acadoVariables.x[ (NX * i) + 5 ] << ", "<<  acadoVariables.x[ (NX * i) + 6 ] << ", "<<  acadoVariables.x[ (NX * i) + 7 ] << "]" );
		}

		acadoVariables.x[ (NX * N) + 0 ] = reference_path.poses[Path_point + N].pose.position.x;
		acadoVariables.x[ (NX * N) + 1 ] = reference_path.poses[Path_point + N].pose.position.y;
		acadoVariables.x[ (NX * N) + 2 ] = acadoVariables.x[ (NX * N) + 2 ];
		acadoVariables.x[ (NX * N) + 3 ] = acadoVariables.x[ (NX * N) + 3 ];
		acadoVariables.x[ (NX * N) + 4 ] = acadoVariables.x[ (NX * N) + 4 ];
		acadoVariables.x[ (NX * N) + 5 ] = acadoVariables.x[ (NX * N) + 5 ];
		acadoVariables.x[ (NX * N) + 6 ] = acadoVariables.x[ (NX * N) + 6 ];
		acadoVariables.x[ (NX * N) + 7 ] = acadoVariables.x[ (NX * N) + 7 ];
		
		ROS_DEBUG_STREAM("Initialization of all " << NX << " elements x at [" << i << "] [" <<  acadoVariables.x[ (NX * i) + 0 ] << ", "<<  acadoVariables.x[ (NX * i) + 1 ] << ", "<<  acadoVariables.x[ (NX * i) + 2 ] << ", "<<  acadoVariables.x[ (NX * i) + 3 ] << ", "<<  acadoVariables.x[ (NX * i) + 4 ] << ", "<<  acadoVariables.x[ (NX * i) + 5 ] << ", "<<  acadoVariables.x[ (NX * i) + 6 ] << ", "<<  acadoVariables.x[ (NX * i) + 7 ] << "]" );
		
		for (i = 0; i < (N-1); ++i)
		{
			acadoVariables.u[ (NU * i) + 0 ] = acadoVariables.u[ (NU * (i+1)) + 0 ];
			acadoVariables.u[ (NU * i) + 1 ] = acadoVariables.u[ (NU * (i+1)) + 1 ];
			ROS_DEBUG_STREAM("Initialization of all " << NU << " elements u at [" << i << "] [" <<  acadoVariables.u[ (NU * i) + 0 ] << ", "<<  acadoVariables.u[ (NU * i) + 1 ]  << "]" );

		}

		acadoVariables.u[ NU * (N-1) + 0 ] = 100.0;
		acadoVariables.u[ NU * (N-1) + 1 ] = 100.0;
		ROS_DEBUG_STREAM("Initialization of all " << NU << " elements u at [" << i << "] [" <<  acadoVariables.u[ (NU * (N-1)) + 0 ] << ", "<<  acadoVariables.u[ (NU * (N-1)) + 1 ] <<  "]" );

		for (i = 0; i < N * NOD; ++i) 
		{
			acadoVariables.od[ i ] = 0.5;
		}
	}
	/* Initialize the measurements/reference. */
	for (i = 0; i < N; ++i) 
	{
		acadoVariables.y[ (NY * i) + 0 ] = reference_path.poses[Path_point + i + 1].pose.position.x;
		acadoVariables.y[ (NY * i) + 1 ] = reference_path.poses[Path_point + i + 1].pose.position.y;
		acadoVariables.y[ (NY * i) + 2 ] = 0.0;
		acadoVariables.y[ (NY * i) + 3 ] = 0.0;
		acadoVariables.y[ (NY * i) + 4 ] = 0.0;
		acadoVariables.y[ (NY * i) + 5 ] = 0.0;
		acadoVariables.y[ (NY * i) + 6 ] = 0.0;
		acadoVariables.y[ (NY * i) + 7 ] = 0.0;
		acadoVariables.y[ (NY * i) + 8 ] = 0.0;
		acadoVariables.y[ (NY * i) + 9 ] = 0.0;
		ROS_DEBUG_STREAM("Initialization of all " << NY << " elements y at [" << i << "] [" <<  acadoVariables.y[ (NY * i) + 0 ] << ", "<<  acadoVariables.y[ (NY * i) + 1 ] << ", "<<  acadoVariables.y[ (NY * i) + 2 ] << ", "<<  acadoVariables.y[ (NY * i) + 3 ] << ", "<<  acadoVariables.y[ (NY * i) + 4 ] << ", "<<  acadoVariables.y[ (NY * i) + 5 ] << ", "<<  acadoVariables.y[ (NY * i) + 6 ] << ", "<<  acadoVariables.y[ (NY * i) + 7 ] << ", " << acadoVariables.y[ (NY * i) + 8 ] << ", "<<  acadoVariables.y[ (NY * i) + 9 ] << ", "<< "]" );
	}

	acadoVariables.yN[ 0 ] = reference_path.poses[Path_point + N ].pose.position.x;
	acadoVariables.yN[ 1 ] = reference_path.poses[Path_point + N ].pose.position.y;
	acadoVariables.yN[ 2 ] = 0.0;
	acadoVariables.yN[ 3 ] = 0.0;
	acadoVariables.yN[ 4 ] = 0.0;
	acadoVariables.yN[ 5 ] = 0.0;
	acadoVariables.yN[ 6 ] = 0.0;
	acadoVariables.yN[ 7 ] = 0.0;

	ROS_DEBUG_STREAM("Initialization of all " << NYN << " elements yN [" <<  acadoVariables.yN[ 0 ] << ", "<<  acadoVariables.yN[ 1 ] << ", "<<  acadoVariables.yN[ 2 ] << ", "<<  acadoVariables.yN[ 3 ] << ", "<<  acadoVariables.yN[ 4 ] << ", "<<  acadoVariables.yN[ 5 ] << ", "<<  acadoVariables.yN[ 6 ] << ", "<<  acadoVariables.yN[ 7 ] << "]" );


	acadoVariables.x0[ 0 ] = current_state.x;
	acadoVariables.x0[ 1 ] = current_state.y;
	acadoVariables.x0[ 2 ] = current_state.theta;
	acadoVariables.x0[ 3 ] = current_state.u;
	acadoVariables.x0[ 4 ] = current_state.v;
	acadoVariables.x0[ 5 ] = current_state.omega;
	acadoVariables.x0[ 6 ] = -10 + 0.5 * Path_point;
	acadoVariables.x0[ 7 ] = 0;

	ROS_DEBUG_STREAM("Initialization of all " << NX << " elements X0 [" <<  acadoVariables.x0[ 0 ] << ", "<<  acadoVariables.x0[ 1 ] << ", "<<  acadoVariables.x0[ 2 ] << ", "<<  acadoVariables.x0[ 3 ] << ", "<<  acadoVariables.x0[ 4 ] << ", "<<  acadoVariables.x0[ 5 ] << ", "<<  acadoVariables.x0[ 6 ] << ", "<<  acadoVariables.x0[ 7 ] << "]" );

	Path_point++;

	//if( VERBOSE ) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );
	
	/* Make the first step
	acado_feedbackStep( );*/

	/* Apply the new control immediately to the process, first NU components. 

	if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e, Objective = %.3e\n\n", iter, acado_getKKT(), acado_getObjective() );*/

	/* Prepare for the next step. 
	acado_preparationStep();*/

	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{
        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e, Objective = %.3e\n\n", iter, acado_getKKT(), acado_getObjective() );

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


	if (KKT_var < 1e-6)
	{
		printf("\n\n Action variables are Tl:   %.3e and Tr: %.3e with KKT: %.3e\n\n", *(actions), *(actions+1), KKT_var);
		temp_state.T_l = *(actions);
		temp_state.T_r = *(actions+1); 
	}
	else
	{
		printf("\n\n Action variables are Tl:   %.3e and Tr: %.3e with KKT: %.3e\n\n", 0, 0, KKT_var);
		temp_state.T_l = 75;
		temp_state.T_r = -75;
	}

	position_pub.publish(temp_state);
}

void ref_cb( const nav_msgs::Path::ConstPtr& reference_msg )
{
	ROS_INFO_STREAM("Path received");
	reference_path = *reference_msg;
	Path_point = 0;
}


int main (int argc, char** argv)
{
	ros::init (argc, argv,"MPC");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	ros::Subscriber gps_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/current_state",10,gps_cb);
	ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("/A_star_tree_path_finding_opt/route",1,ref_cb);
	
	position_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("next_state",10);

	ros::spin();	
}
