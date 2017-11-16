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
nautonomous_mpc_msgs::StageVariable reference_state;
nautonomous_mpc_msgs::StageVariable temp_state;

ros::Publisher position_pub;
ros::Publisher action_pub;

float KKT_var;

/* A template for testing of the solver. */
void gps_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& twist_msg )
{
	/* Some temporary variables. */
	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	current_state = *twist_msg;

	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 1.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;
	

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	acadoVariables.yN[ 0 ] = reference_state.x;
	acadoVariables.yN[ 1 ] = reference_state.y;
	acadoVariables.yN[ 2 ] = reference_state.theta;
	acadoVariables.yN[ 3 ] = reference_state.u;
	acadoVariables.yN[ 4 ] = reference_state.v;
	acadoVariables.yN[ 5 ] = reference_state.omega;

	/* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
	acadoVariables.x0[ 0 ] = current_state.x;
	acadoVariables.x0[ 1 ] = current_state.y;
	acadoVariables.x0[ 2 ] = current_state.theta;
	acadoVariables.x0[ 3 ] = current_state.u;
	acadoVariables.x0[ 4 ] = current_state.v;
	acadoVariables.x0[ 5 ] = current_state.omega;
#endif

	if( VERBOSE ) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );
	
	/* Make the first step*/
	acado_feedbackStep( );

	/* Apply the new control immediately to the process, first NU components. */

	if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

	/* Prepare for the next step. */
	acado_preparationStep();

	/* The "real-time iterations" loop. */
	while ((acado_getKKT() > 1e-6) && iter < 20)
	{
        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

		/* Prepare for the next step. */
		acado_preparationStep();
		iter++;
	}
	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if( !VERBOSE )
	printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	acado_printDifferentialVariables();
	acado_printControlVariables();

	double* states;
	double* actions;
	states = acado_getVariablesX( );
	actions = acado_getVariablesU( );

	temp_state.x = *(states+6);
	temp_state.y = *(states+7);
	temp_state.theta = *(states+8);
	temp_state.u = *(states+9);
	temp_state.v = *(states+10);
	temp_state.omega = *(states+11); 

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
		temp_state.T_l = 0;
		temp_state.T_r = 0;
	}

	position_pub.publish(temp_state);
}

void ref_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& twist_msg )
{
	reference_state = *twist_msg;
}


int main (int argc, char** argv)
{
	ros::init (argc, argv,"MPC");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	
	ros::Subscriber gps_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/current_state",10,gps_cb);
	ros::Subscriber ref_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/reference_state",1,ref_cb);
	
	position_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("next_state",10);

	ros::spin();	
}
