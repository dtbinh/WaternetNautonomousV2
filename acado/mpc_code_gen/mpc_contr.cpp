#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <cmath>
#include <iostream>

USING_NAMESPACE_ACADO

int main( )
{
    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState x;  //X position
	DifferentialState y;  //Y position
	DifferentialState th; //angle
	DifferentialState vx; //X velocity
	DifferentialState vy; //Y velocity
	DifferentialState w;  //angular velocity

	IntermediateState Xcan_1;
	IntermediateState Ycan_1;
	IntermediateState Xcan_2;
	IntermediateState Ycan_2;
	IntermediateState Xcan_3;
	IntermediateState Ycan_3;
	IntermediateState Xcan_4;
	IntermediateState Ycan_4;
	IntermediateState Xcan_5;
	IntermediateState Ycan_5;

	IntermediateState Xcan_6;
	IntermediateState Ycan_6;
	IntermediateState Xcan_7;
	IntermediateState Ycan_7;
	IntermediateState Xcan_8;
	IntermediateState Ycan_8;
	IntermediateState Xcan_9;
	IntermediateState Ycan_9;
	IntermediateState Xcan_10;
	IntermediateState Ycan_10;

	Control fl;
	Control fr;

	OnlineData Xobst_1; 
	OnlineData Yobst_1;
	OnlineData THobst_1;
	OnlineData Aobst_1;
	OnlineData Bobst_1;
	OnlineData Xobst_2; 
	OnlineData Yobst_2;
	OnlineData THobst_2;
	OnlineData Aobst_2;
	OnlineData Bobst_2;
	OnlineData Xobst_3; 
	OnlineData Yobst_3;
	OnlineData THobst_3;
	OnlineData Aobst_3;
	OnlineData Bobst_3;
	OnlineData Xobst_4; 
	OnlineData Yobst_4;
	OnlineData THobst_4;
	OnlineData Aobst_4;
	OnlineData Bobst_4;
	OnlineData Xobst_5; 
	OnlineData Yobst_5;
	OnlineData THobst_5;
	OnlineData Aobst_5;
	OnlineData Bobst_5; 

	OnlineData Xbord_1; 
	OnlineData Ybord_1;
	OnlineData THbord_1;
	OnlineData Abord_1;
	OnlineData Bbord_1;
	OnlineData Xbord_2; 
	OnlineData Ybord_2;
	OnlineData THbord_2;
	OnlineData Abord_2;
	OnlineData Bbord_2;
	OnlineData Xbord_3; 
	OnlineData Ybord_3;
	OnlineData THbord_3;
	OnlineData Abord_3;
	OnlineData Bbord_3;
	OnlineData Xbord_4; 
	OnlineData Ybord_4;
	OnlineData THbord_4;
	OnlineData Abord_4;
	OnlineData Bbord_4;
	OnlineData Xbord_5; 
	OnlineData Ybord_5;
	OnlineData THbord_5;
	OnlineData Abord_5;
	OnlineData Bbord_5;

	const double D_x = 38;
	const double D_y = 5280;
	const double D_t = 1000;
	const double m = 200;
	const double l = 0.8;
	const double I_z = 500; // 14

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;

	Xcan_1 = (Xobst_1 - x) * cos(THobst_1) + (Yobst_1 - y) * sin(THobst_1);
	Ycan_1 = -(Xobst_1 - x) * sin(THobst_1) + (Yobst_1 - y) * cos(THobst_1);
	Xcan_2 = (Xobst_2 - x) * cos(THobst_2) + (Yobst_2 - y) * sin(THobst_2);
	Ycan_2 = -(Xobst_2 - x) * sin(THobst_2) + (Yobst_2 - y) * cos(THobst_2);
	Xcan_3 = (Xobst_3 - x) * cos(THobst_3) + (Yobst_3 - y) * sin(THobst_3);
	Ycan_3 = -(Xobst_3 - x) * sin(THobst_3) + (Yobst_3 - y) * cos(THobst_3);
	Xcan_4 = (Xobst_4 - x) * cos(THobst_4) + (Yobst_4 - y) * sin(THobst_4);
	Ycan_4 = -(Xobst_4 - x) * sin(THobst_4) + (Yobst_4 - y) * cos(THobst_4);
	Xcan_5 = (Xobst_5 - x) * cos(THobst_5) + (Yobst_5 - y) * sin(THobst_5);
	Ycan_5 = -(Xobst_5 - x) * sin(THobst_5) + (Yobst_5 - y) * cos(THobst_5);
	Xcan_6 = (Xbord_1 - x) * cos(THbord_1) + (Ybord_1 - y) * sin(THbord_1);
	Ycan_6 = -(Xbord_1 - x) * sin(THbord_1) + (Ybord_1 - y) * cos(THbord_1);
	Xcan_7 = (Xbord_2 - x) * cos(THbord_2) + (Ybord_2 - y) * sin(THbord_2);
	Ycan_7 = -(Xbord_2 - x) * sin(THbord_2) + (Ybord_2 - y) * cos(THbord_2);
	Xcan_8 = (Xbord_3 - x) * cos(THbord_3) + (Ybord_3 - y) * sin(THbord_3);
	Ycan_8 = -(Xbord_3 - x) * sin(THbord_3) + (Ybord_3 - y) * cos(THbord_3);
	Xcan_9 = (Xbord_4 - x) * cos(THbord_4) + (Ybord_4 - y) * sin(THbord_4);
	Ycan_9 = -(Xbord_4 - x) * sin(THbord_4) + (Ybord_4 - y) * cos(THbord_4);
	Xcan_10 = (Xbord_5 - x) * cos(THbord_5) + (Ybord_5 - y) * sin(THbord_5);
	Ycan_10 = -(Xbord_5 - x) * sin(THbord_5) + (Ybord_5 - y) * cos(THbord_5);

	f << dot(x) == vx * cos(th) + vy * sin(th);			// u*cos(theta) + v*sin(theta)
	f << dot(y) == vx * sin(th) - vy * cos(th);			// u*sin(theta) - v*cos(theta)
	f << dot(th) == - w;						// - omega
	f << dot(vx) == (fl + fr - D_x * vx) / m + w * vy;		// (T_l + T_r - D_x*u)/m + omega*v
	f << dot(vy) == - D_y * vy / m - w * vx;			// - D_y*v/m - omega*u
	f << dot(w) == (l * (fl - fr) - D_t * w) / I_z; 		// (l*(T_l - T_r) - D_theta*omega)/I_z

    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function rf;
    Function rfN;

    rf << x << y << th << vx << vy << w << fl << fr;
    rfN << x << y << th << vx << vy << w;

    DMatrix W = eye<double>(rf.getDim() ) * 1e-6; // Reference
    W(0,0) = 250;
    W(1,1) = 250;
  //  W(3,3) = 100;
    W(6,6) = 1e-3;
    W(7,7) = 1e-3;
    DMatrix WN = eye<double>(rfN.getDim() ) * 1e-6; // Reference

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double tStart = 0.0;
    const double tEnd   = 25.0;

    OCP ocp( tStart, tEnd, tEnd );

   // ocp.minimizeLSQ( Q, h, r );
	ocp.minimizeLSQ( W, rf);
	ocp.minimizeLSQEndTerm( WN, rfN);
	ocp.subjectTo( f );

	ocp.subjectTo( 1 <= (pow(Xcan_1 * Aobst_1 ,2) + pow(Ycan_1 * Bobst_1 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_2 * Aobst_2 ,2) + pow(Ycan_2 * Bobst_2 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_3 * Aobst_3 ,2) + pow(Ycan_3 * Bobst_3 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_4 * Aobst_4 ,2) + pow(Ycan_4 * Bobst_4 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_5 * Aobst_5 ,2) + pow(Ycan_5 * Bobst_5 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_6 * Abord_1 ,2) + pow(Ycan_6 * Bbord_1 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_7 * Abord_2 ,2) + pow(Ycan_7 * Bbord_2 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_8 * Abord_3 ,2) + pow(Ycan_8 * Bbord_3 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_9 * Abord_4 ,2) + pow(Ycan_9 * Bbord_4 ,2 )));
	ocp.subjectTo( 1 <= (pow(Xcan_10 * Abord_5 ,2) + pow(Ycan_10 * Bbord_5 ,2 )));
	ocp.subjectTo( -50.0 <= fl <= 50.0 );
	ocp.subjectTo( -50.0 <= fr <= 50.0 );

	ocp.setNOD(50);


	// Export the code:
	// ----------------

	OCPexport mpc(ocp);
	mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
	mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

	mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);
	mpc.set(NUM_INTEGRATOR_STEPS, 100);

	mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
	mpc.set(QP_SOLVER, QP_QPOASES);
	mpc.set(HOTSTART_QP, NO);

	mpc.set(GENERATE_TEST_FILE, NO);
	mpc.set(GENERATE_MAKE_FILE, NO);
	mpc.set(GENERATE_MATLAB_INTERFACE, NO);

	if (mpc.exportCode( "mpc_code_gen_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
	}
