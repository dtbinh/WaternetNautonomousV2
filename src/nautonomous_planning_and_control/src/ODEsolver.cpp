#include <acado_toolkit.hpp>
#include <acado_integrators.hpp>
#include <acado_gnuplot.hpp>
#include <acado/utils/acado_utils.hpp>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <std_msgs/Int8.h>

USING_NAMESPACE_ACADO
DVector waypoint;
visualization_msgs::Marker waypoint_marker;
visualization_msgs::Marker obstacle_marker;
geometry_msgs::Point p;

std::vector<double> waypoints_x;
std::vector<double> waypoints_y;

ros::Publisher marker_pub;
ros::Publisher marker_pub_2;
ros::Publisher waypoint_pub;

ros::Subscriber obstacle_sub;
ros::Subscriber start_sub;
ros::Subscriber waypoint_sub;

nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::StageVariable start_state;

double obstacle_x = 0;
double obstacle_y = 0;
double obstacle_a = 1;
double obstacle_b = 1;
double obstacle_th = 0;

double start_x = -10;
double start_y = 0;
double start_theta = 0;

void obstacle_cb( const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg )
{
	obstacle = *obstacle_msg;
	obstacle_x = obstacle.state.pose.position.x;
	obstacle_y = obstacle.state.pose.position.y;
	obstacle_a = obstacle.major_semiaxis;
	obstacle_b = obstacle.minor_semiaxis;
	obstacle_th = obstacle.state.pose.position.z;
}

void start_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& start_msg )
{
	start_state = *start_msg;
	start_x = start_state.x;
	start_y = start_state.y;
	start_theta = start_state.theta;

	// Define a right-hand side:
	// -------------------------
	DifferentialState x;
	DifferentialState y;
	TIME t;

	const double k1_max = 0.1;
	const double k2_max = 0.1;

	IntermediateState l;
	IntermediateState he11;
	IntermediateState he12;
	IntermediateState he21;
	IntermediateState h1;
	IntermediateState h2;
	IntermediateState gamma;

	double k1;
	double k2;
	
	DifferentialEquation f;

	l = ((cos(obstacle_th) * x + sin(obstacle_th) * y)/obstacle_a) * ((cos(obstacle_th) * x + sin(obstacle_th) * y)/obstacle_a) + ((-sin(obstacle_th) * x + cos(obstacle_th) * y)/obstacle_b) * ((-sin(obstacle_th) * x + cos(obstacle_th) * y)/obstacle_b) - 1;
	
	he11 = (obstacle_a * obstacle_a - obstacle_b * obstacle_b) * sin(obstacle_th) * cos(obstacle_th);
	he12 = obstacle_a * obstacle_a * cos(obstacle_th) * cos(obstacle_th) + obstacle_b * obstacle_b * sin(obstacle_th) * sin(obstacle_th);
	he21 = obstacle_b * obstacle_b * cos(obstacle_th) * cos(obstacle_th) + obstacle_a * obstacle_a * sin(obstacle_th) * sin(obstacle_th);

	gamma =  0.0792 * t * t * t - 0.02376  * t * t * t * t + 0.0019 * t * t * t * t * t ;

	k1 = k1_max;
	k2 = k2_max;

	h1 = gamma / (obstacle_a * obstacle_b) * (he11 * x - he12 * y);
	h2 = gamma / (obstacle_a * obstacle_b) * (he21 * x - he11 * y);

	f << dot(x ) == h1 - k1 * x * l;
	f << dot(y ) == h2 - k2 * y * l;

	// Define an integrator:
	// ---------------------

	IntegratorRK45 integrator(f);
	
	integrator.set( INTEGRATOR_PRINTLEVEL, LOW );
	integrator.set( PRINT_INTEGRATOR_PROFILE, YES );
	integrator.set( INTEGRATOR_TOLERANCE, 1.0e-8);
	integrator.set( INTEGRATOR_TYPE, INT_RK45);
	
	// Define initial values:
	// ----------------------
	
	double x_start[2] = {start_x - obstacle_x, start_y - obstacle_y};
	
	std::cout << "Start: (" << x_start[0] << "," << x_start[1] << ")" << std::endl;

	double t_start = 0.0;
	double t_end = 5.0;

	// Start the integration:
	// ----------------------
	Grid timeInterval( t_start, t_end, t_end * 2 );	
	integrator.freezeAll();
    	integrator.integrate( timeInterval, x_start );

	// Get the results:
	// ----------------
	
	VariablesGrid differentialStates;

        integrator.getX ( differentialStates );

	GnuplotWindow window;
	  window.addSubplot(differentialStates(0), "X-position" );
	  window.addSubplot(differentialStates(1), "Y-position" );
	window.plot();
		
	std::cout << "Route: " << differentialStates << std::endl;

	waypoint_marker.header.frame_id = "/my_frame";
	waypoint_marker.header.stamp = ros::Time::now();
	waypoint_marker.ns = "points_and_lines";
	waypoint_marker.action = visualization_msgs::Marker::ADD;
	waypoint_marker.pose.orientation.w = 1.0;
    	waypoint_marker.id = 0;
	waypoint_marker.type = visualization_msgs::Marker::POINTS;

	waypoint_marker.scale.x = 0.5;
	waypoint_marker.scale.y = 0.5;
	waypoint_marker.color.g = 1.0;
	waypoint_marker.color.a = 1.0;

	std::cout << "The amount of waypoints is: " << differentialStates.getDim() << std::endl;

	for (int i = 0; i < differentialStates.getDim()/2; i++)
	{
		waypoint = differentialStates.getVector(i);
		std::cout << "Waypoint " << i << " : (" << waypoint[0] << ", " << waypoint[1] << ")" << std::endl;
		p.x = waypoint[0];
      		p.y = waypoint[1];

		waypoints_x.push_back(p.x);
		waypoints_y.push_back(p.y);
	
      		waypoint_marker.points.push_back(p);
	
    		marker_pub.publish(waypoint_marker);
	}
	
	obstacle_marker.header.frame_id = "/my_frame";
	obstacle_marker.header.stamp = ros::Time::now();
	obstacle_marker.ns = "points_and_lines";
	obstacle_marker.action = visualization_msgs::Marker::ADD;
	obstacle_marker.pose.orientation.w = 1.0;
    	obstacle_marker.id = 0;
	obstacle_marker.type = visualization_msgs::Marker::CYLINDER;

	obstacle_marker.scale.x = 0.5;
	obstacle_marker.scale.y = 0.5;
	obstacle_marker.scale.z = 0.5;
	obstacle_marker.color.b = 1.0;
	obstacle_marker.color.a = 1.0;

	obstacle_marker.scale.x = obstacle_a;
	obstacle_marker.scale.y = obstacle_b;

	p.x = 0;
      	p.y = 0;
	
	obstacle_marker.points.push_back(p);

    	marker_pub_2.publish(obstacle_marker);

}

void next_waypoint_cb( const std_msgs::Int8::ConstPtr& ask_waypoint_msg )
{
	int point = ask_waypoint_msg->data;
	p.x = waypoints_x[point];
	p.y = waypoints_y[point];
	waypoint_pub.publish(p);
}

int main(int argc, char** argv) {

	ros::init (argc, argv,"Route_generator");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	marker_pub = nh_private.advertise<visualization_msgs::Marker>("waypoint_marker", 10);
	marker_pub_2 = nh_private.advertise<visualization_msgs::Marker>("obstacle_marker", 10);
	waypoint_pub = nh_private.advertise<geometry_msgs::Point>("next_waypoint", 10);

	obstacle_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacle>("obstacle",10,obstacle_cb);
	start_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("start",10,start_cb);
	waypoint_sub = nh.subscribe<std_msgs::Int8>("ask_for_next_waypoint", 10, next_waypoint_cb);

	ros::Rate poll_rate(100);
	while(marker_pub.getNumSubscribers() == 0){poll_rate.sleep();}
	while(marker_pub_2.getNumSubscribers() == 0){poll_rate.sleep();}
	
	ros::spin();

	return 0;
}

	
