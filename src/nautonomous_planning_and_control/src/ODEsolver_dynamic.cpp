#include <iostream>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/Route.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <nautonomous_planning_and_control/Quaternion_conversion.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace boost::numeric::odeint;

geometry_msgs::Point p;
geometry_msgs::Quaternion q;

std::vector<double> waypoints_x;
std::vector<double> waypoints_y;
int length = 0;

ros::Publisher waypoint_pub;

ros::Subscriber obstacle_sub;
ros::Subscriber start_sub;
ros::Subscriber waypoint_sub;
ros::Subscriber start_time_sub;

nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::StageVariable start_state;

nav_msgs::Path route;
geometry_msgs::PoseStamped Pose;

double obstacle_x = 0;
double obstacle_y = 0;
double obstacle_a = 1;
double obstacle_b = 1;
double obstacle_th = 0;
double obstacle_u = 1;
double obstacle_v = 1;
double obstacle_w = 0;
double start_time = 0;
double current_time = 0;

double start_x = -10;
double start_y = 0;
double start_theta = 0;
double x0;
double x1;

double k_max = 0.1;
double gamma_max = 0.15;

double l;
double he11;
double he12;
double he21;
double h1;
double h2;
double gammaF;

double k1;
double k2; 

double t_start = 0.0;
double t_end = 60.0;

typedef boost::array< double , 2 > state_type;

double safety_margin = 0.5;

void Boat_model( const state_type &x , state_type &dxdt , double t )
{
	x0 = x[0] - (obstacle_x + t * (obstacle_u * cos(obstacle_th) - obstacle_v * sin(obstacle_th)));
	x1 = x[1] - (obstacle_y + t * (obstacle_u * sin(obstacle_th) + obstacle_v * cos(obstacle_th)));
	
	l = pow((cos(obstacle_th) * x0 + sin(obstacle_th) * x1)/obstacle_a,2) + pow((-sin(obstacle_th) * x0 + cos(obstacle_th) * x1)/obstacle_b,2) - 1;
	
	he11 = (pow(obstacle_a,2) - pow(obstacle_b,2)) * sin(obstacle_th) * cos(obstacle_th);
	he12 = pow(obstacle_a * cos(obstacle_th),2) + pow(obstacle_b * sin(obstacle_th),2);
	he21 = pow(obstacle_b * cos(obstacle_th),2) + pow(obstacle_a * sin(obstacle_th),2);

	/*if (t < 5)
	{
		gammaF = gamma_max * (0.0792 * pow(t,3) - 0.02376  * pow(t,4) + 0.0019 * pow(t,5));	
		k1 =  k_max*(0.0792 * pow(t,3) - 0.02376  * pow(t,4) + 0.0019 * pow(t,5));	
	}
	else
	{*/
		gammaF = gamma_max;
		k1 = k_max;
	//}

	k2 = k1;

	h1 = gammaF / (obstacle_a * obstacle_b) * (he11 * x0 - he12 * x1);
	h2 = gammaF / (obstacle_a * obstacle_b) * (he21 * x0 - he11 * x1);

    	dxdt[0] = h1 - k1 * x0 * l;
    	dxdt[1] = h2 - k2 * x1 * l;
}

void write_waypoints( const state_type &x , const double t )
{
	cout << "Waypoint " << t << " : (" << x[0]  << ", " << x[1] << ")" << endl;
	waypoints_x.push_back((double)x[0])  ;
	waypoints_y.push_back((double)x[1]) ;
	length++;
}

void obstacle_cb( const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg )
{
	obstacle = *obstacle_msg;
	obstacle_x = obstacle.state.pose.position.x;
	obstacle_y = obstacle.state.pose.position.y;
	obstacle_a = obstacle.major_semiaxis + safety_margin;
	obstacle_b = obstacle.minor_semiaxis + safety_margin;
	obstacle_th = obstacle.state.pose.orientation.z;
	obstacle_u = obstacle.state.twist.linear.x;
	obstacle_v = obstacle.state.twist.linear.y;
	obstacle_w = obstacle.state.twist.angular.z;
}

void start_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& start_msg )
{
	ros::Time begin = ros::Time::now();
	start_state = *start_msg;
	start_x = start_state.x;
	start_y = start_state.y;
	start_theta = start_state.theta;

	// Define initial values:
	// ----------------------
    	state_type x = {{ start_x , start_y }}; // initial conditions
    	integrate( Boat_model , x , t_start , t_end , 0.1 , write_waypoints );

	// Get the results:
	// ---------------	
	route.poses.clear();

	for (int i = 0; i < length; i++)
	{
		cout << "Waypoint " << i << " : (" << waypoints_x[i]  << ", " << waypoints_y[i] << ")" << endl;
		Pose.pose.position.x = waypoints_x[i];
      		Pose.pose.position.y = waypoints_y[i];

		route.poses.push_back(Pose);
	}
	
	waypoint_pub.publish(route);
	
	waypoints_x.clear();	
	waypoints_y.clear();
	route.poses.clear();
	length = 0;

	ros::Time end = ros::Time::now();
	cout << "Elapsed time is: " << end.toSec()-begin.toSec() << endl;

}

int main(int argc, char **argv)
{
	ros::init (argc, argv,"Route_generator");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");


	nh_private.getParam("k_max", k_max);
	nh_private.getParam("gamma_max", gamma_max);
	nh_private.getParam("final_time", t_end);
	nh_private.getParam("safety_margin", safety_margin);

	waypoint_pub = nh_private.advertise<nav_msgs::Path>("waypoint_route", 10);

	obstacle_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",10,obstacle_cb);
	start_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);
	
	route.header.stamp = ros::Time::now();
	route.header.frame_id = "/map";

	cout << "spin" <<endl;
	ros::spin();
	return 0;
}
