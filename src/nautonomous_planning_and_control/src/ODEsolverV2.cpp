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
#include <std_msgs/Int8.h>
#include <cmath>

using namespace std;
using namespace boost::numeric::odeint;

visualization_msgs::Marker waypoint_marker;
visualization_msgs::Marker obstacle_marker;
geometry_msgs::Point p;
geometry_msgs::Quaternion q;

std::vector<double> waypoints_x;
std::vector<double> waypoints_y;
int length = 0;

ros::Publisher marker_pub;
ros::Publisher marker_pub_2;
ros::Publisher waypoint_pub;

ros::Subscriber obstacle_sub;
ros::Subscriber start_sub;
ros::Subscriber waypoint_sub;

nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::Route route;

double obstacle_x = 0;
double obstacle_y = 0;
double obstacle_a = 1;
double obstacle_b = 1;
double obstacle_th = 0;

double start_x = -10;
double start_y = 0;
double start_theta = 0;

const double k1_max = 0.02;
const double k2_max = 0.02;

double l;
double he11;
double he12;
double he21;
double h1;
double h2;
double gammaF;

double k1;
double k2; 

typedef boost::array< double , 2 > state_type;



void toQuaternion(double pitch, double roll, double yaw)
{
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
}



void lorenz( const state_type &x , state_type &dxdt , double t )
{

	l = pow((cos(obstacle_th) * x[0] + sin(obstacle_th) * x[1])/obstacle_a,2) + pow((-sin(obstacle_th) * x[0] + cos(obstacle_th) * x[1])/obstacle_b,2) - 1;
	
	he11 = (pow(obstacle_a,2) - pow(obstacle_b,2)) * sin(obstacle_th) * cos(obstacle_th);
	he12 = pow(obstacle_a * cos(obstacle_th),2) + pow(obstacle_b * sin(obstacle_th),2);
	he21 = pow(obstacle_b * cos(obstacle_th),2) + pow(obstacle_a * sin(obstacle_th),2);

	if (t < 5)
	{
		gammaF =  0.0792 * pow(t,3) - 0.02376  * pow(t,4) + 0.0019 * pow(t,5) ;	
	}
	else
	{
		gammaF = 1;
	}

	k1 = k1_max;
	k2 = k2_max;

	h1 = gammaF / (obstacle_a * obstacle_b) * (he11 * x[0] - he12 * x[1]);
	h2 = gammaF / (obstacle_a * obstacle_b) * (he21 * x[0] - he11 * x[1]);

    	dxdt[0] = h1 - k1 * x[0] * l;
    	dxdt[1] = h2 - k2 * x[1] * l;
}

void write_lorenz( const state_type &x , const double t )
{
	cout << t << '\t' << x[0] + obstacle_x << '\t' << x[1] + obstacle_y << endl;
	waypoints_x.push_back((double)x[0]+ obstacle_x)  ;
	waypoints_y.push_back((double)x[1]+ obstacle_y) ;
	length++;
	cout << "Write array" <<endl;
}

void obstacle_cb( const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg )
{
	obstacle = *obstacle_msg;
	obstacle_x = obstacle.state.pose.position.x;
	obstacle_y = obstacle.state.pose.position.y;
	obstacle_a = obstacle.major_semiaxis;
	obstacle_b = obstacle.minor_semiaxis;
	obstacle_th = obstacle.state.pose.orientation.z;
		
	std::cout << "Obstacle: " << obstacle << std::endl;
}

void start_cb( const nautonomous_mpc_msgs::StageVariable::ConstPtr& start_msg )
{
	start_state = *start_msg;
	start_x = start_state.x;
	start_y = start_state.y;
	start_theta = start_state.theta;

	std::cout << "Start: " << start_state << std::endl;

	// Define initial values:
	// ----------------------
	double t_start = 0.0;
	double t_end = 10.0;

    	state_type x = {{ start_x - obstacle_x , start_y - obstacle_y }}; // initial conditions
    	integrate( lorenz , x , t_start , t_end , 0.5 , write_lorenz );

	// Get the results:
	// ---------------	

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

	std::cout << "The amount of waypoints is: " << length << std::endl;

	for (int i = 0; i < length; i++)
	{
		std::cout << "Waypoint " << i << " : (" << waypoints_x[i]  << ", " << waypoints_y[i] << ")" << std::endl;
		p.x = waypoints_x[i];
      		p.y = waypoints_y[i];

/*		waypoints_x.push_back(p.x);
		waypoints_y.push_back(p.y);*/
	
      		waypoint_marker.points.push_back(p);

		route.waypoints.push_back(p);

    		marker_pub.publish(waypoint_marker);
	}
	
	obstacle_marker.header.frame_id = "/my_frame";
	obstacle_marker.header.stamp = ros::Time::now();
	obstacle_marker.ns = "points_and_lines";
	obstacle_marker.action = visualization_msgs::Marker::ADD;
	obstacle_marker.pose.orientation.w = 1.0;
    	obstacle_marker.id = 0;
	obstacle_marker.type = visualization_msgs::Marker::CYLINDER;

	obstacle_marker.scale.z = 0.5;
	obstacle_marker.color.b = 1.0;
	obstacle_marker.color.a = 1.0;

	obstacle_marker.scale.x = obstacle_a * 2;
	obstacle_marker.scale.y = obstacle_b * 2;

	obstacle_marker.pose.position.x = obstacle_x;
	obstacle_marker.pose.position.y = obstacle_y;

	toQuaternion(0,0,obstacle_th);

	obstacle_marker.pose.orientation = q;

	
	obstacle_marker.points.push_back(p);

    	marker_pub_2.publish(obstacle_marker);
	
	waypoint_pub.publish(route);
	
	waypoints_x.clear();	
	waypoints_y.clear();
	route.waypoints.clear();
	length = 0;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv,"Route_generator");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");
	ros::Duration(1).sleep();

	marker_pub = nh_private.advertise<visualization_msgs::Marker>("waypoint_marker", 10);
	marker_pub_2 = nh_private.advertise<visualization_msgs::Marker>("obstacle_marker", 10);
	waypoint_pub = nh_private.advertise<nautonomous_mpc_msgs::Route>("waypoint_route", 10);

	obstacle_sub = nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",10,obstacle_cb);
	start_sub = nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/mission_coordinator/start",10,start_cb);

	ros::Duration(1).sleep();
	//ros::Rate poll_rate(100);
	//while(marker_pub.getNumSubscribers() == 0){poll_rate.sleep();}
	//while(marker_pub_2.getNumSubscribers() == 0){poll_rate.sleep();}
	
	std::cout << "Spin" <<std::endl;
	ros::spin();

	return 0;
}
