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
#include <cmath>
#include <nautonomous_planning_and_control/Quaternion_conversion.h>
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
nautonomous_mpc_msgs::Route route;

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
double gamma_max = 0.1;

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
double t_end = 50.0;

typedef boost::array< double , 2 > state_type;
