#include <iostream>
#include <boost/array.hpp>

#include <boost/numeric/odeint.hpp>

#include <ros/ros.h>
#include <nautonomous_mpc_msgs/StageVariable.h>

#include <cmath>

#include <Eigenvalues>

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include "nautonomous_planning_and_control/Quaternion_conversion.h"


using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;

typedef boost::array< double , 6 > state_type;

ros::Subscriber next_state_sub;
ros::Subscriber action_sub;

ros::Publisher state_pub;

double D_x = 130;
double D_y = 5280;
double D_t = 750;
double m = 250;
double l = 0.5;
double I_z = 750;

double th = 0;
double vx = 0;
double vy = 0;
double w = 0;

double f1 = 0;
double f2 = 0;

float angle_correction = 0;
bool current_state_received = false;
bool first_time_received = true;
double Time_of_last_call;
double Current_loop_time;

nautonomous_mpc_msgs::StageVariable start_state;
nautonomous_mpc_msgs::StageVariable current_state;

nautonomous_mpc_msgs::StageVariable next_state;
geometry_msgs::PoseWithCovarianceStamped pose_state;

tf::StampedTransform transform_world_grid;


MatrixXd A(6,6);
MatrixXd C(3,6);
MatrixXd Q(3,3);
MatrixXd R(6,6);
MatrixXd P(6,6);
MatrixXd I(6,6);
MatrixXd S(6,6);
MatrixXd K(6,6);

VectorXd x_est(6);
VectorXd x_upd(6);
VectorXd y_est(3);
VectorXd y_meas(3);

nav_msgs::Odometry Position;

sensor_msgs::Imu Imu;

void lorenz( const state_type &x , state_type &dxdt , double t )
{
	th = x[2];
	vx = x[3];
	vy = x[4];
	w = x[5];
	dxdt[0] = vx * cos(th) + vy * sin(th);
	dxdt[1] =  vx * sin(th) - vy * cos(th);
	dxdt[2] =  - w;
	dxdt[3] =  (f1 + f2 - D_x * vx) / m + w * vy;
	dxdt[4] =  - D_y * vy / m - w * vx;	
	dxdt[5] =  (l * (f1 - f2) - D_t * w) / I_z; 
}

void write_lorenz( const state_type &x , const double t )
{
	cout << t << '\t' << x[0] << '\t' << x[1] << endl;
}

void Call_Observer()
{
        if (current_state_received)
        {
            f1 = start_state.T_l;
            f2 = start_state.T_r;

            A = MatrixXd::Zero(6,6);
            C = MatrixXd::Zero(3,6);
            Q = MatrixXd::Identity(6,6);
            R = MatrixXd::Identity(3,3);
            P = MatrixXd::Zero(6,6);
            I = MatrixXd::Identity(6,6);
            S = MatrixXd::Zero(3,3);
            K = MatrixXd::Zero(6,3);

            // Define initial values:
            // ----------------------
            double t_start = 0.0;
            double t_end = 1.0;

            state_type x = {{ start_state.x , start_state.y, start_state.theta, start_state.u, start_state.v, start_state.omega }}; // initial conditions
            integrate( lorenz , x , t_start , t_end , 0.5 , write_lorenz );

            x_est[0] = x[0];
            x_est[1] = x[1];
            x_est[2] = x[2];
            x_est[3] = x[3];
            x_est[4] = x[4];
            x_est[5] = x[5];

            A(0,2) = -start_state.u * sin(start_state.theta) + start_state.v * cos(start_state.theta);
            A(1,2) =  start_state.u * cos(start_state.theta) + start_state.v * sin(start_state.theta);
            A(0,3) = cos(start_state.theta);
            A(1,3) = sin(start_state.theta);
            A(3,3) = -D_x / m;
            A(4,3) = -start_state.omega;
            A(0,4) = sin(start_state.theta);
            A(1,4) = -cos(start_state.theta);
            A(3,4) = start_state.omega;
            A(4,4) = -D_y / m;
            A(2,5) = -1;
            A(3,5) = start_state.u;
            A(4,5) = -start_state.v;
            A(5,5) = -D_t / I_z;

            C(0,0) = 1;
            C(1,1) = 1;
            C(2,2) = 1;

            Q = Q * 1e6;
            R = R * 1e-6;

            P = A * P * A.transpose() + Q;

            y_est = y_meas - x_est.head(3);

            S = C * P * C.transpose() + R;

            K = P * C.transpose() * S.inverse();

            x_upd = x_est + K * y_est;

            P = (I - K * C) * P;

            next_state.x = x_upd[0];
            next_state.y = x_upd[1];
            next_state.theta = x_upd[2];
            next_state.u = x_upd[3];
            next_state.v = x_upd[4];
            next_state.omega = x_upd[5];
            next_state.T_l = f1;
            next_state.T_r = f2;

            state_pub.publish(next_state);
        }

}

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    pose_state = *pose_msg;

    if (first_time_received)
    {
        first_time_received = false;
        start_state.x = (pose_state.pose.pose.position.x - transform_world_grid.getOrigin().x()) * cos(tf::getYaw(transform_world_grid.getRotation())) + (pose_state.pose.pose.position.y - transform_world_grid.getOrigin().y()) * sin(tf::getYaw(transform_world_grid.getRotation()));
        start_state.y = -(pose_state.pose.pose.position.x - transform_world_grid.getOrigin().x()) * sin(tf::getYaw(transform_world_grid.getRotation())) + (pose_state.pose.pose.position.y - transform_world_grid.getOrigin().y()) * cos(tf::getYaw(transform_world_grid.getRotation()));
        start_state.theta = toEulerAngle(pose_state.pose.pose.orientation);
    }
    else
    {
        y_meas[0] = (pose_state.pose.pose.position.x - transform_world_grid.getOrigin().x()) * cos(tf::getYaw(transform_world_grid.getRotation())) + (pose_state.pose.pose.position.y - transform_world_grid.getOrigin().y()) * sin(tf::getYaw(transform_world_grid.getRotation()));
        y_meas[1] = -(pose_state.pose.pose.position.x - transform_world_grid.getOrigin().x()) * sin(tf::getYaw(transform_world_grid.getRotation())) + (pose_state.pose.pose.position.y - transform_world_grid.getOrigin().y()) * cos(tf::getYaw(transform_world_grid.getRotation()));
        y_meas[2] = toEulerAngle(pose_state.pose.pose.orientation);

    }

    current_state_received = true;
}

void action_cb (const geometry_msgs::Twist::ConstPtr& action_msg)
{
    start_state.T_l = 100 * (action_msg->linear.x - action_msg->angular.z);
    start_state.T_r = 100 * (action_msg->linear.x + action_msg->angular.z);
}

int main(int argc, char **argv)
{
	ros::init (argc, argv,"Ekf");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

        ROS_INFO_STREAM("Startup started");

        tf::TransformListener listener;

        ROS_INFO_STREAM("Transform listener setup");

        next_state_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/state/pose/robot_pose_ekf/odom_combined",10, pose_cb);
        action_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10, action_cb);

	state_pub = nh_private.advertise<nautonomous_mpc_msgs::StageVariable>("next_state",10);

        ros::Rate loop_rate(100);

        ros::Duration(1).sleep();

        ROS_INFO_STREAM("Startup finished");

        while (ros::ok())
        {
                listener.lookupTransform("/map", "/occupancy_grid", ros::Time(0), transform_world_grid);

                Current_loop_time = ros::Time::now().toSec();
                if (Current_loop_time - Time_of_last_call > 1)
                {
                        Time_of_last_call = Current_loop_time;
                        Call_Observer();
                }

                ros::spinOnce();
                loop_rate.sleep();
        }
        ROS_INFO_STREAM("Shizzle finished");
        return 0;
}
