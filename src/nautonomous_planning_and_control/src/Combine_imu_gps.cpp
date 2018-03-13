#include <iostream>
#include <boost/array.hpp>

#include <boost/numeric/odeint.hpp>

#include <ros/ros.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_pose_msgs/PointWithCovarianceStamped.h>

#include <cmath>

#include <Eigenvalues>

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"

ros::Subscriber gps_sub;
ros::Subscriber imu_sub;

ros::Publisher state_pub;

nautonomous_pose_msgs::PointWithCovarianceStamped Position;
nav_msgs::Odometry Odom;

sensor_msgs::Imu Imu;

void gps_cb(const nautonomous_pose_msgs::PointWithCovarianceStamped::ConstPtr& pos_msg)
{
	Position = *pos_msg;
	Odom.header = Position.header;
	Odom.header.frame_id = "/gps_link";
//	Odom.pose.pose.position = Position.point;

	state_pub.publish(Odom);

}

void imu_cb(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	Imu = *imu_msg;
	Odom.pose.pose.orientation = Imu.orientation;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv,"Combi");
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	imu_sub = nh.subscribe<sensor_msgs::Imu>("/sensor/imu/imu",10,imu_cb);
	gps_sub = nh.subscribe<nautonomous_pose_msgs::PointWithCovarianceStamped>("/utm",10,gps_cb);

	state_pub = nh_private.advertise<nav_msgs::Odometry>("state",10);

	ros::spin();

	return 0;
}
