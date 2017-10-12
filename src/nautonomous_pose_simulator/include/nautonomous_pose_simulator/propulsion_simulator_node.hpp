/*
 * propulsion_simulator_node.hpp
 *
 *  Created on: Feb 9, 2017
 *      Author: zeeuwe01
 */

#ifndef PROPULSIONSIMULATORNODE_HPP_
#define PROPULSIONSIMULATORNODE_HPP_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>

double delta_time;

ros::Time current_time;
ros::Time last_time;

geometry_msgs::Twist position;
geometry_msgs::Twist velocity;
geometry_msgs::Twist delta;

geometry_msgs::Quaternion odom_quat;

// nav_msgs::OccupancyGrid nautonomous_occupancy_grid;

ros::Publisher odom_pub;


/*
 * If the propulsion topic arrives, read message and set the velocity x, y, th for the robot.
 */
void propulsionCallback(const geometry_msgs::Twist::ConstPtr& twist);

/*
 * Updates the map, with the grid and meta data from the message
 */
// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy);

void calculatePosition();

// Implement check if the simulator does not cross the border of the map, commented out since it cannot be tested currently.
// void calculateMapIndex();

void publishTransform(tf::TransformBroadcaster transform_broadcaster);

void publishOdometry();

void simulatePropulsion(ros::NodeHandle n, tf::TransformBroadcaster transform_broadcaster);


void positionCallback(const geometry_msgs::Pose2D::ConstPtr& pose2d);

#endif /* PROPULSIONSIMULATORNODE_HPP_ */


