#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>

nav_msgs::OccupancyGrid nautonomous_occupancy_grid;

double velocity_x = 0;
double velocity_y = 0;
double velocity_theta = 0;

double disturbance_x = 0;
double disturbance_y = 0;
double disturbance_theta = 0;

//Position of the robot
double position_x = 0.0;	//629267.0;
double position_y = 0.0;	//5807159.0;
double theta = 0.0;

/**
 *\brief If the propulsion topic arrives, read message and set the velocity x, y, th for the robot.
 *\param geometry_msgs::Twist twist
 *\return 
 */
void propulsionCallback(const geometry_msgs::Twist::ConstPtr& twist) {	
	//ROS_INFO("propulsion callback %f", twist->linear.x);
	velocity_x = twist->linear.x;
	velocity_y = twist->linear.y;
	velocity_theta = twist->angular.z;
}

void disturbanceCallback(const geometry_msgs::Twist::ConstPtr& twist) {
	disturbance_x = twist->linear.x;
	disturbance_y = twist->linear.y;
	disturbance_theta = twist->angular.z;
}

/**
 *\brief Updates the map, with the grid and meta data from the message
 *\param nav_msgs::OccupancyGrid occupancy
 *\return
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy) {
	nautonomous_occupancy_grid.data = occupancy->data;
	nautonomous_occupancy_grid.header = occupancy->header;
	nautonomous_occupancy_grid.info = occupancy->info;
}

void positionCallback(geometry_msgs::Pose2DPtr pose2d) {
	position_x = pose2d->x;
	position_y = pose2d->y;
	theta = pose2d->theta;
}

/**
 *\brief Main for propulsion_sim node. Subscribes and advertise topics. Constant updating map 
 *\param
 *\return
 */

int main(int argc, char **argv) {

	ros::init(argc, argv, "nautonomous_propulsion_simulator");

	ros::NodeHandle n;
	ros::NodeHandle n_private;
		
	//Topics to subscribe to, multiplexed propulsion and the dimensions of the map.
	ros::Subscriber propulsionSubscriber = n.subscribe(
			"twist_topic", 10, propulsionCallback);
	ros::Subscriber disturbanceSubscriber = n.subscribe(
			"disturbance_topic", 10, disturbanceCallback);

	ros::Subscriber mapSubscriber = n.subscribe("map_topic", 10, mapCallback);

	ros::Subscriber positionSubscriber = n.subscribe("initial_position_topic", 10, positionCallback);	

	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_combined_topic", 10); //odom_combined

	tf::TransformBroadcaster odom_broadcaster;


	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    ros::Rate r(15);
	
	while (n.ok()) {

		ros::spinOnce();               /// check for incoming messages
		current_time = ros::Time::now();

		/// compute odometry in a typical way given the velocities of the robot
		double dt 					= (current_time - last_time).toSec();
		double delta_propulsion_x 	= (velocity_x * cos(theta) - velocity_y * sin(theta)) * dt;
		double delta_propulsion_y 	= (velocity_x * sin(theta) + velocity_y * cos(theta)) * dt;

		double delta_theta = velocity_theta * dt;

		//only if the disturbance is measured from the nautonomous itself.
		//double delta_disturbance_x = (disturbance_x * cos(theta) - disturbance_y * sin(theta)) * dt;
		//double delta_disturbance_y = (disturbance_x * sin(theta) + disturbance_y * cos(theta)) * dt;

		double delta_disturbance_x = disturbance_x * dt;
		double delta_disturbance_y = disturbance_y * dt;

		double delta_x = delta_propulsion_x + delta_disturbance_x;
		double delta_y = delta_propulsion_y + delta_disturbance_y;

		//ROS_INFO("delta (%4.2f, %4.2f | %4.2f)", delta_x, delta_y, delta_theta);
		position_x 	+= delta_x;
		position_y 	+= delta_y;
		theta 		+= delta_theta;
		
		//ROS_INFO("pos (%4.2f, %4.2f | %4.2f)", position_x, position_y, theta);
		double degrees = (theta * 180.0 / M_PI);

		//x and y to index the occupancy grid map.
		// if(nautonomous_occupancy_grid.info.height){
		// 	int index_x = (position_x - nautonomous_occupancy_grid.info.origin.position.x)
		// 					/ nautonomous_occupancy_grid.info.resolution;
		// 	int index_y = (position_y - nautonomous_occupancy_grid.info.origin.position.y)
		// 			/ nautonomous_occupancy_grid.info.resolution;
		// 	//ROS_INFO("(%4.2f, %4.2f | %4.2f [%4.2f]) is index (%d, %d)", x, y, th, degrees, index_x, index_y);
		// 	int index = nautonomous_occupancy_grid.info.width * index_y + index_x;
		// 	if (index >= 0 && index <= (nautonomous_occupancy_grid.info.height*nautonomous_occupancy_grid.info.width)) {
		// 		int value = nautonomous_occupancy_grid.data[index];
		// 		//ROS_INFO("index %d value %d", index, value);

		// 		if (value > 50) {
		// 			ROS_INFO("Undoing position change");
		// 			//undo position change
		// 			position_x -= delta_x;
		// 			position_y -= delta_y;
		// 			theta -= delta_theta;
		// 		}
		// 	}
		// }

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = position_x;
		odom_trans.transform.translation.y = position_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time; 
		odom.header.frame_id = "odom"; //odom

		//set the position
		odom.pose.pose.position.x = position_x;
		odom.pose.pose.position.y = position_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link"; //base_link
		odom.twist.twist.linear.x = velocity_x;
		odom.twist.twist.linear.y = velocity_y;
		odom.twist.twist.angular.z = velocity_theta;

		//publish the message
		odom_pub.publish(odom);

		// Time
		last_time = current_time;
		r.sleep();
	}

	return 0;
}