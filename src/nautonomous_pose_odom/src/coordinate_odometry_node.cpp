#include <nautonomous_pose_odom/utm_odometry_node.h>

/**
 * Publishes odometry from the coordinate
 * @params sensor_msgs::NavSatFixConstPtr& fix fix coordinate
 */
void callback_coordinate(const nautonomous_pose_msgs::PointWithCovarianceStampedConstPtr& point_with_covariance_stamped)
{	
	nav_msgs::Odometry odom_msg;

	// Set header stamp
	odom_msg.header.stamp = point_with_covariance_stamped->header.stamp;

	if (old_time)
	{
		odom_msg.header.stamp = ros::Time::now();
	}

	// Set Frames
	odom_msg.header.frame_id = point_with_covariance_stamped->header.frame_id;
	
	if (!frame_id.empty()) 
	{
		odom_msg.header.frame_id = frame_id;
	}

	odom_msg.child_frame_id = child_frame_id;

	// Set Pose
	odom_msg.pose.pose.position.x = point_with_covariance_stamped->point.x;
	odom_msg.pose.pose.position.y = point_with_covariance_stamped->point.y;
	odom_msg.pose.pose.position.z = 0;

	// Set Quaternation
	odom_msg.pose.pose.orientation.x = 0;
	odom_msg.pose.pose.orientation.y = 0;
	odom_msg.pose.pose.orientation.z = 0;
	odom_msg.pose.pose.orientation.w = 1;

	// Extract covariances from gps position.
	double covariance_x = point_with_covariance_stamped->position_covariance[0];
	double covariance_y = point_with_covariance_stamped->position_covariance[4];
	double covariance_z = point_with_covariance_stamped->position_covariance[8];

	// Use ENU covariance to build XYZRPY covariance
	boost::array<double, 36> covariance = {{
		covariance_x, 0, 0, 0, 0, 0,
		0, covariance_y, 0, 0, 0, 0,
		0, 0, covariance_z, 0, 0, 0,
		0, 0, 0, rot_cov, 0, 0,  
		0, 0, 0, 0, rot_cov, 0, 
		0, 0, 0, 0, 0, rot_cov
	}};

	// Set Covariance
	odom_msg.pose.covariance = covariance;

	// Publish message
	odom_pub.publish(odom_msg);
}

/**
 * Subscribes to 'fix', publishes to 'odom'.
 */
int main (int argc, char **argv) 
{
    ros::init(argc, argv, "coordinate_odometry_node");

    ros::NodeHandle node;

	// Publisher
    odom_pub = node.advertise<nav_msgs::Odometry>("odom_coordinate", 10);

	// Params
    ros::NodeHandle priv_node("~");
    priv_node.param<std::string>("frame_id", frame_id, "gps_link");
    priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
	priv_node.param<bool>("old_time", old_time, false);

	// Subscriber
    ros::Subscriber coordinate_sub = node.subscribe("coordinate", 10, callback_coordinate);

    ros::spin();
}
