#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <nautonomous_obstacle_detection/Quaternion_conversion.h>


ros::Publisher marker_pub;
ros::Publisher marker_pub_2;
ros::Publisher marker_pub_3;
ros::Publisher marker_pub_4;

ros::Subscriber Detection_sub;
ros::Subscriber Tracking_sub;

visualization_msgs::Marker detect_obstacle_marker;
visualization_msgs::Marker track_obstacle_marker;
visualization_msgs::Marker detect_obstacle_arrow;
visualization_msgs::Marker track_obstacle_arrow;

visualization_msgs::MarkerArray detection_obstacle_array;
visualization_msgs::MarkerArray tracking_obstacle_array;

void detection_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	for (int i = 0; i < obstacle_msg->obstacles.size(); i++)
	{
		detect_obstacle_marker.pose.position.x = obstacle_msg->obstacles.at(i).pose.position.x;
		detect_obstacle_marker.pose.position.y = obstacle_msg->obstacles.at(i).pose.position.y;
		detect_obstacle_marker.pose.position.z = 0;
		detect_obstacle_marker.pose.orientation = toQuaternion(0,0,obstacle_msg->obstacles.at(i).pose.orientation.z);
		detect_obstacle_marker.scale.x = obstacle_msg->obstacles.at(i).major_semiaxis;
		detect_obstacle_marker.scale.y = obstacle_msg->obstacles.at(i).minor_semiaxis;
		detect_obstacle_marker.ns = i;
		detection_obstacle_array.markers.push_back(detect_obstacle_marker);

		detect_obstacle_arrow.pose.position.x = obstacle_msg->obstacles.at(i).pose.position.x;
		detect_obstacle_arrow.pose.position.y = obstacle_msg->obstacles.at(i).pose.position.y;
		detect_obstacle_arrow.pose.position.z = 0;
		detect_obstacle_arrow.pose.orientation = toQuaternion(0,0,obstacle_msg->obstacles.at(i).pose.orientation.z);
		detect_obstacle_arrow.scale.x = obstacle_msg->obstacles.at(i).twist.linear.x * 4;
		detect_obstacle_arrow.scale.y = obstacle_msg->obstacles.at(i).twist.linear.x;
		detect_obstacle_arrow.ns = i + obstacle_msg->obstacles.size();
		detection_obstacle_array.markers.push_back(detect_obstacle_arrow);
	}

    	marker_pub.publish(detection_obstacle_array);
	detection_obstacle_array.markers.clear();

}

void tracking_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacle_msg)
{
	for (int i = 0; i < obstacle_msg->obstacles.size(); i++)
	{
		track_obstacle_marker.pose.position.x = obstacle_msg->obstacles.at(i).pose.position.x;
		track_obstacle_marker.pose.position.y = obstacle_msg->obstacles.at(i).pose.position.y;
		track_obstacle_marker.pose.position.z = 1;
		track_obstacle_marker.pose.orientation = toQuaternion(0,0,obstacle_msg->obstacles.at(i).pose.orientation.z);
		track_obstacle_marker.scale.x = obstacle_msg->obstacles.at(i).major_semiaxis;
		track_obstacle_marker.scale.y = obstacle_msg->obstacles.at(i).minor_semiaxis;
		track_obstacle_marker.ns = i;
		tracking_obstacle_array.markers.push_back(track_obstacle_marker);

		track_obstacle_arrow.pose.position.x = obstacle_msg->obstacles.at(i).pose.position.x;
		track_obstacle_arrow.pose.position.y = obstacle_msg->obstacles.at(i).pose.position.y;
		track_obstacle_arrow.pose.position.z = 1;
		track_obstacle_arrow.pose.orientation = toQuaternion(0,0,obstacle_msg->obstacles.at(i).pose.orientation.z);
		track_obstacle_arrow.scale.x = obstacle_msg->obstacles.at(i).twist.linear.x * 4;
		track_obstacle_arrow.scale.y = obstacle_msg->obstacles.at(i).twist.linear.x;
		track_obstacle_arrow.ns = i + obstacle_msg->obstacles.size();
		tracking_obstacle_array.markers.push_back(track_obstacle_arrow);
	}

    	marker_pub_2.publish(tracking_obstacle_array);
	tracking_obstacle_array.markers.clear();
}

int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "visualization");	
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	marker_pub = 		nh_private.advertise<visualization_msgs::MarkerArray>("detection_obstacles",10);
	marker_pub_2 = 		nh_private.advertise<visualization_msgs::MarkerArray>("tracking_obstacles",10);

	Detection_sub =	 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_detection/obstacles",10, detection_cb);
	Tracking_sub = 		nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/Obstacle_tracking/obstacles",10, tracking_cb);

	detect_obstacle_marker.header.frame_id = track_obstacle_marker.header.frame_id = "/map";
	detect_obstacle_marker.header.stamp = track_obstacle_marker.header.stamp = ros::Time::now();
	detect_obstacle_marker.action = track_obstacle_marker.action = visualization_msgs::Marker::ADD;
    	detect_obstacle_marker.id = track_obstacle_marker.id = 0;
	detect_obstacle_marker.type = track_obstacle_marker.type = visualization_msgs::Marker::CYLINDER;

	detect_obstacle_arrow.header.frame_id = track_obstacle_arrow.header.frame_id = "/map";
	detect_obstacle_arrow.header.stamp = track_obstacle_arrow.header.stamp = ros::Time::now();
	detect_obstacle_arrow.action = track_obstacle_arrow.action = visualization_msgs::Marker::ADD;
    	detect_obstacle_arrow.id = track_obstacle_arrow.id = 0;
	detect_obstacle_arrow.type = track_obstacle_arrow.type = visualization_msgs::Marker::ARROW;

	detect_obstacle_marker.scale.z = 0.5;
	detect_obstacle_marker.color.r = 1.0;
	detect_obstacle_marker.color.b = 1.0;
	detect_obstacle_marker.color.a = 0.5;

	track_obstacle_marker.scale.z = 0.5;
	track_obstacle_marker.color.g = 1.0;
	track_obstacle_marker.color.b = 1.0;
	track_obstacle_marker.color.a = 0.5;

	detect_obstacle_arrow.scale.z = 0.5;
	detect_obstacle_arrow.color.r = 1.0;
	detect_obstacle_arrow.color.b = 1.0;
	detect_obstacle_arrow.color.a = 1.0;

	track_obstacle_arrow.scale.z = 0.5;
	track_obstacle_arrow.color.g = 1.0;
	track_obstacle_arrow.color.b = 1.0;
	track_obstacle_arrow.color.a = 1.0;

	ros::spin();
}

