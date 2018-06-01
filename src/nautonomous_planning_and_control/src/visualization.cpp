#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nautonomous_mpc_msgs/Obstacle.h>
#include <nautonomous_mpc_msgs/Obstacles.h>
#include <nautonomous_mpc_msgs/StageVariable.h>
#include <nautonomous_mpc_msgs/WaypointList.h>
#include <nautonomous_mpc_msgs/Waypoint.h>
#include <nautonomous_mpc_msgs/Route.h>
#include <nautonomous_planning_and_control/Quaternion_conversion.h>

ros::Publisher marker_pub;
ros::Publisher marker_pub_2;
ros::Publisher marker_pub_3;
ros::Publisher marker_pub_4;
ros::Publisher marker_pub_5;

ros::Subscriber EKF_sub;
ros::Subscriber obstacle_sub;
ros::Subscriber obstacles_sub;
ros::Subscriber waypoint_sub;
ros::Subscriber waypoints_sub;
ros::Subscriber route_sub;

nautonomous_mpc_msgs::StageVariable current_state;
nautonomous_mpc_msgs::Obstacle obstacle;
nautonomous_mpc_msgs::Obstacles obstacles;
nautonomous_mpc_msgs::Route route;

visualization_msgs::Marker line_strip;
visualization_msgs::Marker route_marker;
visualization_msgs::Marker point;
visualization_msgs::Marker obstacle_marker;
visualization_msgs::MarkerArray obstacle_array;
visualization_msgs::Marker obstacles_marker;
visualization_msgs::MarkerArray obstacles_array;
visualization_msgs::Marker waypoint_marker;
visualization_msgs::MarkerArray waypoint_array;

int accepted_waypoint_error;

geometry_msgs::Point p;
double ns = 0;

void EKF_cb(const nautonomous_mpc_msgs::StageVariable::ConstPtr& ekf_msg)
{
	current_state = *ekf_msg;
	p.x = ekf_msg->x;
      	p.y = ekf_msg->y;

      	line_strip.points.push_back(p);

    	marker_pub.publish(line_strip);
}

void obstacle_cb(const nautonomous_mpc_msgs::Obstacle::ConstPtr& obstacle_msg)
{
	obstacle = *obstacle_msg;
	obstacle_marker.scale.x = obstacle.major_semiaxis * 2;
	obstacle_marker.scale.y = obstacle.minor_semiaxis * 2;

	obstacle_marker.pose.position.x = obstacle.pose.position.x;
	obstacle_marker.pose.position.y = obstacle.pose.position.y;
	obstacle_marker.pose.position.z = -0.5;

	obstacle_marker.pose.orientation = toQuaternion(obstacle.pose.orientation.x, obstacle.pose.orientation.y, obstacle.pose.orientation.z);
	obstacle_marker.ns = ns;
	ns++;

	obstacle_array.markers.push_back(obstacle_marker);

    	marker_pub_2.publish(obstacle_array);

}

void obstacles_cb(const nautonomous_mpc_msgs::Obstacles::ConstPtr& obstacles_msg)
{
	obstacles = *obstacles_msg;
	for (int i = 0; i < obstacles.obstacles.size(); i++)
	{
		obstacle = obstacles.obstacles[i];
		obstacles_marker.scale.x = obstacle.major_semiaxis * 2;
		obstacles_marker.scale.y = obstacle.minor_semiaxis * 2;

		obstacles_marker.pose.position.x = obstacle.pose.position.x;
		obstacles_marker.pose.position.y = obstacle.pose.position.y;
		obstacles_marker.pose.position.z = -1.0;

		obstacles_marker.pose.orientation = toQuaternion(obstacle.pose.orientation.x, obstacle.pose.orientation.y, obstacle.pose.orientation.z);
		obstacles_marker.ns = ns;
		ns++;

		obstacles_array.markers.push_back(obstacles_marker);
	}

    	marker_pub_4.publish(obstacles_array);

}

void waypoint_cb(const nautonomous_mpc_msgs::Waypoint::ConstPtr& waypoint_msg)
{
	waypoint_marker.pose.position.x = waypoint_msg->stage.x;
	waypoint_marker.pose.position.y = waypoint_msg->stage.y;
	waypoint_marker.pose.position.z = 1.0;

	waypoint_marker.scale.x = waypoint_msg->Uncertainty.data * 2;
	waypoint_marker.scale.y = waypoint_msg->Uncertainty.data * 2;

	if ((waypoint_marker.pose.position.x > -1e6) && (waypoint_marker.pose.position.x < 1e6) && (waypoint_marker.pose.position.y > -1e6) && (waypoint_marker.pose.position.y < 1e6))
	{
		waypoint_array.markers.push_back(waypoint_marker);

		marker_pub_3.publish(waypoint_array);
	}
}

void waypoints_cb(const nautonomous_mpc_msgs::WaypointList::ConstPtr& waypoints_msg)
{
	waypoint_marker.pose.position.x = waypoints_msg->stages[0].x;
	waypoint_marker.pose.position.y = waypoints_msg->stages[0].y;
	waypoint_marker.pose.position.z = 1.0;

	waypoint_marker.scale.x = waypoints_msg->Uncertainty[0].data * 2;
	waypoint_marker.scale.y = waypoints_msg->Uncertainty[0].data * 2;

	if ((waypoint_marker.pose.position.x > -1e6) && (waypoint_marker.pose.position.x < 1e6) && (waypoint_marker.pose.position.y > -1e6) && (waypoint_marker.pose.position.y < 1e6))
	{
		waypoint_array.markers.push_back(waypoint_marker);

		marker_pub_3.publish(waypoint_array);
	}
}

void route_cb(const nautonomous_mpc_msgs::Route::ConstPtr& route_msg)
{
	route = *route_msg;

	for (int i = 0; i < route.waypoints.size(); i++)
	{
		p.x = route.waypoints[i].x;
      		p.y = route.waypoints[i].y;
	
      		route_marker.points.push_back(p);

    		marker_pub_5.publish(route_marker);
	}
}

int main(int argc, char **argv)
{
 	/* Initialize ROS */
	ros::init(argc, argv, "visualization");	
	ros::NodeHandle nh("");
	ros::NodeHandle nh_private("~");

	marker_pub = 		nh_private.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	marker_pub_2 = 		nh_private.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 10);
	marker_pub_3 = 		nh_private.advertise<visualization_msgs::MarkerArray>("waypoints",10);
	marker_pub_4 = 		nh_private.advertise<visualization_msgs::MarkerArray>("obstacles",10);
	marker_pub_5 = 		nh_private.advertise<visualization_msgs::Marker>("route",10);

	EKF_sub =	 	nh.subscribe<nautonomous_mpc_msgs::StageVariable>("/Ekf/next_state",10, EKF_cb);
	waypoint_sub = 		nh.subscribe<nautonomous_mpc_msgs::Waypoint>("/mission_coordinator/reference_state",10,waypoint_cb);
	waypoints_sub = 	nh.subscribe<nautonomous_mpc_msgs::WaypointList>("/mission_coordinator/reference_states",10,waypoints_cb);
	obstacle_sub = 		nh.subscribe<nautonomous_mpc_msgs::Obstacle>("/mission_coordinator/obstacle",10,obstacle_cb);
	obstacles_sub = 	nh.subscribe<nautonomous_mpc_msgs::Obstacles>("/mission_coordinator/obstacles",10,obstacles_cb);
	route_sub = 		nh.subscribe<nautonomous_mpc_msgs::Route>("/Route_generator/waypoint_route",10,route_cb);

	point.header.frame_id = line_strip.header.frame_id = "/map";
	point.header.stamp = line_strip.header.stamp = ros::Time::now();
	point.ns = line_strip.ns = "points_and_lines";
	point.action = line_strip.action = visualization_msgs::Marker::ADD;
	point.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    	point.id = 0;
	line_strip.id = 1;
    	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	point.type = visualization_msgs::Marker::POINTS;

	point.scale.x = 0.5;
	point.scale.y = 0.5;
	point.color.g = 1.0;
	point.color.a = 1.0;

	line_strip.scale.x = 0.2;
	line_strip.color.r = 1.0;
	line_strip.color.a = 1.0;

	route_marker.header.frame_id = obstacles_marker.header.frame_id = waypoint_marker.header.frame_id = obstacle_marker.header.frame_id = "/map";
	route_marker.header.stamp = waypoint_marker.header.stamp = obstacle_marker.header.stamp = ros::Time::now();
	obstacles_marker.ns = obstacle_marker.ns = "obstacle";
	waypoint_marker.ns = "waypoints";
	route_marker.ns = "route";
	route_marker.action = obstacles_marker.action = waypoint_marker.action = obstacle_marker.action = visualization_msgs::Marker::ADD;
	route_marker.pose.orientation.w = obstacles_marker.pose.orientation.w = waypoint_marker.pose.orientation.w = obstacle_marker.pose.orientation.w = 1.0;
    	route_marker.id = obstacles_marker.id = waypoint_marker.id = obstacle_marker.id = 0;
	obstacles_marker.type = obstacle_marker.type = visualization_msgs::Marker::CYLINDER;
	waypoint_marker.type = visualization_msgs::Marker::CYLINDER;
	route_marker.type = visualization_msgs::Marker::POINTS;

	obstacle_marker.scale.z = 0.5;
	obstacle_marker.color.r = 1.0;
	obstacle_marker.color.b = 1.0;
	obstacle_marker.color.a = 1.0;

	obstacles_marker.scale.z = 0.5;
	obstacles_marker.color.g = 1.0;
	obstacles_marker.color.b = 1.0;
	obstacles_marker.color.a = 1.0;

	waypoint_marker.scale.z = 0.5;
	waypoint_marker.color.g = 1.0;
	waypoint_marker.color.a = 1.0;
	waypoint_marker.pose.position.z = 0.0;

	route_marker.scale.x = 0.2;
	route_marker.scale.y = 0.2;
	route_marker.scale.z = 0.2;
	route_marker.color.b = 0.0;
	route_marker.color.g = 0.0;
	route_marker.color.r = 1.0;
	route_marker.color.a = 1.0;

	ros::spin();
}

