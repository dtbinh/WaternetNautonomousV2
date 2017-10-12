#include <ros/ros.h>

#include <std_msgs/Int32.h>

#include <nautonomous_mission_coordinator/mission_coordinator_server.h>

/**
 *\brief Creates MissionServer (mission_server.cpp) and MoveBaseActionClient (move_base_action_client.cpp). Loops through goals from MissionServer
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mission_coordinator_node");
	ros::NodeHandle node_handle;
	ros::NodeHandle node_private_handle("~");

	ROS_INFO("Starting coordinator mission server");
	MissionCoordinatorServer mission_coordinator_server(node_handle, node_private_handle, "server");
	
	ros::spin();
}
