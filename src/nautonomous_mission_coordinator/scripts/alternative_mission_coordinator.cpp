#include <ros/ros.h>
#include <vector>
#include <nautonomous_mission_coordinator/mission_coordinator_server.h>
#include <geometry_msgs/Pose2D.h>
#include <nautonomous_map_msgs/Crop.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mission_Coordinator");
	ros::NodeHandle node_handle_;
  	std::string config_name_;
	// Create a map cropping service request
	nautonomous_map_msgs::Crop crop_srv;
	geometry_msgs::Pose2D waypoint;
	std::vector<geometry_msgs::Pose2D> CompleteRoute;

	//int x_pos[] = {626737,626742,626735,626730,626737};
	//int y_pos[] = {5807621,5807627,5807632,5807625,5807621};

	int x_pos[] = {628714,629118,628714};
	int y_pos[] = {5802769,5802739,5802769};

	int t_pos[] = {0,0,0,0,0};
	int route_length = 3;

	for (int i = 0; i < route_length ;i++)
	{
		waypoint.x = x_pos[i];
		waypoint.y = y_pos[i];
		waypoint.theta = t_pos[i];
		CompleteRoute.push_back(waypoint);
	}

	crop_srv.request.route = CompleteRoute;

	crop_srv.request.name = "Testgracht_deel_1";
	crop_srv.request.rectangular = true;

	// Execute map cropping service call
	ros::ServiceClient map_cropper_client = node_handle_.serviceClient<nautonomous_map_msgs::Crop>("/crop");

	if(map_cropper_client.call(crop_srv))
	{
		config_name_ = crop_srv.response.config_name;
	} 
	else 
	{
		ROS_ERROR("MCS: Failed request for map cropper!");
		return false;
	}	

	
	ROS_INFO("MCS: Request Segment Map");

	// Create a map server service request
	nautonomous_map_msgs::Load load_srv;

	std::string path_configuration = ros::package::getPath("nautonomous_configuration");
	load_srv.request.config_name = path_configuration + "/config/navigation/map/amsterdam_cropped_" + crop_srv.request.name + ".yaml";

	ROS_INFO(load_srv.request.config_name.c_str());

	// Execute map server request
	ros::ServiceClient global_map_client = node_handle_.serviceClient<nautonomous_map_msgs::Load>("/load");
	
	if(global_map_client.call(load_srv))
	{
		std::string status = load_srv.response.status;
		//TODO what to do with the status.
	} 
	else 
	{
		ROS_ERROR("MCS: Failed request for global map server!");
		return false;
	}


}
