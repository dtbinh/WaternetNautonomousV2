#include <nautonomous_mission_coordinator/mission_coordinator_server.h>

/**
 *\brief Constructor for MissionServer
 *\param ros::NodeHandle nh_
 *\param string name
 *\return MissionCoordinatorServer
 */
MissionCoordinatorServer::MissionCoordinatorServer(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle, std::string name) :
		mission_coordinator_action_server_(node_handle, name.c_str(), boost::bind(&MissionCoordinatorServer::processMissionPlanGoal, this, _1),
				false) 
{
	mission_coordinator_action_server_.start();

	node_handle_ = node_handle;

	private_node_handle.param("routing_enabled", routing_enabled_, false);
	private_node_handle.param("map_enabled", map_enabled_, false);
	private_node_handle.param("planner_enabled", planner_enabled_, false);

	ROS_INFO("MCS: routing %i, map %i, planner %i", routing_enabled_, map_enabled_, planner_enabled_);
	
	move_base_client_ = new MoveBaseClient(node_handle_);
}

/**
 *\brief Empty deconstructor for MissionCoordinatorServer
 *\param 
 *\return MissionCoordinatorServer
 */
MissionCoordinatorServer::~MissionCoordinatorServer(void) 
{
	if(move_base_client_){
		delete move_base_client_;
		move_base_client_ = nullptr;
	}
}

/**
 *\brief Coordinate Routing from the Vaarkaart using a service request
 *\return success
 */
bool MissionCoordinatorServer::requestVaarkaartRoute()
{	
	//If the operation should be automatically planned, we use the vaarkaart to do so.
	if(!routing_enabled_)
	{	
		ROS_ERROR("MCS: Wanted to schedule automatic routing, however the routing is disabled.");
		return false;
	}
	ROS_INFO("MCS: Request Vaarkaart route");

	//Create routing request for the vaarkaart.
	nautonomous_routing_msgs::Route route_srv;
	//Assume that the start is at index 0 and the destination is at index 1 of the route.
	route_srv.request.start = route_[0];
	route_srv.request.destination = route_[1];

	ROS_INFO("MCS: Request Vaarkaart route 2");
	// Create the routing 
	ros::ServiceClient vaarkaart_client = node_handle_.serviceClient<nautonomous_routing_msgs::Route>("route");	
	if(vaarkaart_client.call(route_srv)) 
	{
		// Copy the path from the routing to the path vector	
		route_ = route_srv.response.route;
		route_ids_ = route_srv.response.route_ids;
	} 
	else 
	{
		ROS_ERROR("MCS: Failed request for routing");
		return false;
	}		

	return true;
}

/**
 *\brief Execute map cropping
 *\return success
 */
bool MissionCoordinatorServer::requestCropMap()
{
	if (!map_enabled_) {
		ROS_WARN("MCS: Skipping map cropping, because the map is not enabled");
		return true;
	}

	ROS_INFO("MCS: Coordinate Map Cropping");

	// Create a map cropping service request
	nautonomous_map_msgs::Crop crop_srv;
	crop_srv.request.route = route_;
	crop_srv.request.name = current_operation_.name;
	crop_srv.request.rectangular = !routing_enabled_;

	// Execute map cropping service call
	ros::ServiceClient map_cropper_client = node_handle_.serviceClient<nautonomous_map_msgs::Crop>("crop");

	if(map_cropper_client.call(crop_srv))
	{
		config_name_ = crop_srv.response.config_name;
	} 
	else 
	{
		ROS_ERROR("MCS: Failed request for map cropper!");
		return false;
	}	

	return true;
}

/**
 *\brief Coordinate map server
 *\return success
 */
bool MissionCoordinatorServer::requestGlobalMap()
{	
	if (!map_enabled_){
		ROS_WARN("MCS: Skipping loading global map");
		return true;
	}
	
	ROS_INFO("MCS: Request Global Map");

	// Create a map server service request
	nautonomous_map_msgs::Load load_srv;
	load_srv.request.config_name = config_name_;

	// Execute map server request
	ros::ServiceClient global_map_client = node_handle_.serviceClient<nautonomous_map_msgs::Load>("load");

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

	return true;
}

bool MissionCoordinatorServer::requestSegmentMap(int route_id)
{
	if (!map_enabled_){
		ROS_WARN("MCS: Skipping loading segment map");
		return true;
	}
	
	ROS_INFO("MCS: Request Segment Map");

	// Create a map server service request
	nautonomous_map_msgs::Load load_srv;

	std::string path_configuration = ros::package::getPath("nautonomous_configuration");
	load_srv.request.config_name = path_configuration + "/config/navigation/map/amsterdam_cropped_segment_" + std::to_string(route_id) + ".yaml";

	ROS_INFO(load_srv.request.config_name.c_str());

	// Execute map server request
	ros::ServiceClient global_map_client = node_handle_.serviceClient<nautonomous_map_msgs::Load>("load");
	
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

	return true;
}

/**
 *\brief Coordinate move base goal
 *\return success
 */
bool MissionCoordinatorServer::requestMoveBaseGoals()
{

	ROS_INFO("MCS: Request Move Base Goal");
	bool success = true;
	int route_size = route_.size();
	for (int route_index = 0; route_index < route_size; route_index++)
	{
		
		success = requestMoveBaseGoal(route_index);
		if (!success)
		{
			return success;
		}
	
	}

	return success;
}

bool MissionCoordinatorServer::requestMoveBaseGoal(int route_index)
{
	if(!planner_enabled_){
		ROS_WARN("MCS: Skipping sending move base a request for a goal");
		ros::Duration(5).sleep();
		return true;
	}

	bool success = move_base_client_->requestGoal(route_.at(route_index));
	
	if (!success)
	{
		ROS_ERROR("MCS: Move Base failed to execute goal!");
		return success;
	}

	// Publish the current progression.
	mission_plan_feedback_.feedback.progression = (int) ((route_index + 1) / route_.size()); //arbitrary value for now TODO
	mission_plan_feedback_.feedback.status = "Ok";
	mission_coordinator_action_server_.publishFeedback(mission_plan_feedback_);

	ros::spinOnce();

	return success;
}

/**
 *\brief Coordinate mission main function to coordinate tasks
 *\param nautonomous_operation_action::MissionPlanGoalConstPtr &goal
 *\return
 */
void MissionCoordinatorServer::processMissionPlanGoal(const nautonomous_mission_msgs::MissionPlanGoalConstPtr &goal) 
{	

	//Check token: TODO
	//authenticate

	bool success = true;

	// For each operation in the mission plan execute the operation.
	for(std::vector<nautonomous_mission_msgs::OperationPlan>::const_iterator operation_pointer = goal->operations.begin(); operation_pointer != goal->operations.end(); ++operation_pointer)
	{
		current_operation_ = *operation_pointer;

		// Copy the path from the action to the path vector	
		route_ = current_operation_.route;
		
		if (!current_operation_.automatic_routing) {
			success = customRoutingCoordination();		
		} else {
			success = automaticRoutingCoordination();
		}
		if (!success)
		{
			break;
		}
	}	
	// Return the progress and status for both success and failed action.
	if (success) 
	{
		ROS_INFO("MCS: Completed mission plan");
		mission_plan_result_.result.progression = 100;
		mission_plan_result_.result.status = mission_plan_feedback_.feedback.status;
		// Set the action state to succeeded
		mission_coordinator_action_server_.setSucceeded(mission_plan_result_);
	} 
	else 
	{
		ROS_INFO("MCS: Aborted mission plan");
		//TODO correct progression calculation
		mission_plan_result_.result.progression = 0;
		mission_plan_result_.result.status = "Failed coordinating mission request";
		// Set the action state to aborted
		mission_coordinator_action_server_.setAborted(mission_plan_result_);
	}
}

bool MissionCoordinatorServer::customRoutingCoordination()
{
	bool success = true;

	ROS_INFO("MCS: custom Routing Coordination");

	// Cropper
	success = requestCropMap();
	if (!success)
	{
		return success;
	}

	// Map Server
	success = requestGlobalMap();
	if (!success)
	{
		return success;
	}

	// Navigation stack
	success = requestMoveBaseGoals();
	if (!success)
	{
		return success;
	}

	return success;
} 

bool MissionCoordinatorServer::automaticRoutingCoordination()
{
	bool success = true;

	ROS_INFO("MCS: Automatic Routing Coordination");

	success = requestVaarkaartRoute();
	if (!success)
	{
		return success;
	}

	int route_size = route_ids_.size();
	for (int route_index = 0; route_index < route_size; route_index++)
	{
		// Map Server
		success = requestSegmentMap(route_ids_[route_index]);
		if (!success)
		{
			return success;
		}

		success = requestMoveBaseGoal(route_index);
		if (!success)
		{
			return success;
		}
	}

	return success;
}
