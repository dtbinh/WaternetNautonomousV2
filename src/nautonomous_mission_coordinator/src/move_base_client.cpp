
#include <nautonomous_mission_coordinator/move_base_client.h>

MoveBaseClient::MoveBaseClient(ros::NodeHandle node_handle) 
{
	move_base_action_client_ = new MoveBaseActionClient("move_base", true);

	map_center_point_.x = 0.0;
	map_center_point_.y = 0.0;
	map_center_point_.z = 0.0;

	map_center_subscriber_ = node_handle.subscribe("map_center_topic", 1, &MoveBaseClient::callbackMapCenter, this);

}

MoveBaseClient::~MoveBaseClient() 
{
	if(move_base_action_client_)
	{
		delete move_base_action_client_;
		move_base_action_client_ = nullptr;
	}
}

void MoveBaseClient::callbackMapCenter(const geometry_msgs::Point::ConstPtr& point_message)
{
	map_center_point_.x = point_message->x;
	map_center_point_.y = point_message->y;
	map_center_point_.z = point_message->z;
}	



bool MoveBaseClient::requestGoal(const geometry_msgs::Pose2D pose2d) 
{
	while (!move_base_action_client_->waitForServer(ros::Duration(5.0))) 
	{
		ROS_ERROR("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;
	
	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "odom";
	goal.target_pose.header.stamp = ros::Time::now();
	
	//Client publish the wanted position of the robot
	goal.target_pose.pose.position.x = pose2d.x - map_center_point_.x;
	goal.target_pose.pose.position.y = pose2d.y - map_center_point_.y;
	goal.target_pose.pose.position.z = 0;
	
	ROS_INFO("New move base goal: %f %f %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z);

	//Client publish the wanted orientation of the robot
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = std::sin(pose2d.theta * 0.5);
	goal.target_pose.pose.orientation.w = std::cos(pose2d.theta * 0.5);

	move_base_action_client_->sendGoal(goal);

	move_base_action_client_->waitForResult();

	return (move_base_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
	
}
