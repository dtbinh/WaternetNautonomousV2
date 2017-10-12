/*
 * ActionManager.h
 *
 *  Created on: Mar 19, 2016
 *      Author: zeeuwe01
 */

#ifndef MOVEBASEACTION_H_
#define MOVEBASEACTION_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

class MoveBaseClient{
private:
	MoveBaseActionClient* move_base_action_client_;
	
  	geometry_msgs::Point map_center_point_;
  	ros::Subscriber map_center_subscriber_;

	void callbackMapCenter(const geometry_msgs::Point::ConstPtr& point_message);
public:
	MoveBaseClient(ros::NodeHandle node_handle);
	~MoveBaseClient();
	bool requestGoal(const geometry_msgs::Pose2D pose2d);
};

#endif /* MOVEBASEACTION_H_ */
