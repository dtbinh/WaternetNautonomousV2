#ifndef MISSIONSERVER_H_
#define MISSIONSERVER_H_

#include <vector>
#include <string> 

#include <ros/ros.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Pose2D.h>

#include <nautonomous_map_msgs/Crop.h>
#include <nautonomous_map_msgs/Load.h>

#include <nautonomous_mission_msgs/MissionPlanAction.h>

#include <nautonomous_routing_msgs/Route.h>

#include <nautonomous_mission_coordinator/move_base_client.h>

typedef actionlib::SimpleActionServer<nautonomous_mission_msgs::MissionPlanAction> MissionCoordinatorActionServer;

class MissionCoordinatorServer
{
private:
  ros::NodeHandle node_handle_;

  MissionCoordinatorActionServer mission_coordinator_action_server_;

  std::vector<geometry_msgs::Pose2D> route_;
  std::vector<int> route_ids_;

  std::string config_name_;

  bool routing_enabled_;
  bool map_enabled_;
  bool planner_enabled_;

  MoveBaseClient* move_base_client_;
  
  nautonomous_mission_msgs::MissionPlanFeedback mission_plan_feedback_;
  nautonomous_mission_msgs::MissionPlanResult mission_plan_result_;

  nautonomous_mission_msgs::OperationPlan current_operation_;

public:
  MissionCoordinatorServer(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle, std::string name);
  ~MissionCoordinatorServer();

    /**
  *\brief Coordinate Routing from the Vaarkaart using a service request
  *\param std::vector<geometry_msgs::Pose2D> &path
  *\return success
  */
  bool requestVaarkaartRoute();
  /**
  *\brief Coordinate map cropping
  *\param std::basic_string<char> &config_file_name
  *\return success
  */
  bool requestCropMap();

  /**
  *\brief Coordinate map server
  *\param std::basic_string<char> &config_file_name
  *\return success
  */
  bool requestGlobalMap();

  bool requestSegmentMap(int route_id);

  /**
  *\brief Coordinate move base goal
  *\param std::vector<geometry_msgs::Pose2D> &path
  *\return success
  */
  bool requestMoveBaseGoals();

  bool requestMoveBaseGoal(int route_index);

  /**
  *\brief Empty constructor for MissionCoordinator
  *\param nautonomous_operation_action::MissionGoalConstPtr &goal
  *\return
  */
  void processMissionPlanGoal(const nautonomous_mission_msgs::MissionPlanGoalConstPtr &goal);

  bool customRoutingCoordination();

  bool automaticRoutingCoordination();
};

#endif /* MISSIONSERVER_H_ */
