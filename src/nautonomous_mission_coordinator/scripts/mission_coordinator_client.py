#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

from geometry_msgs.msg import Pose2D

from nautonomous_mission_msgs.msg import MissionPlanAction, OperationPlan, MissionPlanGoal

operation_name = "coenhaven_dry_dock"

def mission_plan_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('server', MissionPlanAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    route_poses = []
    # Creates a goal to send to the action server.
    if operation_name == "coenhaven": 
        #route_poses = [Pose2D(626748, 5807649, 0), Pose2D(626765, 5807638, 0), Pose2D(626776, 5807655, 0), Pose2D(626759, 5807666, 0)]
	route_poses = [Pose2D(626759, 5807666, 0), Pose2D(626776, 5807655, 0), Pose2D(626765, 5807638, 0), Pose2D(626748, 5807649, 0)]
    elif operation_name == "canals":
        route_poses = [Pose2D(628777, 5804903, 0), Pose2D(629444, 5803420, 0)]
    elif operation_name == "coenhaven_dry_dock":
        route_poses = [Pose2D(626737, 5807621, 0), Pose2D(626742, 5807627, 0), Pose2D(626735, 5807632, 0), Pose2D(626730, 5807625, 0), Pose2D(626737, 5807621, 0)]
    elif operation_name == "handhaving":
        route_poses = [Pose2D(628976, 5806012, 0), Pose2D(628254, 5804525,0), Pose2D(628861, 5802994, 0), Pose2D(628787, 5802587, 0)]
    elif operation_name == "amsterdam_test":
        route_poses = [Pose2D(10, 10, 0), Pose2D(-10, 10,0), Pose2D(-10, -10, 0), Pose2D(10, -10, 0)]

    mac = "mac" 
    token = "token"
    
    if (len(route_poses) <= 1):
        return "Error route too short"
	
    automatic_routing = False
    
    if len(route_poses) <= 2:
	    automatic_routing = True

    # Create actions
    operation1 = OperationPlan(uuid = None, name = operation_name, route = route_poses, actions = None, automatic_routing = automatic_routing)
    operations = [operation1]

    # Create goal
    goal = MissionPlanGoal(mac = mac, token = token, operations = operations)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mission_plan_client_node')
        operation_name = rospy.get_param('~operation_name_param', "coenhaven")

        rospy.sleep(5)

        print("Requesting mission plan to be executed.")
        result = mission_plan_client()
   	
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

    print("finished")
