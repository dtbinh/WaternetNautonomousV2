# nautonomous mission coordinator{#nautonomous_mission_coordinator}
Contains a MissionServer (mission_server.cpp) for generating goals and a MoveBaseActionClient (move_base_action_client.cpp) to move to the goals. (?)


operation_manager.cpp <br />
move_base_action_client.cpp <br />
mission_server.cpp

##Nodes
action_operation_manager

##Topics
###Subscribe
/move_base/feedback <br />
/move_base/status <br />
/move_base/result <br />
/mission_action/goal <br />
/mission_action/cancel <br />

###Publish
/mission_action/status <br />
/move_base/goal <br />
/move_base/cancel <br />
/mission_action/result <br />
/mission_action/feedback


## Files
[Src](dir_4427628519c91f4e3c80ab0755fad9e1.html) | [Include](dir_a7ed21d1c4d0824279af0883323bf173.html)

##Overview
![launch_action_client.launch](../images/launch_action_client.png)
<br />
![legenda](../images/legenda.png)


