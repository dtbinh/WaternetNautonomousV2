#!/bin/bash

rosservice call /load "config_name: '/home/daley/WaternetNautonomousV2/src/nautonomous_configuration/config/navigation/map/amsterdam_cropped_probeersel.yaml'"  &

sleep 1 &

rosrun nautonomous_planning_and_control_using_search tree_path_planner &

rosrun nautonomous_planning_and_control_using_search opt_tree_path_planner_2 &

rostopic pub /mission_coordinator/goal nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: 115.0, y: 25.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

sleep 1
rostopic pub /mission_coordinator/start nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -112.0, y: 18.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" 
