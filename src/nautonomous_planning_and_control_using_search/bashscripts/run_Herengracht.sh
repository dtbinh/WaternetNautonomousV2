#!/bin/bash

rosservice call /load "config_name: '/home/daley/WaternetNautonomousV2/src/nautonomous_configuration/config/navigation/map/amsterdam_cropped_Herengracht_eindstuk.yaml'"  &

#sleep 1 &

#rosrun nautonomous_planning_and_control_using_search grid_planner_2 &

sleep 1

rostopic pub --once /mission_coordinator/goal_state nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: 109.6, y: -221.3, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

#rostopic pub --once /mission_coordinator/start nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -70.0, y: 135.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

sleep 1

#rostopic pub --once /mission_coordinator/start nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -70.0, y: 135.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

rostopic pub --once /mission_coordinator/start_state nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -48.6, y: 199.0, theta: 1.5, u: 0.0, v: 0.0, omega: 0.0}" 

