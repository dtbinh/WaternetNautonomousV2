#!/bin/bash

rosservice call /load "config_name: '/home/daley/WaternetNautonomousV2/src/nautonomous_configuration/config/navigation/map/amsterdam_test.yaml'"  &

sleep 1 &

rosrun nautonomous_planning_and_control_using_search grid_planner_2 &

#rostopic pub /mission_coordinator/goal nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: 241.0, y: 13.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

rostopic pub /mission_coordinator/goal nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -80.31, y: 236.849, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

sleep 1
rostopic pub /mission_coordinator/start nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -245.0, y: -207.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" 
