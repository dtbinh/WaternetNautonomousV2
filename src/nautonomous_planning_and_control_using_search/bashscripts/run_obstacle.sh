#!/bin/bash

rosservice call /load "config_name: '/home/daley/WaternetNautonomousV2/src/nautonomous_configuration/config/navigation/map/amsterdam_cropped_Probeersel.yaml'"  &

sleep 1

rostopic pub --once /mission_coordinator/goal nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: 25.0, y: 0.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &


sleep 1
rostopic pub --once /mission_coordinator/obstacle nautonomous_mpc_msgs/Obstacle "state:
  pose:
    position:
      x: 20
      y: 0.0
      z: 3.14
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.5
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
major_semiaxis: 5
minor_semiaxis: 5" &

sleep 1

rostopic pub --once /mission_coordinator/start nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -25.0, y: 0.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" 

sleep 2

rostopic pub --once /mission_coordinator/current_state nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -25.0, y: 0.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" 

