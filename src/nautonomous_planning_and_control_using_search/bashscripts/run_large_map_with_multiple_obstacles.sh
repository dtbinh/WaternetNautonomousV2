#!/bin/bash

#rosservice call /load "config_name: '/home/daley/WaternetNautonomousV2/src/nautonomous_configuration/config/navigation/map/amsterdam_test.yaml'"  &

sleep 1

rostopic pub --once /mission_coordinator/goal nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: 241.0, y: 13.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

sleep 1
rostopic pub --once /mission_coordinator/obstacles nautonomous_mpc_msgs/Obstacles "obstacles:
- 
  state:
    pose:
      position:
        x: 54.0
        y: -4.0
        z: 3.7
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
  major_semiaxis: 10
  minor_semiaxis: 4
- 
  state:
    pose:
      position:
        x: -122.0
        y: -108.0
        z: 0.5
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
  major_semiaxis: 10
  minor_semiaxis: 4" &

sleep 1

rostopic pub --once /mission_coordinator/start nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -245.0, y: -207.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" 

sleep 1

rostopic pub --once /true_obstacle nautonomous_mpc_msgs/Obstacles "obstacles:
- 
  state:
    pose:
      position:
        x: 54.0
        y: -4.0
        z: 3.7
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
  major_semiaxis: 10
  minor_semiaxis: 4
- 
  state:
    pose:
      position:
        x: -122.0
        y: -108.0
        z: 0.5
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
  major_semiaxis: 10
  minor_semiaxis: 4" &

sleep 5

rostopic pub --once /mission_coordinator/current_state nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -245.0, y: -207.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" 

