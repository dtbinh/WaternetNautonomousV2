#!/bin/bash

rosservice call /load "config_name: '/home/daley/WaternetNautonomousV2/src/nautonomous_configuration/config/navigation/map/amsterdam_cropped_Probeersel.yaml'"  &

sleep 1 &
rostopic pub --once /mission_coordinator/goal nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: 25.0, y: 23.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

sleep 1

rostopic pub --once /mission_coordinator/start nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -26.0, y: -27.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &




