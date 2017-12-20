#!/bin/bash

rosservice call /load "config_name: '/home/daley/WaternetNautonomousV2/src/nautonomous_planning_and_control_neural_network_training/include/amsterdam_cropped.yaml'"  &

sleep 1 &

rosrun nautonomous_planning_and_control_neural_network_training opt_tree_path_planner

