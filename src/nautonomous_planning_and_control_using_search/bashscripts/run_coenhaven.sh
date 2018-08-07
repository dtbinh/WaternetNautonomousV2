#!/bin/bash

rosservice call /load "config_name: '/home/daley/WaternetNautonomousV2/src/nautonomous_configuration/config/navigation/map/amsterdam_cropped_coenhaven.yaml'"  &

sleep 1

rosbag play ~/WaternetNautonomousV2/src/nautonomous_planning_and_control_using_search/Borderfiles/HerengrachtBorder.bag
