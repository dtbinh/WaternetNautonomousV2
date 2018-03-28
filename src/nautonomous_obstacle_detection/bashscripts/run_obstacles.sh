#!/bin/bash

for i in {0..15}
do
a=`echo "0.1 * $i" | bc`
b=`echo "0.0" | bc`
c=`echo "0.0" | bc`
d=`echo "0.1 * $i + 10.0" | bc`

rostopic pub --once /Obstacle_detection/obstacles nautonomous_mpc_msgs/Obstacles "obstacles:
- 
  state:
    pose:
      position:
        x: $a
        y: $b
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0
    twist:
      linear:
        x: 0.0
        y: 0.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 0.0
  major_semiaxis: 1
  minor_semiaxis: 1
- 
  state:
    pose:
      position:
        x: $c
        y: $d
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0
    twist:
      linear:
        x: 0.0
        y: 0.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 0.0
  major_semiaxis: 1
  minor_semiaxis: 1" &

sleep 1
done
