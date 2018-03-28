#!/bin/bash

rostopic pub --once /mission_coordinator/obstacles nautonomous_mpc_msgs/Obstacles "obstacles: 
- 
  state: 
    pose: 
      position: 
        x: -10.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    twist: 
      linear: 
        x: 0.0
        y: 0.0
        z: 0.0
      angular: 
        x: 0.0
        y: 0.0
        z: 0.0
  major_semiaxis: 10
  minor_semiaxis: 2
- 
  state: 
    pose: 
      position: 
        x: -20.0
        y: -1
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    twist: 
      linear: 
        x: 0.5
        y: 0.0
        z: 0.0
      angular: 
        x: 0.0
        y: 0.0
        z: 0.0
  major_semiaxis: 15
  minor_semiaxis: 1
- 
  state: 
    pose: 
      position: 
        x: 25.0
        y: 5.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: -0.989645932961
        w: 0.14353023157
    twist: 
      linear: 
        x: 0.0
        y: 0.0
        z: 0.0
      angular: 
        x: 0.0
        y: 0.0
        z: 0.0
  major_semiaxis: 5
  minor_semiaxis: 5
- 
  state: 
    pose: 
      position: 
        x: 0.0
        y: -5
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: -0.986496285356
        w: 0.16378363465
    twist: 
      linear: 
        x: 0.0
        y: 0.0
        z: 0.0
      angular: 
        x: 0.0
        y: 0.0
        z: 0.0
  major_semiaxis: 9
  minor_semiaxis: 9
- 
  state: 
    pose: 
      position: 
        x: -10
        y: -10
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.996592677258
        w: 0.0824805167001
    twist: 
      linear: 
        x: 0.0
        y: 0.0
        z: 0.0
      angular: 
        x: 0.0
        y: 0.0
        z: 0.0
  major_semiaxis: 4
  minor_semiaxis: 2" &

