#!/bin/bash

rostopic pub --once /Map_modifier/borders nautonomous_mpc_msgs/Obstacles "obstacles: 
- 
  pose: 
    position: 
      x: -14.4784637742
      y: -31.7465229278
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.706485152732
      w: 0.707727863638
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  major_semiaxis: 144.754609854
  minor_semiaxis: 4.72538863936
  ns: 0
- 
  pose: 
    position: 
      x: -96.2222476109
      y: -10.7484550209
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.726492968019
      w: 0.687173898965
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  major_semiaxis: 122.672814035
  minor_semiaxis: 4.50732556223
  ns: 0
- 
  pose: 
    position: 
      x: 65.0269221568
      y: 114.582927236
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.999796625119
      w: 0.0201670126754
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  major_semiaxis: 75.5396049418
  minor_semiaxis: 1.45417791789
  ns: 0
- 
  pose: 
    position: 
      x: -63.9229926174
      y: -157.84927006
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.928965201608
      w: 0.370167062557
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  major_semiaxis: 33.3190551536
  minor_semiaxis: 4.87317265853
  ns: 0
- 
  pose: 
    position: 
      x: -123.739829355
      y: 108.924446601
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.997762664836
      w: 0.0668555507047
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  major_semiaxis: 17.3315023051
  minor_semiaxis: 1.35541389349
  ns: 0
" &
