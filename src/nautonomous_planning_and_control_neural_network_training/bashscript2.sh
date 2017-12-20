#!/bin/bash
for i in {0..1000}
do
echo "Start simulation " $i 

rostopic pub --once /mission_coordinator/goal nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: 125.0, y: 0.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" &

sleep 0.1
rostopic pub --once /mission_coordinator/obstacle nautonomous_mpc_msgs/Obstacle "state:
  pose:
    position:
      x: $(( ( RANDOM % 50 )  - 25  ))
      y: $(( ( RANDOM % 10 )  - 5 ))
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
major_semiaxis: $(( ( RANDOM % 6 )  + 3 ))
minor_semiaxis: $(( ( RANDOM % 3 )  + 3 ))" &

sleep 0.1
rostopic pub --once /mission_coordinator/start nautonomous_mpc_msgs/StageVariable "{T_l: 0.0, T_r: 0.0, x: -125.0, y: 0.0, theta: 0.0, u: 0.0, v: 0.0, omega: 0.0}" 

sleep 1
done
