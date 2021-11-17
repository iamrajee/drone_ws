#!/bin/bash
source devel/setup.bash

#roslaunch px4 mavros_posix_tests_offboard_attctl.test gui:=true
rostopic pub -r 10 /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 
