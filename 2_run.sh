#!/bin/bash
#Terminal2

source devel/setup.bash
#roslaunch px4 posix_sitl.launch
roslaunch px4 posix_sitl.launch world:=/home/rajendra/drone_ws/src/PX4-Autopilot/Tools/sitl_gazebo/worlds/warehouse.world
