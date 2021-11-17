#!/bin/bash
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r
rosdep update
