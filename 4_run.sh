#!/bin/bash
source devel/setup.bash

roslaunch px4 mavros_posix_tests_offboard_attctl.test gui:=true
