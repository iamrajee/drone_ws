#!/bin/bash
source devel/setup.bash
rostopic pub -r 10 /mavros/keyboard_command/keyboard_sub std_msgs/Char 97
