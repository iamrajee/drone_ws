#!/bin/bash
cd drone_ws/src
git clone https://github.com/mavlink/mavros.git
git clone https://github.com/PX4/PX4-Autopilot.git
DONT_RUN=1 make px4_sitl_default gazebo
git clone --recursive https://github.com/PX4/PX4-SITL_gazebo.git
sudo bash mavros/mavros/scripts/install_geographiclib_datasets.sh
cd  mavros
git checkout master
pip install pymavlink
cd ../../
pip3 install kconfiglib
catkin_make
./rosdep
catkin_make
