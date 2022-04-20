# warehouse-quad
Program for a warehouse maintainance (Tilt) Quadrotor.

# Installation

Installtion itself is largely trivial, you just need to do clone this repo inside the source of you catkin workspace and build.
Steps might look like : 
* `cd ~/catkin_ws/src`
* `git clone http://github.com/harshsinh/warehouse-quad.git`
* `cd ~/catkin_ws`
* `catkin_make`

Make sure have the following dependencies satisfied.

## Dependencies
* [ROS](www.ros.org) : Robot Operating System  
For Installing ROS refer [ROS Installation Guide](http://wiki.ros.org/ROS/Installation). Preferably install `ROS-desktop-full`, otherwise make sure all relevant packages are installed.
* [Mavros](http://wiki.ros.org/mavros) : ROS communication driver for various autopilots with MAVLink communication protocol.  
Run `sudo apt-get install ros-kinetic-mavros` and `sudo apt-get install ros-kinetic-mavros-extras`
* [px4flow_node](http://wiki.ros.org/px4flow_node) : This package parses the MAVLINK messages from the PX4Flow optical flow board, and converts them to ROS messages before publishing them.  
Refer to the guide on the link for installing this.
* [Zbar](https://github.com/ZBar/ZBar) : An open source Bar Code and QR Code Reader.  
Run `sudo apt-get install libzbar-dev` from commandline or install from source following instructions from the above link.   
* [Eigen3](http://eigen.tuxfamily.org/dox/) : Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.  
Run `sudo apt-get install libeigen3-dev` from command line or to install from source use the instructions from the [link](https://bitbucket.org/eigen/eigen/src/034b6c3e101792a3cc3ccabd9bfaddcabe85bb58/INSTALL?at=default&fileviewer=file-view-default)
* [CV Bridge](http://wiki.ros.org/cv_bridge) : converts between ROS Image messages and OpenCV images.  
Run `sudo apt-get install ros-kinetic-cv-bridge`
* [Image transport](http://wiki.ros.org/image_transport) : For subscribing and publishing images as ROS message.  
Run `sudo apt-get install ros-kinetic-image-transport`

**Note**  We implemented this system on ROS Kinetic, so all the instructions are for installing ROS Kinetic packages, though feel free to replace the `-kinetic` with whatever your ROS distro might be.

#### Please refer to the contributions guide [`CONTRIBUTING.md`](https://github.com/harshsinh/warehouse-quad/blob/master/CONTRIBUTING.md) prior to making any pull request.
