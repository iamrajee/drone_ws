================ Mavros debuging ===============
e.g unable to take off
rosrun mavros checkid
rostopic echo /mavros/state
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /diagnostics
# ERROR: Unknown model iris_depth_camera (not found by name on
cd workspace/src
cp Fast-Planner/storage/1024_iris_depth_camera ~/.ros/etc/init.d-posix/airframes/1024_iris_depth_camera
??mzahana sudo apt install ros-noetic-multi-map-server
# =========== Install ===========
### ERROR: missing geometric_controller or mavros_controllers
git clone https://github.com/Jaeyoung-Lim/mavros_controllers.git
### ??? 
https://github.com/HKUST-Aerial-Robotics/mockasimulator

# ============= Normal fast-planner ==============
### T1
source devel/setup.bash && roslaunch plan_manage rviz.launch

### T2
source devel/setup.bash && roslaunch plan_manage kino_replan.launch


# =============== Beomsu7 =================
https://github.com/beomsu7/Fast-Planner

### T1
roslaunch plan_manage px4_sitl_kino_replan.launch

### T2
rosrun mavros mavsafety arm

### T3
rosrun mavros mavsys mode -c OFFBOARD

# ================ Mzahana =================
https://github.com/mzahana/px4_fast_planner

### T1
roslaunch px4_fast_planner px4_fast_planner.launch

### T2
rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 19.0
    y: 15.0
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

# ============ Fast Planner ERROR ===========
Triggered!
[TRIG]: from WAIT_TARGET to GEN_NEW_TRAJ
[kino replan]: -----------------------
start:  -0.0112145 -0.00287183 -0.00545828, -0.00900632 -0.00325972  -0.0105637, 0 0 0
goal:0 0 1, 0 0 0
[fast_planner_node-2] process has died [pid 99980, exit code -11, cmd /home/rajendra/drone_ws/devel/lib/plan_manage/fast_planner_node /odom_world:=/mavros/local_position/odom /sdf_map/odom:=/mavros/local_position/odom /sdf_map/cloud:=/pcl_render_node/cloud /sdf_map/pose:=/mavros/local_position/pose /sdf_map/depth:=/d435/depth/image_rect_raw __name:=fast_planner_node __log:=/home/rajendra/.ros/log/cf8b4932-4483-11ec-a2f0-cf1b4db027f8/fast_planner_node-2.log].
log file: /home/rajendra/.ros/log/cf8b4932-4483-11ec-a2f0-cf1b4db027f8/fast_planner_node-2*.log


# =================== Vins-fusion ============
```https://github.com/HKUST-Aerial-Robotics/VINS-Fusion```

### 1. Clone
cd ~/drone_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
cd ../

### 2. Download and Install ceres-solver
sudo apt-get install cmake
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev

http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
cp ~/Downloads/ceres-solver-2.0.0.tar.gz .
tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.0.0
make -j3
make test
make install

### 3. Build 
catkin_make

### 4. Errors

#### error: ‘integer_sequence’ is not a member of ‘std’
https://github.com/colmap/colmap/issues/905
https://programmerah.com/solved-ceres-compile-error-integer_sequence-is-not-a-member-of-std-31728/
https://stackoverflow.com/questions/17424477/implementation-c14-make-integer-sequence
My ErrorLog.txt => https://gist.github.com/iamrajee/6e43d88e65214f2fa96a30d3dcb365bb
Solution (c++11 => c++14) in:
cd ~/drone_ws/src/VINS-Fusion
code camera_models/CMakeLists.txt
code global_fusion/CMakeLists.txt
code loop_fusion/CMakeLists.txt
code vins_estimator/CMakeLists.txt

rosrun vins vins_node ~/drone_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
=> Segmentation Fault
https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/issues/134
https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/issues/106#issuecomment-864415677
https://github.com/mrdavoodi/VINS-Fusion/commits/master

cd ~/drone_ws/src/VINS-Fusion

code camera_models/include/camodocal/calib/CameraCalibration.h
https://github.com/mrdavoodi/VINS-Fusion/commit/0e50050be32a8a5292b862fa86a605e3adf6761e

code camera_models/include/camodocal/camera_models/Camera.h
https://github.com/mrdavoodi/VINS-Fusion/commit/fc3f3ccd1786dec1fe73baeb2900a8992ee3cc65

code camera_models/include/camodocal/chessboard/Chessboard.h
https://github.com/mrdavoodi/VINS-Fusion/commit/0461b57850d5b5393b9476950f883178670a71b9

code loop_fusion/src/ThirdParty/DVision/BRIEF.h
https://github.com/mrdavoodi/VINS-Fusion/commit/37144c454117092baac22818caf9c8e33c7e66f0

### ERROR: fatal error: opencv2/imgcodecs/legacy/constants_c.h: no such file or directory ros
Because using old version of opencv
https://github.com/opencv/opencv/issues/13201
https://github.com/opencv/opencv/pull/13253/commits/562676c5dae4d25b00f88556bd8de99330d4e484
https://github.com/raulmur/ORB_SLAM2/issues/782
https://answers.ros.org/question/225229/error-opencv2imgcodecshpp/

#### Solution 1. Tried didn't work
cd /usr/include/opencv/modules/imgcodecs/include/opencv2/imgcodecs
sudo cp -r /usr/include/opencv4/opencv2/imgcodecs/legacy .
(But have to build again so aborded => mv legacy/ .legacy/)

#### Solution 2. Changing CV bridge opencv version older(3 or opencv) to newer(4)
cd /opt/ros/noetic/share/cv_bridge/cmake
code cv_bridgeConfig.cmake
=> din't work

#### Simple =>? removing that inclusion (#include "opencv2/imgcodecs/legacy/constancts_c.h").
Doesn't either Works?

### fatal error: development/mavlink.h: No such file or directory
https://github.com/PX4/PX4-SITL_gazebo/issues/823
cd ~/px4_ws/src/PX4-SITL_gazebo
git checkout 6fa6ec78a7a1619b7b90ac0c01aaec919defbe35

?? catkin_make
[100%] Linking CXX executable /home/rajendra/drone_ws/devel/lib/vins/vins_node
/usr/bin/ld: warning: libopencv_video.so.4.2, needed by /home/rajendra/drone_ws/devel/lib/libvins_lib.so, may conflict with libopencv_video.so.3.4
[100%] Built target vins_node

# rtab drone
cd drone_ws/src
https://github.com/matlabbe/rtabmap_drone_example
git clone https://github.com/matlabbe/rtabmap_drone_example.git
git clone https://github.com/SyrianSpock/realsense_gazebo_plugin.git
cd ../
./rosdep
sudo apt install ros-noetic-teleop-twist-joy
sudo apt install ros-noetic-imu-complementary-filter
catkin_make

roslaunch rtabmap_drone_example gazebo.launch
roslaunch rtabmap_drone_example rviz.launch
roslaunch rtabmap_drone_example slam.launch
rosrun rtabmap_drone_example offboard

??
rosrun teleop_twist_keyboard teleop_twist_keyboard.py


========= malithana rrt
roscore
rosrun rrt-planning rrt
rosrun rviz rviz #add mark from available topic



# ======================== Ardupilot ========================

# Linux
Installation: https://ardupilot.org/dev/docs/building-setup-linux.html
Setting up SITL on Linux: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html


# built
cd ~/drone_ws/ardupilot
./waf configure --board sitl

# configure
cd ~/drone_ws/ardupilot/ArduCopter
sim_vehicle.py -w (once)

# run
sim_vehicle.py --console --map

/home/rajendra/drone_ws/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --no-extra-ports -I 0 --out=udp:54.161.56.169:14550

/home/rajendra/drone_ws/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --no-extra-ports -I 0 --out=udp:54.161.56.169:14550


# ======================== HKUST/FUEL ========================
#error PCL requires C++14 or above 7 | #error PCL requires C++14 or above
=> Change to c++11 to c++14  
=> https://www.google.com/search?q=%23error+PCL+requires+C%2B%2B14+or+above+7+%7C+%23error+PCL+requires+C%2B%2B14+or+above&oq=%23error+PCL+requires+C%2B%2B14+or+above+7+%7C+++%23error+PCL+requires+C%2B%2B14+or+above&aqs=chrome..69i57.316j0j7&sourceid=chrome&ie=UTF-8  
https://github.com/PointCloudLibrary/pcl/issues/2968  
https://programmerah.com/error-error-pcl-requires-c14-or-above-44572/  



# ====================== FUEL ===================
```https://github.com/HKUST-Aerial-Robotics/FUEL```
# T1
source devel/setup.bash && roslaunch exploration_manager rviz.launch

# T2
source devel/setup.bash && roslaunch exploration_manager exploration.launch


