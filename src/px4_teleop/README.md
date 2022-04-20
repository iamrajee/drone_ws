# px4_teleop
PX4 teleop package

## Nodes
- px4_teleop_com  
  Teleop node with commands
- px4_teleop_joy  
  Teleop node with joystick
- joy_publisher  
  Publishes sensor_msgs::Joy message to /joy_publisher/joy topics

## Launch
- teleop_joy_sim.launch  
  Runs px4_teleop_joy and joy_publisher for simulation
  
## Config
Config files contain axes and buttons mapping
- f710.yaml  
  For Logicool f710 joystick
- f310.yaml  
  For Logicool f310 joystick
- u3312s.yaml  
  For Elecom U3312S joystick

## Running Nodes
px4_teleop_joy node reads a config file written in yaml.  
Run the following command to read specific config file.  
```bash
roslaunch px4_teleop teleop_joy_sim.launch joy_config_path:=<path_to_config>
```

In addition to this, an RC mode is set 1 by default.  
Run the following command to change the RC mode.  
```bash
roslaunch px4_teleop teleop_joy_sim.launch joy_rc_mode:=<rc_mode>
```

## Document
You can build documents by running 
```
doxygen Doxyfile
```
at the root of `px4_teleop` repository.
And the documents will be build in `docs` directory.

The document is also available on https://uenota.github.io/px4_teleop/.
