/**
* @file joy_publisher.cpp
* @brief Read input of joystick and publish it
* @author Takaki Ueno
*/

// C/C++ Libraries
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <string>
#include <stdexcept>

#include <limits.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

// roscpp
#include <ros/ros.h>
#include <ros/package.h>

// sensor_msgs
#include <sensor_msgs/Joy.h>


int main(int argc, char **argv){

  ros::init(argc, argv, "joy_publisher");
  ros::NodeHandle nh("~");

  ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("joy", 100);

  ros::Rate rate(100);

  // Declare joystick file descriptor
  std::string joy_dev;
  nh.param<std::string>("joy_dev", joy_dev, "/dev/input/js0");

  ROS_INFO("Device: %s", joy_dev.c_str());

  // Set variables about joystick
  int joy_fd = -1;
  int num_of_axes = 0;
  int num_of_buttons = 0;
  char name_of_joystick[80];
  std::vector<int32_t> joy_button;
  std::vector<float> joy_axes;

  // Open joy_dev
  if((joy_fd = open(joy_dev.c_str(), O_RDONLY)) < 0){
    ROS_ERROR("Failed to open %s", joy_dev.c_str());
    return -1;
  }

  // Get info about joystick
  ioctl(joy_fd, JSIOCGAXES, &num_of_axes);
  ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
  ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

  // Resize vector
  joy_button.resize(num_of_buttons,0);
  joy_axes.resize(num_of_axes,0);

  ROS_INFO("Joystick: %s", name_of_joystick);
  ROS_INFO("Axis: %d", num_of_axes);
  ROS_INFO("Buttons: %d", num_of_buttons);

  // Use non-blocking mode
  fcntl(joy_fd, F_SETFL, O_NONBLOCK);

  while(ros::ok()){

    // https://www.kernel.org/doc/Documentation/input/joystick-api.txt
    //struct js_event{
    //  __u32 time;   /* event timestamp in milliseconds */
    //  __s16 value;  /* value */
    //  __u8 type;    /* event type */
    //  __u8 number;  /* axis/button number */
    //}
    js_event js;

    sensor_msgs::Joy joy_msg;

    auto _ = read(joy_fd, &js, sizeof(js_event));

    // the driver will issue synthetic JS_EVENT_INIT on open
    // js.type will be like following if it is issuing INIT BUTTON event
    //     js.type = JS_EVENT_BUTTON | JS_EVENT_INIT
    // So, js.type & ~JS_EVENT_INIT equals
    // JS_EVENT_BUTTON or JS_EVENT_AXIS
    switch(js.type & ~JS_EVENT_INIT){
      case JS_EVENT_AXIS:
        try{
          joy_axes.at((int)js.number) = (float)(js.value)/SHRT_MAX;
        }catch(std::out_of_range& e){
          break;
        }
        break;

      case JS_EVENT_BUTTON:
        try{
          joy_button.at((int)js.number) = js.value;
        }catch(std::out_of_range& e){
          break;
        }
        break;
    }

    joy_msg.axes = joy_axes;
    joy_msg.buttons = joy_button;

    joy_pub.publish(joy_msg);

    ros::spinOnce();
    rate.sleep();
  }

  close(joy_fd);
  return 0;
}