/**
 * @file px4_teleop_joy.cpp
 * @brief Node for controlling px4 based drone by joystick
 * @author Takaki Ueno
 */

// C++ libraries
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// roscpp
#include <ros/package.h>
#include <ros/ros.h>

// sensor_msgs
#include <sensor_msgs/Joy.h>

// geometry_msgs
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>

// mavros_msgs
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// header from px4_teleop_cmds
#include <px4_teleop_cmds.hpp>

// yaml-cpp
#include <yaml-cpp/yaml.h>

//! Rc mode constant
const int RC_MODE_ONE = 1;
//! Rc mode constant
const int RC_MODE_TWO = 2;

//! Storage for path of this package
const std::string px4_teleop_path = ros::package::getPath("px4_teleop");

//! Storage for vehicle state
mavros_msgs::State current_state;

/**
 * @brief Callback function for state subscriber
 * @param msg Incoming message
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

//! Storage for current global position
sensor_msgs::NavSatFix curr_gpos;

/**
 * @brief Callback function for global position subscriber
 * @param msg Incoming message
 */
void curr_gpos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  curr_gpos = *msg;
}

//! Storage for local position
geometry_msgs::PoseStamped local_pos;

/**
 * @brief Callback function for local position subscriber
 * @param msg Incoming message
 */
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  local_pos = *msg;
}

//! Storage for joystick input
sensor_msgs::Joy joy_msg;
/**
 * @brief Callback function for joystick input subscriber
 * @param msg Incoming message
 */
void joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
{
  joy_msg = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "px4_teleop_joy");
  ros::NodeHandle nh("~");

  ros::Rate rate(20);

  // Publisher
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);

  // Subscriber
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 100, joy_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, state_cb);
  ros::Subscriber curr_gpos_sub =
      nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/pose", 10, curr_gpos_cb);
  ros::Subscriber local_pos_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, local_pos_cb);

  // Survice Client
  ros::ServiceClient set_hp_client = nh.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  // Config file path
  std::string joy_config_path;
  nh.param<std::string>("joy_config_path", joy_config_path, px4_teleop_path + "/config/f710.yaml");

  // Binding file path
  std::string joy_binding_path;
  nh.param<std::string>("joy_binding_path", joy_binding_path, px4_teleop_path + "/config/f310_binding.yaml");

  // Rc mode
  int joy_rc_mode;
  nh.param<int>("joy_rc_mode", joy_rc_mode, 1);

  // Takeoff height
  float takeoff_height;
  nh.param<float>("takeoff_height", takeoff_height, 2.0);

  // MAV_FRAME
  std::string mav_frame;
  nh.param<std::string>("/mavros/setpoint_velocity/mav_frame", mav_frame, "LOCAL_NED");

  ROS_INFO("RC Mode: %d", joy_rc_mode);
  ROS_INFO("Config: %s", joy_config_path.c_str());
  ROS_INFO("MAV_FRAME: %s", mav_frame.c_str());

  // Read config file
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(joy_config_path);
  }
  catch (YAML::Exception& e)
  {
    ROS_ERROR("Failed to read config file");
    return -1;
  }

  // Read config file
  YAML::Node binding;
  try
  {
    binding = YAML::LoadFile(joy_binding_path);
  }
  catch (YAML::Exception& e)
  {
    ROS_ERROR("Failed to read binding file");
    return -1;
  }

  // Read setting
  switch (joy_rc_mode)
  {
    case RC_MODE_ONE:
      config = config["mode1"];
      break;
    case RC_MODE_TWO:
      config = config["mode2"];
      break;
  }

  // Wait for connection
  while (ros::ok() and current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // Wait for /mavros/global_position/global
  ROS_INFO("Waiting for message from /mavros/global_position/global");
  const std::string topic = "/mavros/global_position/global";
  sensor_msgs::NavSatFix init_gpos = *ros::topic::waitForMessage<sensor_msgs::NavSatFix>(topic);
  double init_latitude = init_gpos.latitude;
  double init_longitude = init_gpos.longitude;
  double init_altitude = init_gpos.altitude;
  ROS_INFO("Initial Lat:%f Lon:%f Alt:%f", init_latitude, init_longitude, init_altitude);

  // set home position as current position
  mavros_msgs::CommandHome set_hp_cmd;
  set_hp_cmd.request.current_gps = true;
  while (not(set_hp_client.call(set_hp_cmd)) and set_hp_cmd.response.success)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("HP set.");

  // set mode as offboard
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  while (not(set_mode_client.call(offb_set_mode)))
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Offboard enabled.");

  ros::Time last_request = ros::Time::now();
  ros::Time button_last_event = ros::Time::now();

  // print button binding
  printf("\n\n");
  ROS_INFO("===== BUTTON BINDING =====");
  ROS_INFO("ARM     \t: %s",
           binding["buttons"][config["button_map"]["arm"].as<std::string>()].as<std::string>().c_str());
  ROS_INFO("TAKEOFF \t: %s",
           binding["buttons"][config["button_map"]["takeoff"].as<std::string>()].as<std::string>().c_str());
  ROS_INFO("LAND    \t: %s",
           binding["buttons"][config["button_map"]["land"].as<std::string>()].as<std::string>().c_str());
  ROS_INFO("DISARM  \t: %s",
           binding["buttons"][config["button_map"]["disarm"].as<std::string>()].as<std::string>().c_str());
  ROS_INFO("DEADMAN \t: %s",
           binding["buttons"][config["button_map"]["deadman"].as<std::string>()].as<std::string>().c_str());

  // print axes binding
  ROS_INFO("===== AXES BINDING =====");
  ROS_INFO("PITCH   \t: %s", binding["axes"][config["axes_map"]["pitch"].as<std::string>()].as<std::string>().c_str());
  ROS_INFO("YAW     \t: %s", binding["axes"][config["axes_map"]["yaw"].as<std::string>()].as<std::string>().c_str());
  ROS_INFO("ROLL    \t: %s", binding["axes"][config["axes_map"]["roll"].as<std::string>()].as<std::string>().c_str());
  ROS_INFO("THROTTLE\t: %s",
           binding["axes"][config["axes_map"]["throttle"].as<std::string>()].as<std::string>().c_str());

  // print axes scaling
  ROS_INFO("===== AXES SCALING =====");
  ROS_INFO("PITCH   \t: %f", config["axes_scale"]["pitch"].as<double>());
  ROS_INFO("YAW     \t: %f", config["axes_scale"]["yaw"].as<double>());
  ROS_INFO("ROLL    \t: %f", config["axes_scale"]["roll"].as<double>());
  ROS_INFO("THROTTLE\t: %f", config["axes_scale"]["throttle"].as<double>());
  printf("\n\n");

  // Initialize variables related to joy
  std::vector<float> joy_axes;
  std::vector<int32_t> joy_button;

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    joy_axes = joy_msg.axes;
    joy_button = joy_msg.buttons;

    if (current_state.mode != "OFFBOARD" and (ros::Time::now() - last_request > ros::Duration(0.1)))
    {
      if ((set_mode_client.call(offb_set_mode)))
      {
        ROS_DEBUG("Offboard enabled.");
      }
      last_request = ros::Time::now();
    }

    if(joy_axes.size()==0 or joy_button.size()==0)
    {
      continue;
    }

    geometry_msgs::TwistStamped cmd_vel_msg;
    try
    {

      double lin_x = config["axes_scale"]["pitch"].as<double>() * joy_axes.at(config["axes_map"]["pitch"].as<int>());
      double lin_y = config["axes_scale"]["roll"].as<double>() * joy_axes.at(config["axes_map"]["roll"].as<int>());
      double lin_z =
          config["axes_scale"]["throttle"].as<double>() * joy_axes.at(config["axes_map"]["throttle"].as<int>());
      double ang_z = config["axes_scale"]["yaw"].as<double>() * joy_axes.at(config["axes_map"]["yaw"].as<int>());

      cmd_vel_msg.twist.linear.x = lin_x;
      cmd_vel_msg.twist.linear.y = lin_y;
      cmd_vel_msg.twist.linear.z = lin_z;
      cmd_vel_msg.twist.angular.z = ang_z;

      cmd_vel_pub.publish(cmd_vel_msg);

      if (ros::Time::now() - button_last_event > ros::Duration(0.1))
      {
        if (joy_button.at(config["button_map"]["deadman"].as<int>()))
        {
          if (joy_button.at(config["button_map"]["arm"].as<int>()))
          {
            arm(arming_client, current_state);
          }
          else if (joy_button.at(config["button_map"]["disarm"].as<int>()))
          {
            disarm(arming_client);
          }
          else if (joy_button.at(config["button_map"]["takeoff"].as<int>()))
          {
            takeoff(takeoff_client, current_state, local_pos, init_gpos, takeoff_height);
          }
          else if (joy_button.at(config["button_map"]["land"].as<int>()))
          {
            land(landing_client, local_pos, curr_gpos, init_gpos);
          }
          else if (joy_button.at(config["button_map"]["exit"].as<int>()))
          {
            break;
          }
          button_last_event = ros::Time::now();
        }
      }
    }
    catch (std::out_of_range& e)
    {
      ROS_ERROR("%s", e.what());
      continue;
    }
  }

  ROS_INFO("Exitting px4_teleop.");
  land(landing_client, local_pos, curr_gpos, init_gpos);

  return 0;
}
