/**
* @file px4_teleop_com.cpp
* @brief Node for controlling px4 based drone by command
* @author Takaki Ueno
*/

// C/C++ libraries
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <csignal>

// roscpp
#include <ros/ros.h>

// geometry_msgs
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

// sensor_msgs
#include <sensor_msgs/NavSatFix.h>

// mavros_msgs
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>

// Header from px4_teleop
#include <px4_teleop_cmds.hpp>

// Set signal handler
// std::signal() returns a pointer to the handler function
// that was in charge of handling this signal before the call of std::signal()
 //! Pointer to old handler function
void (*old)(int);


//! Storage for vehicle state
mavros_msgs::State current_state;

/**
* @brief Callback function for state subscriber
* @param msg Incoming message
*/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


//! Storage for current global position
sensor_msgs::NavSatFix curr_gpos;

/**
* @brief Callback function for global position subscriber
* @param msg Incoming message
*/
void curr_gpos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    curr_gpos = *msg;
}


//! Storage for local position
geometry_msgs::PoseStamped local_pos;
/**
* @brief Callback function for local position subscriber
* @param msg Incoming message
*/
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

/**
* @brief Prints a list of available commands
*/
void printUsage(){
    std::cout << "Available Commands:" << std::endl;
    std::cout << "\tarm" << std::endl;
    std::cout << "\tdisarm" << std::endl;
    std::cout << "\ttakeoff" << std::endl;
    std::cout << "\tland" << std::endl;
    std::cout << "\tcmd_vel" << std::endl;
    std::cout << "\thelp" << std::endl;
    std::cout << "\tquit" << std::endl;
}

/**
* @brief Handles interrupt signal from kyeboard
* @param sig Input signal
*/
void interrupt_handler(int sig){
    std::signal(sig, old);
    exit(0);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "px4_teleop_com");
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 100, state_cb);
    ros::Subscriber curr_gpos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/pose", 10, curr_gpos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 100, local_pos_cb);

    // Publisher
    ros::Publisher target_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 100);

    // Survice Client
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
               ("mavros/cmd/arming");
    ros::ServiceClient set_hp_client = nh.serviceClient<mavros_msgs::CommandHome>
               ("mavros/cmd/set_home");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
               ("mavros/cmd/takeoff");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>
               ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
               ("mavros/set_mode");

    ros::Rate rate(20);

    // Command string and vector of commands
    std::string command, buffer;
    std::vector<std::string> command_args;

    // Set signal handler
    // std::signal() returns a pointer to the handler function
    // that was in charge of handling this signal before the call of std::signal()
    old = std::signal(SIGINT, interrupt_handler);

    // Wait for connection
    while(ros::ok() and current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Wait for /mavros/global_position/global
    ROS_INFO("Waiting for message from /mavros/global_position/global");
    const std::string topic = "mavros/global_position/global";
    sensor_msgs::NavSatFix init_gpos = *ros::topic::waitForMessage<sensor_msgs::NavSatFix>(topic);
    double init_latitude = init_gpos.latitude;
    double init_longitude = init_gpos.longitude;
    double init_altitude = init_gpos.altitude;
    ROS_INFO("Initial Lat:%f Lon:%f Alt:%f", init_latitude, init_longitude, init_altitude);

    // set home position as current position
    mavros_msgs::CommandHome set_hp_cmd;
    set_hp_cmd.request.current_gps = true;
    while( not(set_hp_client.call(set_hp_cmd)) and
               set_hp_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("HP set.");

    // set mode as offboard
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    while( not(set_mode_client.call(offb_set_mode))){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Offboard enabled.");


    while(ros::ok()){

        // Wait for input
        std::cout << "com> ";
        std::getline(std::cin, command);
        if(std::cin.fail() or std::cin.eof()){
            std::cin.clear();
        }

        std::stringstream ss(command);

        command_args.clear();
        command_args.shrink_to_fit();

        while(not ss.eof()){
            ss >> buffer;
            command_args.push_back(buffer);
        }

        if(command_args[0]=="arm" and command_args.size()==1)
        {
            arm(arming_client, current_state);
        }
        else if(command_args[0]=="takeoff" and command_args.size()<3)
        {
            try{
                takeoff(takeoff_client, current_state, 
                        local_pos, init_gpos,
                        std::stod(command_args.at(1)));
            }catch(...){
                std::cout << "Invalid argument." << std::endl;
                std::cout << "Usage:\n\ttakeoff height" << std::endl;
            }
        }
        else if(command_args[0]=="land" and command_args.size()==1)
        {
            land(landing_client, local_pos, curr_gpos, init_gpos);
        }
        else if(command_args[0]=="disarm" and command_args.size()==1)
        {
            disarm(arming_client);
        }
        else if(command_args[0]=="help" and command_args.size()==1)
        {
            printUsage();
        }
        else if(command_args[0]=="quit" and command_args.size()==1)
        {
            exit(0);
        }
        else if(command_args[0]=="cmd_vel" and command_args.size()<8)
        {
            try{
                cmd_vel(target_vel_pub, set_mode_client,
                        current_state,
                        std::stod(command_args.at(1)),
                        std::stod(command_args.at(2)),
                        std::stod(command_args.at(3)),
                        std::stod(command_args.at(4)),
                        std::stod(command_args.at(5)));
            }catch(...){
                std::cout << "Invalid argument." << std::endl;
                std::cout << "Usage:\n\tcmd_vel dt vx vy vz ang_z" << std::endl;
            }
        }
        else
        {
            std::cout << "Invalid command." << std::endl;
            std::cout << "Type \"help\" to print available commands." << std::endl;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // Restore initial handler
    std::signal(SIGINT, old);
    return 0;
}
