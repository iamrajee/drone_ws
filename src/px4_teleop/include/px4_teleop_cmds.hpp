/**
* @file px4_teleop_cmds.hpp
* @brief Command functions for px4_teleop package
* @author Takaki Ueno
*/

#ifndef INCLUDED_px4_teleop_cmds_hpp_
#define INCLUDED_px4_teleop_cmds_hpp_

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


/**
* @brief Arm vehicle
* @param client Service client for arm
* @param state State of vehicle
*/
void arm(ros::ServiceClient& client, mavros_msgs::State& state){

    ros::Rate rate(20);

    // reject arming if already armed
    if(state.armed){
        ROS_WARN("Arm Rejected. Already armed.");
        return;
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // call arm service
    while( not(client.call(arm_cmd)) and
                arm_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed.");
    return;
}

/**
* @brief Take off vehicle
* @param client Service client for takeoff
* @param state State of vehicle
* @param lpos Current local position
* @param hp Home position
* @param height Takeoff height
*/
void takeoff(ros::ServiceClient& client,
                mavros_msgs::State& state,
                geometry_msgs::PoseStamped& lpos,
                sensor_msgs::NavSatFix& hp,
                double height){

    ros::Rate rate(20);

    if(!state.armed){
        ROS_WARN("Takeoff rejected. Arm first.");
        return;
    }

    if(lpos.pose.position.z > 1.0){
        ROS_WARN("Takeoff rejected. Already took off.");
        return;
    }

    double home_alt = hp.altitude;
    double home_lon = hp.longitude;
    double home_lat = hp.latitude;

    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = home_alt + height;
    takeoff_cmd.request.longitude = home_lon;
    takeoff_cmd.request.latitude = home_lat;

    ros::Time takeoff_last_request = ros::Time::now();

    ROS_DEBUG("set lat: %f, lon: %f, alt: %f", home_lat, home_lon, home_alt);

    while( not(client.call(takeoff_cmd)) and
                takeoff_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
        if(ros::Time::now()-takeoff_last_request>ros::Duration(3.0)){
            ROS_WARN("Takeoff service call failed.");
            return;
        }
    }

    takeoff_last_request = ros::Time::now();

    while(lpos.pose.position.z < height-0.1){
        ros::spinOnce();
        rate.sleep();
        if(ros::Time::now()-takeoff_last_request>ros::Duration(3.0)){
            ROS_WARN("Takeoff failed.");
            return;
        }
    }
    ROS_INFO("Vehicle tookoff.");
    return;
}

/**
* @brief Land vehicle
* @param client Service client for landing
* @param lpos Current local position
* @param gpos Global position at the time of landing
* @param hp Home position
*/
void land(ros::ServiceClient& client,
            geometry_msgs::PoseStamped& lpos,
            sensor_msgs::NavSatFix& gpos,
            sensor_msgs::NavSatFix& hp){

    ros::Rate rate(20);

    double curr_alt = gpos.altitude;
    double curr_lon = gpos.longitude;
    double curr_lat = gpos.latitude;
    double home_alt = hp.altitude;

    mavros_msgs::CommandTOL landing_cmd;
    landing_cmd.request.altitude = curr_alt - (curr_alt-home_alt) + 0.5;
    landing_cmd.request.longitude = curr_lon;
    landing_cmd.request.latitude = curr_lat;

    while( not(client.call(landing_cmd)) and
                landing_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landing...");

    while(lpos.pose.position.z > 0.1){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landed.");
    return;
}

/**
* @brief Disarm vehicle
* @param client Service client for disarm
*/
void disarm(ros::ServiceClient& client){

    ros::Rate rate(20);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    while(!client.call(arm_cmd) && arm_cmd.response.success){
    }
    ROS_INFO("Vehicle disarmed");
    return;
}

/**
* @brief Send velocity command to vehicle
* @param pub Velocity command publisher
* @param client Service client for mode setting
* @param state Current state of vehicle
* @param dt Duration
* @param vx Linear velocity along x axis
* @param vy Linear velocity along y axis
* @param vz Linear velocity along z axis
* @param ang_z Angular velocity around z axis
*/
void cmd_vel(ros::Publisher& pub,
             ros::ServiceClient& client,
             mavros_msgs::State& state,
             double dt, double vx, double vy, double vz, double ang_z){

    ros::Rate rate(20);

    ros::Time start = ros::Time::now();
    ros::Time last_request = ros::Time::now();

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    geometry_msgs::TwistStamped target_vel_msg;
    target_vel_msg.twist.linear.x = vx;
    target_vel_msg.twist.linear.y = vy;
    target_vel_msg.twist.linear.z = vz;
    target_vel_msg.twist.angular.z = ang_z;

    ROS_INFO("Vehicle moving ...");

    while (ros::Time::now() - start < ros::Duration(dt)){
        pub.publish(target_vel_msg);

        if(state.mode!="OFFBOARD" and
            (ros::Time::now() - last_request > ros::Duration(0.1))){
            if((client.call(offb_set_mode))){
                ROS_INFO("Offboard enabled.");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle arrived destination.");


    mavros_msgs::SetMode loiter_set_mode;
    loiter_set_mode.request.custom_mode = "AUTO.LOITER";
    while( not(client.call(loiter_set_mode))){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle hovering.");
    return;
}


#endif