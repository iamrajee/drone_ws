#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Range.h"
#include <hemd/markerInfo.h>
#include <hemd/line.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <fstream>
#include <iostream>
using namespace std;

ofstream myfile;
float z, c1, c2;
bool _detected=false;
int mode_line=1;
int _flag_c = 1;
double yaw, yaw_set, yaw_init;

geometry_msgs::PoseStamped setpoint;
tf::Quaternion q;
tf::Quaternion q1;
geometry_msgs::PoseStamped mocap;



void linecb(const hemd::line::ConstPtr& msg)
{
  mode_line = msg->mode;
  yaw_set = msg->slope + yaw;
  c1 = msg->c1;
  c2 = msg->c2;

}
void posecb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  z = msg->pose.position.z;
}
void markercb(const hemd::markerInfo::ConstPtr& msg)
{
  _detected = msg->detect;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

	tf::Quaternion q(0, 0, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3 m(q);
	double r, p;
	m.getRPY(r,p,yaw);

}

void statecb(const mavros_msgs::State::ConstPtr& msg){
	if(msg->mode != "OFFBOARD"){
		yaw_init = yaw;
	}
}

void takeoff()
{
	setpoint.pose.position.z = 0.5f;
	q.setRPY(0, 0, yaw_init);
	mocap.pose.position.y = 0.0f; 
	setpoint.pose.orientation.z = q.z();
	setpoint.pose.orientation.w = q.w();
	cout<<"takeoff"<<endl;

}

void em_land()
{
	setpoint.pose.position.z = 0.0f;
	q.setRPY(0, 0, yaw_init);
	setpoint.pose.orientation.z = q.z();
	setpoint.pose.orientation.w = q.w();
	mocap.pose.position.y = 0;
	mocap.pose.position.x = 0;
	mocap.pose.position.z = 0;
	cout<<"emergency_land"<<endl;
}


void hold_line()
{
	myfile << yaw_set<<endl;
	setpoint.pose.position.z = 0.9f;
	q1.setRPY(0, 0, yaw_set);
	setpoint.pose.orientation.z = q1.z();
	setpoint.pose.orientation.w = q1.w();
	mocap.pose.position.y = c1;
	mocap.pose.position.x = c2;
	mocap.pose.position.z = -1.0;
	cout<<"hold_line"<<endl;

}

void turn()
{
	myfile << yaw_set<<endl;
	setpoint.pose.position.z = 0.9f;
	q1.setRPY(0, 0, yaw_set);
	setpoint.pose.orientation.z = q1.z();
	setpoint.pose.orientation.w = q1.w();
	mocap.pose.position.y = c1;
	mocap.pose.position.x = c2;
	mocap.pose.position.z = -3.0;
	cout<<"turn"<<endl;

}

void hold_cross()
{
	cout<< "cross" << endl;
	setpoint.pose.position.z = 0.9f;
	q1.setRPY(0, 0, yaw_set);
	setpoint.pose.orientation.z = q1.z();
	setpoint.pose.orientation.w = q1.w();
	mocap.pose.position.y = c1;
	mocap.pose.position.x = c2;	
	mocap.pose.position.z = -2.0;

}
void cruise()
{
switch (_flag_c) {
  			
  			case 1:
  				
  				hold_line();
  				if (mode_line == 2)
  				{
  					_flag_c = 2;
  				}
				else if (mode_line == 3)
				{
					_flag_c = 3;
				}
  				break;
  			
  			case 2:
  				
				if (mode_line == 1)
  				{
  					_flag_c = 1;
					return;
  				}
				else if (mode_line == 3)
				{
					_flag_c = 3;
				}
  				hold_cross();
  				break;

			case 3:

				turn();
				if (mode_line == 2)
				{
					_flag_c = 2;
				}
				else if (mode_line == 1)
  				{
  					_flag_c = 1;
					return;
  				}
				break;
  			case 0:
  				
  				 em_land(); //emergency_land
				 break;
  				
  			} 

	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/mavros/vision_pose/pose", 10, posecb);
  ros::Subscriber sub_marker = n.subscribe("/marker", 10, markercb);
  ros::Subscriber sub_line = n.subscribe("/warehouse_quad/line",10, linecb);
  ros::Subscriber sub1 = n.subscribe("/mavros/imu/data",100, imuCallback);
  ros::Subscriber sub2 = n.subscribe("/mavros/state",10, statecb);
  

  ros::Publisher setpoint_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  ros::Publisher mocap_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);


  ros::Rate loop_rate(100);

  int _flag=1;
  	
  while (ros::ok())
  {

    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "quad";
	setpoint.pose.position.x = 0.0f;
    setpoint.pose.position.y = 0.0f;
    
    mocap.header.stamp = ros::Time::now();
    mocap.header.frame_id = "line";
	
    switch (_flag) {

      case 1:  //takeoff
        takeoff();
       if (1)
	 	{
          _flag = 2;

        }
        break;

      case 2:  //forward
  		
  		if (mode_line == 0)
  		{
  			_flag = 3;
  		}
  		
  		cruise();
  		break;
      
      case 3: 
        
        em_land(); 
        break;

      default:
		
		em_land(); 
     	break;
    }

    setpoint_pub.publish(setpoint);
    mocap_pub.publish(mocap);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
