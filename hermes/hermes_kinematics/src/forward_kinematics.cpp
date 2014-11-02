/****************************************************************************************
*** File: forward_kinematics.cpp													  ***
*** Author: HSMRS MQP Team															  ***
*** Date: 11/01/2014																  ***
***																					  ***
*** Description: This file creates a ROS node to calculate the forward kinematics of  ***
***              the Hermes robot.  It accepts twist messages for the rotation speeds ***
***				 of the four wheels and outputs an Odometry message for the velocity  ***
***              of the robot.														  ***
****************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

//Global Variables
ros::Publisher odom_pub;

//Function prototypes
void publishOdometry();
void flMotorTwistCallback(geometry_msgs::Twist);
void frMotorTwistCallback(geometry_msgs::Twist);
void blMotorTwistCallback(geometry_msgs::Twist);
void brMotorTwistCallback(geometry_msgs::Twist);


int main(int argc, char **argv)
{
	//ROS Setup
	ROS_INFO("%s", msg.data.c_str());
	ros::init(argc, argv, "hermes/kinematics/forward_kinematics");
	ros::NodeHandle n;

	//Publishers
	odom_pub = n.advertise<nav_msgs::Odometry>("/hermes/odom", 1000);

	//Subscribers

	//Main loop
	while (ros::ok())
	{

	}

	return 0;
}

void publishOdometry()
{
	odom_pub.publish(msg);
}

void flMotorTwistCallback(geometry_msgs::Twist)
{

}

void frMotorTwistCallback(geometry_msgs::Twist)
{

}

void blMotorTwistCallback(geometry_msgs::Twist)
{

}

void brMotorTwistCallback(geometry_msgs::Twist)
{
	
}
