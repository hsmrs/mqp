/****************************************************************************************
*** File: drive_manager.cpp													 		  ***
*** Author: HSMRS MQP Team															  ***
*** Date: 11/02/2014																  ***
***																					  ***
*** Description: This file creates a ROS node to interpret joystick input and output  ***
				 velocity commands for the EPOS motor controllers. Code adapted from  ***
				 the Oryx 2 MQP team. 												  ***
****************************************************************************************/

#include <epos_manager/GroupEPOSControl.h>
#include <epos_manager/EPOSControl.h>
#include <epos_manager/GroupMotorInfo.h>
#include <epos_manager/MotorInfo.h>

epos_manager::GroupEPOSControl joyToDriveMessage(float left_value, float right_value,float scale){
	epos_manager::GroupEPOSControl drive_msg;
	epos_manager::EPOSControl motor_a, motor_b, motor_c, motor_d;

	motor_a.node_id=0;
	motor_b.node_id=1;
	motor_c.node_id=2;
	motor_d.node_id=3;

	motor_a.control_mode=epos_manager::EPOSControl::VELOCITY;
	motor_b.control_mode=epos_manager::EPOSControl::VELOCITY;
	motor_c.control_mode=epos_manager::EPOSControl::VELOCITY;
	motor_d.control_mode=epos_manager::EPOSControl::VELOCITY;

	//Use Tom's code here: map left and right joy axes to [speed_a, speed_b, speed_c, speed_d]

	motor_a.setpoint=speed_a;
	motor_b.setpoint=speed_b;
	motor_c.setpoint=speed_c;
	motor_d.setpoint=speed_d;

	drive_msg.motor_group.push_back(motor_a);
	drive_msg.motor_group.push_back(motor_b);
	drive_msg.motor_group.push_back(motor_c);
	drive_msg.motor_group.push_back(motor_d);

	return drive_msg;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
	if (msg->buttons[LEFT_BUTTON] <.1 && msg->buttons[RIGHT_BUTTON] <.1) {
		epos_manager::GroupEPOSControl drive_msg;
		float scale = 1;

		 //Drive Straight Forward
		if (msg->axes[D_PAD_UP_DOWN] > 0){
			drive_msg = joyToDriveMessage(1, 1, scale);
		}
		//Drive Straight Backward
		else if (msg->axes[D_PAD_UP_DOWN] < 0){
			drive_msg = joyToDriveMessage(-1, -1, scale);
		}
		//Full Left Turn
		else if (msg->axes[D_PAD_LEFT_RIGHT] > 0){
			drive_msg = joyToDriveMessage(-1, 1, scale);
		}
		//Full Right Turn
		else if (msg->axes[D_PAD_LEFT_RIGHT] < 0){
			drive_msg = joyToDriveMessage(1, -1, scale);
		}
		//Else, use joy sticks
		else drive_msg = joyToDriveMessage(msg->axes[LEFT_VERTICAL_AXIS],msg->axes[RIGHT_VERTICAL_AXIS], scale);

		group_drive_publisher.publish(drive_msg);
	}
}

/*void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
	epos_manager::GroupEPOSControl drive_msg;
	epos_manager::EPOSControl front_right,front_left,back_right,back_left;

	front_right.node_id=FRONT_RIGHT_WHEEL;
	back_right.node_id=BACK_RIGHT_WHEEL;
	front_left.node_id=FRONT_LEFT_WHEEL;
	back_left.node_id=BACK_LEFT_WHEEL;

	front_right.control_mode=epos_manager::EPOSControl::VELOCITY;
	back_right.control_mode=epos_manager::EPOSControl::VELOCITY;
	front_left.control_mode=epos_manager::EPOSControl::VELOCITY;
	back_left.control_mode=epos_manager::EPOSControl::VELOCITY;

	double angularToLinearCoef = (baseWidth*baseWidth + baseLength*baseLength)/(2*baseWidth);
	long left_speed  = velocityToRPM(msg->linear.x - msg->angular.z*angularToLinearCoef);
	long right_speed = -velocityToRPM(msg->linear.x + msg->angular.z*angularToLinearCoef);

	front_right.setpoint=right_speed;
	back_right.setpoint=right_speed;
	front_left.setpoint=left_speed;
	back_left.setpoint=left_speed;

	drive_msg.motor_group.push_back(front_right);
	drive_msg.motor_group.push_back(front_left);
	drive_msg.motor_group.push_back(back_left);
	drive_msg.motor_group.push_back(back_right);

	group_drive_publisher.publish(drive_msg);
}*/



int main (int argc, char **argv){
	ros::init(argc, argv, "hermes/drive_manager");
	ros::NodeHandle n;

	//ros::param::get("~Max_Velocity", maxVelocity);
	//ros::param::get("~Base_Width", baseWidth);
	//ros::param::get("~Base_Length", baseLength);

	//maxRPM = velocityToRPM(maxVelocity);
	ros::Subscriber joy_subscriber = n.subscribe("joy", 1, joyCallback);
	//ros::Subscriber drive_motor_twist_subscriber = n.subscribe("motors/Drive_Motors/Twist",1,twistCallback);
	group_drive_publisher = n.advertise<epos_manager::GroupEPOSControl>("motors/Drive_Motors/Group_Motor_Control", 1);

	//dynamic_reconfigure::Server<oryx_manager::drive_managerConfig> server;
	//dynamic_reconfigure::Server<oryx_manager::drive_managerConfig>::CallbackType f;

	//f = boost::bind(reconfigureCallback, _1 ,_2);
//	server.setCallback(f);

	ros::spin();
}

/*int velocityToRPM(float velocity){
	velocity = velocity/WHEEL_DIAMETER/3.14159265*60*GEAR_RATIO;
	return (int) velocity;
}*/

