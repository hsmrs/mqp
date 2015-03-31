#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

ros::Publisher newOdomPub;
std::string odomTopic, newOdomTopic, newOdomFrame, newBaseFrame;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	nav_msgs::Odometry newMsg = *msg;
	newMsg.header.frame_id = newOdomFrame;
	newMsg.child_frame_id = newBaseFrame;
	newMsg.header.stamp = ros::Time::now();
	newOdomPub.publish(newMsg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_frame_swap");

	ros::NodeHandle n;
	
	ros::param::param<std::string>("~odom_topic", odomTopic, "odom");
	ros::param::param<std::string>("~new_odom_topic", newOdomTopic, "wheel_odom");
	ros::param::param<std::string>("~new_odom_frame", newOdomFrame, "wheel_odom");
	ros::param::param<std::string>("~new_base_frame", newBaseFrame, "thor");
	
	ros::Subscriber odomSub = n.subscribe(odomTopic, 1000, odomCallback);
	newOdomPub = n.advertise<nav_msgs::Odometry>(newOdomTopic, 100);
	
	ros::spin();
	
	return 0;
}
