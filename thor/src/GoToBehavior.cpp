#include "thor/GoToBehavior.h"

GoToBehavior::GoToBehavior(Robot* parent, geometry_msgs::Point goal, ros::NodeHandle n){
	ROS_INFO("Creating GoTo behavior");
	this->parent = parent;
	goalMsg.header.frame_id = "map";
	goalMsg.point = goal;
	goalPub = n.advertise<geometry_msgs::PointStamped>("navigation/goal", 1000, true);
	cancelMsg.data = "cancel";
	cancelPub = n.advertise<std_msgs::String>("navigation/cancel", 1000);
	progressSub = n.subscribe("navigation/progress", 1000, &GoToBehavior::progressCallback, this);
}

void GoToBehavior::execute(){
	ROS_INFO("Sending goal");
  	goalPub.publish(goalMsg);
}

void GoToBehavior::resume(){
	goalPub.publish(goalMsg);
}

void GoToBehavior::pause(){
	cancelPub.publish(cancelMsg);
}

void GoToBehavior::stop(){
	cancelPub.publish(cancelMsg);
}

void GoToBehavior::progressCallback(const std_msgs::String::ConstPtr& msg){
	std::string progress = msg->data;
	if (progress == "complete"){
		//Tell thor task is complete.
		parent->cancelTask();
	}
}
