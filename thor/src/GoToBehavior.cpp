#include "thor/GoToBehavior.h"

GoToBehavior::GoToBehavior(Robot* parent, geometry_msgs::Point goal, ros::NodeHandle n){
	this->parent = parent;
	goalMsg = goal;
	goalPub = n.advertise<geometry_msgs::Point>("thor/goal", 1000, true);
	cancelMsg.data = "cancel";
	cancelPub = n.advertise<std_msgs::String>("thor/navigation/cancel", 1000);
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