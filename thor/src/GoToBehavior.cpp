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
	progressPub = n.advertise<std_msgs::String>("progress", 1000);
}

void GoToBehavior::execute(){
	ROS_INFO("Sending goal");
	boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	isExecuting = true;
  	goalPub.publish(goalMsg);
}

void GoToBehavior::resume(){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	isExecuting = true;
	goalPub.publish(goalMsg);
}

void GoToBehavior::pause(){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	isExecuting = false;
	cancelPub.publish(cancelMsg);
}

void GoToBehavior::stop(){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	isExecuting = false;
	cancelPub.publish(cancelMsg);
}

void GoToBehavior::progressCallback(const std_msgs::String::ConstPtr& msg){
	std::string progress = msg->data;
	boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	if (progress == "complete" && isExecuting){
		isExecuting = false;
		
		//Tell thor task is complete.
		ROS_INFO("GoToTask complete, calling cancelTask");
		progressPub.publish(*msg);
	}
}
