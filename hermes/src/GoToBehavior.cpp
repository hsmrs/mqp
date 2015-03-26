#include "hermes/GoToBehavior.h"

int instance;
bool global_isExecuting;
std::string global_progress;

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
	
	info = "robot: " + parent->getName();
	global_isExecuting = false;
	global_progress = "";
}
/*
GoToBehavior::~GoToBehavior(){
	goalPub.shutdown();
	progressPub.shutdown();
	progressSub.shutdown();
}*/

void GoToBehavior::execute(){
	ROS_INFO("GoToBehavior::execute");
	ROS_INFO("Sending goal");
	try
	{
	    global_isExecuting = true;
      	goalPub.publish(goalMsg);
  	}
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error GoToBehavior::execute %s", e.what());
  	}
}

void GoToBehavior::resume(){
	ROS_INFO("GoToBehavior::resume");
    try
    {
	    global_isExecuting = true;
	    goalPub.publish(goalMsg);
	}
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error GoToBehavior::resume %s", e.what());
  	}
}

void GoToBehavior::pause(){
	ROS_INFO("GoToBehavior::pause");
    try
    {
	    global_isExecuting = false;
	    cancelPub.publish(cancelMsg);
    }
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error in GoToBehavior::pause %s", e.what());
  	}
}

void GoToBehavior::stop(){
	ROS_INFO("GoToBehavior::stop");
    try
    {
	    global_isExecuting = false;
	    cancelPub.publish(cancelMsg);
    }
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error in GoToBehavior::stop %s", e.what());
  	}
}

std::string GoToBehavior::checkProgress(){
	ROS_INFO("GoToBehavior::checkProgress");
	return global_progress;
}

void GoToBehavior::progressCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("GoToBehavior::progressCallback");
	std::string progress = msg->data;
	
	if (progress == "complete" && global_isExecuting){
	    global_isExecuting = false;
		
	    //Tell parent task is complete.
	    //protected_progress.store(100);
	    global_progress = "complete";
	    ROS_INFO("GoToTask complete");
	}
	
}
