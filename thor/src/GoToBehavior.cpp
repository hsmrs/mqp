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
	
	info = "robot: " + parent->getName();
}

GoToBehavior::~GoToBehavior(){
	goalPub.shutdown();
	progressPub.shutdown();
	progressSub.shutdown();
}

void GoToBehavior::execute(){
	ROS_INFO("Sending goal");
	try
	{
	    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    isExecuting = true;
	    isExecutingLock.unlock();
      	goalPub.publish(goalMsg);
  	}
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error %s", e.what());
  	}
}

void GoToBehavior::resume(){
    try
    {
        boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    isExecuting = true;
	    isExecutingLock.unlock();
	    goalPub.publish(goalMsg);
	}
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error %s", e.what());
  	}
}

void GoToBehavior::pause(){
    try
    {
        boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    isExecuting = false;
	    isExecutingLock.unlock();
	    cancelPub.publish(cancelMsg);
    }
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error %s", e.what());
  	}
}

void GoToBehavior::stop(){
    try
    {
        boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    isExecuting = false;
	    isExecutingLock.unlock();
	    cancelPub.publish(cancelMsg);
    }
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error %s", e.what());
  	}
}

std::string GoToBehavior::checkProgress(){
	boost::mutex::scoped_lock progressLock(progressMutex);
	return progress;
}

void GoToBehavior::progressCallback(const std_msgs::String::ConstPtr& msg){
	std::string progress = msg->data;
	try
	{
	    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    if (progress == "complete" && isExecuting){
		    isExecuting = false;
		
		    //Tell thor task is complete.
		    boost::mutex::scoped_lock progressLock(progressMutex);
		    this->progress = "complete";
		    progressLock.unlock();
		    ROS_INFO("GoToTask complete, calling cancelTask on %s", info.c_str());
		    //progressPub.publish(*msg);
	    }
	    isExecutingLock.unlock();
	}
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error %s", e.what());
  	}
}
