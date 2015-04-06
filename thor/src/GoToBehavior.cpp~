#include "thor/GoToBehavior.h"

int instance;
bool bad_isExecuting;
std::string bad_progress;

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
	protected_progress.store(0);
	protected_isExecuting.store(false);
	bad_isExecuting = false;
	bad_progress = "";
	myInstance = instance++;
	ROS_INFO("Creating GoToBehavior #%d", myInstance);
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
	    //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	    //isExecuting = true;
	    //protected_isExecuting.store(true);
	    bad_isExecuting = true;
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
        //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	    //isExecuting = true;
	    //isExecutingLock.unlock();
	    //protected_isExecuting.store(true);
	    bad_isExecuting = true;
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
        //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	    //isExecuting = false;
	    //protected_isExecuting.store(false);
	    bad_isExecuting = false;
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
        //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	    //isExecuting = false;
	    //protected_isExecuting.store(false);
	    bad_isExecuting = false;
	    cancelPub.publish(cancelMsg);
    }
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error in GoToBehavior::stop %s", e.what());
  	}
}

std::string GoToBehavior::checkProgress(){
	ROS_INFO("GoToBehavior::checkProgress");
	//boost::mutex::scoped_lock progressLock(progressMutex);
	//std::unique_lock<std::recursive_mutex> progressLock(progressMutex);
	//return progress;
	//return (protected_progress.load() == 100 ? "complete" : "");
	return bad_progress;
}

void GoToBehavior::progressCallback(const std_msgs::String::ConstPtr& msg){
    if (&bad_progress == NULL){
        ROS_INFO("NULL confirmed on bad progress");
    }
    /*if (this == NULL){
        ROS_INFO("NULL confirmed");
    }
	ROS_INFO("GoToBehavior::progressCallback on instance: %d", myInstance);
	//ROS_INFO("Parent: %s", parent->getName().c_str());
	ROS_INFO("CancelMsg: %s", cancelMsg.data.c_str());
	ROS_INFO("Info: %s", info.c_str());
	ROS_INFO("Progress: %d", protected_progress.load());*/
	std::string progress = msg->data;
	ROS_INFO("Checkpoint #1");
	try
	{
	    //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	    //boost::mutex::scoped_lock progressLock(progressMutex);
		//std::unique_lock<std::recursive_mutex> progressLock(progressMutex);
		ROS_INFO("Checkpoint #2");
		//if (progress == "complete" && protected_isExecuting.load()){
		if (progress == "complete" && bad_isExecuting){
		    //isExecuting = false;
		    ROS_INFO("Checkpoint #3");
		    //protected_isExecuting.store(false);
		    bad_isExecuting = false;
		    ROS_INFO("Sleep");
		    ros::Duration(1).sleep();
		
		    //Tell thor task is complete.
		    //this->progress = "complete";
		    ROS_INFO("Checkpoint #4");
		    //protected_progress.store(100);
	        bad_progress = "complete";
		    ROS_INFO("GoToTask complete");//, calling cancelTask on %s", info.c_str());
		    //progressPub.publish(*msg);
	    }
	    //progressLock.unlock();
	    //isExecutingLock.unlock();
	    ROS_INFO("Checkpoint #5");
	}
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error in GoToBehavior::progressCallback %s", e.what());
  	}
  	ROS_INFO("Checkpoint #6");
}
