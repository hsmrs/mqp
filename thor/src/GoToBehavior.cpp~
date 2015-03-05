#include "thor/GoToBehavior.h"

int instance;

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
	myInstance = instance++;
}

GoToBehavior::~GoToBehavior(){
	goalPub.shutdown();
	progressPub.shutdown();
	progressSub.shutdown();
}

void GoToBehavior::execute(){
	ROS_INFO("GoToBehavior::execute");
	ROS_INFO("Sending goal");
	try
	{
	    //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	    //isExecuting = true;
	    protected_isExecuting.store(true);
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
	    protected_isExecuting.store(true);
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
	    protected_isExecuting.store(false);
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
	    protected_isExecuting.store(false);
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
	return (protected_progress.load() == 100 ? "complete" : "");
}

void GoToBehavior::progressCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("GoToBehavior::progressCallback on instance: %d", myInstance);
	std::string progress = msg->data;
	try
	{
	    //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	    //boost::mutex::scoped_lock progressLock(progressMutex);
		//std::unique_lock<std::recursive_mutex> progressLock(progressMutex);
		if (progress == "complete" && protected_isExecuting.load()){
		    //isExecuting = false;
		    protected_isExecuting.store(false);
		
		    //Tell thor task is complete.
		    //this->progress = "complete";
		    protected_progress.store(100);
		    ROS_INFO("GoToTask complete, calling cancelTask on %s", info.c_str());
		    //progressPub.publish(*msg);
	    }
	    //progressLock.unlock();
	    //isExecutingLock.unlock();
	}
  	catch(boost::lock_error& e)
  	{
  	    ROS_INFO("lock error in GoToBehavior::progressCallback %s", e.what());
  	}
}
