
#ifndef _GO_TO_BEHAVIOR_H_
#define _GO_TO_BEHAVIOR_H_

#include "thor/Behavior.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "hsmrs_framework/Robot.h"
#include "geometry_msgs/PointStamped.h"
#include <boost/thread/mutex.hpp>
#include <mutex>

class GoToBehavior : public Behavior{

private:
	
	Robot* parent;
	bool isExecuting;
	geometry_msgs::PointStamped goalMsg;
	std_msgs::String cancelMsg;
	ros::Publisher goalPub;
	ros::Publisher cancelPub;
	ros::Publisher progressPub;
	ros::Subscriber progressSub;
	
	std::string info;
	std::string progress;

	//boost::mutex isExecutingMutex;
	//boost::mutex progressMutex;
	std::recursive_mutex isExecutingMutex;
	std::recursive_mutex progressMutex;

public:

	GoToBehavior(Robot* parent, geometry_msgs::Point goal, ros::NodeHandle n);

	~GoToBehavior();

	virtual void execute();

	virtual void resume();

	virtual void pause();

	virtual void stop();

	virtual std::string checkProgress();
	
	void progressCallback(const std_msgs::String::ConstPtr& msg);
};


#endif
