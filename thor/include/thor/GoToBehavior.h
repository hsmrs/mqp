
#ifndef _GO_TO_BEHAVIOR_H_
#define _GO_TO_BEHAVIOR_H_

#include "thor/Behavior.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "hsmrs_framework/Robot.h"

class GoToBehavior : public Behavior{

private:
	
	Robot* parent;
	bool isExecuting;
	geometry_msgs::Point goalMsg;
	std_msgs::String cancelMsg;
	ros::Publisher goalPub;
	ros::Publisher cancelPub;

public:

	GoToBehavior(Robot* parent, geometry_msgs::Point goal, ros::NodeHandle n);

	virtual void execute();

	virtual void resume();

	virtual void pause();

	virtual void stop();
};


#endif