
#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include "ros/ros.h"

class Behavior {

private:
	double maxLinearVelocity, maxAngularVelocity;
	ros::Publisher cmdVelPub;
	
public:

	Behavior(){}

	virtual void execute() = 0;

	virtual void resume() = 0;

	virtual void pause() = 0;

	virtual void stop() = 0;
};

#endif