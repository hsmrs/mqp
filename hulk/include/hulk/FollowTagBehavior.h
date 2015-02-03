
#ifndef _FOLLOW_TAG_BEHAVIOR_H_
#define _FOLLOW_TAG_BEHAVIOR_H_

#include "hulk/Behavior.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "ar_track_alvar/AlvarMarkers.h"

class Hulk;

class FollowTagBehavior : public Behavior{

private:
	const std::string MARKER_TOPIC;

	double maxLinearVelocity, maxAngularVelocity;
	ros::Publisher cmdVelPub;

	int tagID;
	bool isExecuting;
	bool isObstacle;

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

	void tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);

public:

	FollowTagBehavior(Hulk* parent, double maxLinearVelocity, double maxAngularVelocity, int tagID, ros::NodeHandle n, 
		std::string cmdVelTopic, std::string laserTopic);

	virtual void execute();

	virtual void resume();

	virtual void pause();

	virtual void stop();
};


#endif