
#ifndef _FOLLOW_TAG_BEHAVIOR_H_
#define _FOLLOW_TAG_BEHAVIOR_H_

#include "thor/Behavior.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "thor/thor_node.h"

class Thor;

class FollowTagBehavior : public Behavior{

private:
	const std::string MARKER_TOPIC;

	double maxLinearVelocity, maxAngularVelocity;
	ros::Publisher cmdVelPub;
	ros::Subscriber laserSub;
	ros::Subscriber markerSub;

	int tagID;
	bool isExecuting;
	bool isObstacle;
	std::string progress;


	boost::mutex progressMutex;

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

	void tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);

public:

	FollowTagBehavior(Thor* parent, double maxLinearVelocity, double maxAngularVelocity, int tagID, ros::NodeHandle n, 
		std::string cmdVelTopic, std::string laserTopic);

	~FollowTagBehavior();

	virtual void execute();

	virtual void resume();

	virtual void pause();

	virtual void stop();

	virtual std::string checkProgress();
};


#endif
