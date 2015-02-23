
#ifndef _SEARCH_BEHAVIOR_H_
#define _SEARCH_BEHAVIOR_H_

#include "thor/Behavior.h"
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "thor/thor_node.h"

class Thor;

class SearchBehavior : public Behavior{

private:
	const std::string MARKER_TOPIC;

	ros::Subscriber markerSub;
	ros::Subscriber progressSub;

	ros::Publisher cmdVelPub;
	ros::Publisher goalPub;
	ros::Publisher cancelPub;

	Robot* parent;
	int tagID;
	std::vector<geometry_msgs::PointStamped> boundaryVertices;
	std::vector<geometry_msgs::PointStamped> goals;
	geometry_msgs::PointStamped goalMsg;
	double maxLinearVelocity, maxAngularVelocity;
	double resolution;
	bool isExecuting;
	bool isFound;
	std_msgs::String cancelMsg;


	void tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);

	void progressCallback(const std_msgs::String::ConstPtr& msg);

	void createGoals();

public:

	SearchBehavior(Robot* parent, double maxLinearVelocity, double maxAngularVelocity, double resolution, int tagID, 
		std::vector<geometry_msgs::PointStamped> boundaryVertices, ros::NodeHandle n, std::string cmdVelTopic);

	virtual void execute();

	virtual void resume();

	virtual void pause();

	virtual void stop();
};


#endif
