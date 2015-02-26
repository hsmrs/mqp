#ifndef _AR_TAG_LOCALIZATION_H_
#define _AR_TAG_LOCALIZATION_H_

#include "ros/ros.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <deque>

class ArTagLocalization {
private:
	const std::string ODOM_PUB_TOPIC;

	ros::Subscriber tagSub;
	ros::Publisher odomPub;

	tf::TransformBroadcaster br;
	tf::TransformListener listener;

	int lastMarkerTime;
	int lastMarkerX;
	int lastMarkerY;
	
	bool isFirst;
	tf::Transform odomAnchor;
	std::string odomFrame;
	
	std::deque<geometry_msgs::Pose> lastPoses;

	void tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
	geometry_msgs::Pose getOutputPose(double x, double y, double z, geometry_msgs::Quaternion q);

public:

	ArTagLocalization();
};

#endif
