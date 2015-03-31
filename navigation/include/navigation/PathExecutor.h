#ifndef _PLAN_EXECUTOR_H
#define _PLAN_EXECUTOR_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include <cmath>
#include <queue>

class PlanExecutor {
private:

	ros::Publisher velPub;
	ros::Publisher progressPub;
	ros::Publisher nextPointPub;
	ros::Subscriber poseSub;
	ros::Subscriber pathSub;
	ros::Subscriber cancelSub;

	geometry_msgs::Pose currentPose;
	std::deque<geometry_msgs::Pose> path;

	double maxLinearVel;
	double maxAngularVel;

	bool isCanceled;

public:

	PlanExecutor();

	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);

	void pathCallback(const nav_msgs::Path::ConstPtr& pathMsg);

	void cancelCallback(const std_msgs::String::ConstPtr cancelMsg);

	void executePath();

	double distance(double x1, double y1, double x2, double y2);
};

#endif
