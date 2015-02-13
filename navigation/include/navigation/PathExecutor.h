#ifndef _PLAN_EXECUTOR_H
#define _PLAN_EXECUTOR_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"

#include <cmath>
#include <queue>

class PlanExecutor {
private:

	ros::Publisher velPub;
	ros::Subscriber poseSub;
	ros::Subscriber pathSub;

	geometry_msgs::Pose currentPose;
	std::deque<geometry_msgs::Pose> path;

	double maxLinearVel;
	double maxAngularVel;

public:

	PlanExecutor();

	void poseCallback(const geometry_msgs::Pose::ConstPtr& poseMsg);

	void pathCallback(const nav_msgs::Path::ConstPtr& pathMsg);

	void executePath();

	double distance(double x1, double y1, double x2, double y2);
};

#endif