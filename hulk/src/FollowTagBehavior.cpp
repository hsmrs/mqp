#include "hulk/FollowTagBehavior.h"

void FollowTagBehavior::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	isObstacle = false;
	if (isObstacle == true){
		geometry_msgs::Twist velMsg;
		velMsg.linear.x = 0;
		velMsg.angular.z = 0;

		cmdVelPub.publish(velMsg);

	}
}

void FollowTagBehavior::tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
	if (!isExecuting || 
		msg->markers.size() == 0) return;

	double linK = 0.4;
	double angK = 2;	
	double x = msg->markers[0].pose.pose.position.x;
	double y = msg->markers[0].pose.pose.position.y;
	double linVel = std::max(std::min(maxLinearVelocity, linK * x), 0.0);
	double angVel = std::max(std::min(maxAngularVelocity, angK * y), -1*maxAngularVelocity);

	if (x <= 1.0){
		linVel = 0;
	}

	geometry_msgs::Twist velMsg;
	velMsg.linear.x = linVel;
	velMsg.angular.z = angVel;

	cmdVelPub.publish(velMsg);
}


FollowTagBehavior::FollowTagBehavior(Hulk* parent, double maxLinearVelocity, double maxAngularVelocity, int tagID, 
	ros::NodeHandle n, std::string cmdVelTopic, std::string laserTopic) : 
MARKER_TOPIC("/ar_pose_marker")
{
	isExecuting = false;
	isObstacle = false;

	this->maxLinearVelocity = maxLinearVelocity;
	this->maxAngularVelocity = maxAngularVelocity;
	this->tagID = tagID;

	ros::Publisher cmdVelPub = n.advertise<geometry_msgs::Twist>(cmdVelTopic, 1000);
	ros::Subscriber laserSub = n.subscribe(laserTopic, 1000, &FollowTagBehavior::laserCallback, this);
	ros::Subscriber markerSub = n.subscribe(MARKER_TOPIC, 1000, &FollowTagBehavior::tagCallback, this);
}

void FollowTagBehavior::execute(){
	isExecuting = true;
}

void FollowTagBehavior::resume(){
	isExecuting = true;
}

void FollowTagBehavior::pause(){
	isExecuting = false;
}

void FollowTagBehavior::stop(){
	isExecuting = false;
}