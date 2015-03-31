#include "husky/FollowTagBehavior.h"

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

	for (int i = 0; i < msg->markers.size(); ++i){
		if (msg->markers[i].id == tagID){
			double linK = 0.4;
			double angK = 2;	
			double x = msg->markers[i].pose.pose.position.x;
			double y = msg->markers[i].pose.pose.position.y;
			double linVel = std::max(std::min(maxLinearVelocity, linK * x), 0.0);
			double angVel = std::max(std::min(maxAngularVelocity, angK * y), -1*maxAngularVelocity);

			if (x <= 1.0){
				linVel = 0;
			}

			geometry_msgs::Twist velMsg;
			velMsg.linear.x = linVel;
			velMsg.angular.z = angVel;

			cmdVelPub.publish(velMsg);

			return;
		}
	}
}


FollowTagBehavior::FollowTagBehavior(Robot* parent, double maxLinearVelocity, double maxAngularVelocity, int tagID,
	ros::NodeHandle n, std::string cmdVelTopic, std::string laserTopic) : 
MARKER_TOPIC("ar_pose_marker")
{
	isExecuting = false;
	isObstacle = false;

	this->maxLinearVelocity = maxLinearVelocity;
	this->maxAngularVelocity = maxAngularVelocity;
	this->tagID = tagID;
	
	cmdVelPub = n.advertise<geometry_msgs::Twist>(cmdVelTopic, 1000);
	laserSub = n.subscribe(laserTopic, 1000, &FollowTagBehavior::laserCallback, this);
	markerSub = n.subscribe(MARKER_TOPIC, 1000, &FollowTagBehavior::tagCallback, this);
}

FollowTagBehavior::~FollowTagBehavior(){
	cmdVelPub.shutdown();
	laserSub.shutdown();
	markerSub.shutdown();
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

std::string FollowTagBehavior::checkProgress(){
	boost::mutex::scoped_lock progressLock(progressMutex);
	return progress;
}
