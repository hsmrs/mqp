#include "ros/ros.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include <tf/transform_broadcaster.h>

//ros::Subscriber tag_sub;
tf::TransformBroadcaster *br;

void tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
	ROS_INFO("Hit!");
	if (msg->markers.size() == 0) return; 

	//Figure out which tag we are looking at.
	int tagID = msg->markers[0].id; 
	std::stringstream ss;
	ss << "hsmrs/marker_" << tagID;
	std::string markerFrameID = ss.str();

	//Manually create transform from robot to tag
	tf::Transform transform;

	double x, y, z;
	x = msg->markers[0].pose.pose.position.x;
	y = msg->markers[0].pose.pose.position.y;
	z = msg->markers[0].pose.pose.position.z;

	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->markers[0].pose.pose.orientation, q);

	transform.setOrigin( tf::Vector3(x, y, z) );
	transform.setRotation(q);

	//Broadcast transform
	ROS_INFO("Broadcasting transform!");
	br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot", markerFrameID));
	ROS_INFO("Transform broadcasted!");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "transform_test");

	ros::NodeHandle nh;
	ros::Subscriber tag_sub = nh.subscribe("ar_pose_marker", 1000, tagCallback);
	br = new tf::TransformBroadcaster;

	ros::spin();
	
	return 0;
}
