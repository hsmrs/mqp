#include "navigation/ArTagLocalization.h"

ArTagLocalization::ArTagLocalization() : ODOM_PUB_TOPIC("/ar_tag_odom") {

	ros::NodeHandle nh;

	std::string markerTopic;
	std::string odomTopic;
	ros::param::param<std::string>("~marker_topic", markerTopic, "ar_pose_marker");
	ros::param::param<std::string>("~odom_topic", odomTopic, "ar_tag_odom");
	
	isFirst = true;

	tagSub = nh.subscribe(markerTopic, 100, &ArTagLocalization::tagCallback, this);
	odomPub = nh.advertise<nav_msgs::Odometry>(odomTopic, 100);

	ros::spin();
}

void ArTagLocalization::tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
	if (msg->markers.size() == 0) return;

	//Figure out which tag we are looking at.
	int tagID = msg->markers[0].id;
	std::stringstream ss;
	ss << "/hsmrs/marker_" << tagID;
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
	//ROS_INFO("Broadcasting transform!");
	br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), markerFrameID, "thor"));
	//ROS_INFO("Transform broadcasted!");
	
	//Listen for transform from map to robot
	//ROS_INFO("Listening for transform!");
	tf::StampedTransform mapTransform;
	try{
		listener.waitForTransform("/map", "/thor",
			ros::Time(0), ros::Duration(0.1));
		listener.lookupTransform("/map", "/thor",
			ros::Time(0), mapTransform);
		//ROS_INFO("Transform heard");	
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	if (isFirst){
		isFirst = false;
		odomAnchor = *(new tf::Transform(mapTransform.getRotation(), mapTransform.getOrigin()));
	}
	br.sendTransform(tf::StampedTransform(odomAnchor, ros::Time::now(), "/map", "odom"));

	//Unpack position and orientation
	double map_x, map_y;
	map_x = mapTransform.getOrigin().x();
	map_y = mapTransform.getOrigin().y();

	geometry_msgs::Quaternion quat;
	tf::quaternionTFToMsg(mapTransform.getRotation(), quat);

	//Publish pose as odometry message
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.pose.pose.position.x = map_x;
	odom_msg.pose.pose.position.y = map_y;
	odom_msg.pose.pose.orientation = quat;
	
	for (int i = 0; i < 36; i++){
		odom_msg.pose.covariance[i] = 1;
		odom_msg.twist.covariance[i] = 1000;
	}

	odomPub.publish(odom_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ar_tag_localization");

	ArTagLocalization* loc = new ArTagLocalization();
	return 0;
}
