#include "navigation/ArTagLocalization.h"

ArTagLocalization::ArTagLocalization() : ODOM_PUB_TOPIC("/ar_tag_odom") {

	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::string markerTopic;
	std::string odomTopic;
	ros::param::param<std::string>("~marker_topic", markerTopic, "ar_pose_marker");
	ros::param::param<std::string>("~odom_topic", odomTopic, "ar_tag_odom");
	ros::param::param<std::string>("~wheel_odom_frame", odomFrame, "wheel_odom");
	ros::param::param<std::string>("~target_frame", target_frame, "thor");
	
	isFirst = true;
	
	lastPoses = std::deque<geometry_msgs::Pose>();
	lastPoses.push_front(geometry_msgs::Pose());

	tagSub = nh.subscribe(markerTopic, 100, &ArTagLocalization::tagCallback, this);
	odomPub = nh.advertise<nav_msgs::Odometry>(odomTopic, 100);

	while(ros::ok()){
		if (!isFirst){
			br.sendTransform(tf::StampedTransform(odomAnchor, ros::Time::now(), "/map", odomFrame));
			br.sendTransform(tf::StampedTransform(odomAnchor, ros::Time::now(), "/map", "odom_combined"));		
			ros::Duration(0.01).sleep();
		}
	}
}

void ArTagLocalization::tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
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
	//ROS_INFO("Broadcasting transform!");
	br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), markerFrameID, "target_frame"));
	//ROS_INFO("Transform broadcasted!");
	
	//Listen for transform from map to robot
	//ROS_INFO("Listening for transform!");
	tf::StampedTransform mapTransform;
	try{
		listener.waitForTransform("map", "target_frame",
			ros::Time(0), ros::Duration(0.1));
		listener.lookupTransform("map", "target_frame",
			ros::Time(0), mapTransform);
		//ROS_INFO("Transform heard");	
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("Error in ARTagLocalization tag callback: %s",ex.what());
		return;
	}
	
	if (isFirst){
		isFirst = false;
		odomAnchor = *(new tf::Transform(mapTransform.getRotation(), mapTransform.getOrigin()));
	}
	br.sendTransform(tf::StampedTransform(odomAnchor, ros::Time::now(), "/map", odomFrame));
	br.sendTransform(tf::StampedTransform(odomAnchor, ros::Time::now(), "/map", "odom_combined"));

	//Unpack position and orientation
	double map_x, map_y, map_z;
	map_x = mapTransform.getOrigin().x();
	map_y = mapTransform.getOrigin().y();
	map_z = mapTransform.getOrigin().z();
	
	if (map_x > 4 || map_y > 4){
		return;
	}

	geometry_msgs::Quaternion quat;
	tf::quaternionTFToMsg(mapTransform.getRotation(), quat);

	//Publish pose as odometry message
	nav_msgs::Odometry odom_msg;
	odom_msg.header.frame_id = "map";
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.child_frame_id = "map";
	odom_msg.pose.pose = getOutputPose(map_x, map_y, map_z, quat);
	/*
	odom_msg.pose.pose.position.x = map_x;
	odom_msg.pose.pose.position.y = map_y;
	odom_msg.pose.pose.position.z = map_z;
	odom_msg.pose.pose.orientation = quat;
	*/
	
	for (int i = 0; i < 6; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			if(i == j)
			{
				odom_msg.pose.covariance[i*6 + j] = 1;
			}
			else
			{
				odom_msg.pose.covariance[i*6 + j] = 0.0;
			}
			odom_msg.twist.covariance[i*6 + j] = 1000000000;
		}
	}

	odomPub.publish(odom_msg);
}

geometry_msgs::Pose ArTagLocalization::getOutputPose(double x, double y, double z, geometry_msgs::Quaternion q)
{
	geometry_msgs::Pose output = geometry_msgs::Pose();
	
	output.position.x = x;
	output.position.y = y;
	output.position.z = z;
	output.orientation = q;
	
	geometry_msgs::Pose last = lastPoses.front();
	
	output.position.x = std::max(last.position.x - 1, std::min(last.position.x + 1, output.position.x));
	output.position.y = std::max(last.position.y - 1, std::min(last.position.y + 1, output.position.y));
	output.position.z = std::max(last.position.z - .25, std::min(last.position.z + .25, output.position.z));

	if(lastPoses.size() >= 5)
	{
		lastPoses.pop_back();
	}
	
	lastPoses.push_front(output);
	
	output.position.x = 0;
	output.position.y = 0;
	output.position.z = 0;
	int poses = lastPoses.size();
	for(int i = 0; i < lastPoses.size(); i++)
	{
		output.position.x += lastPoses[i].position.x * (1.0/poses);
		output.position.y += lastPoses[i].position.y * (1.0/poses);
		output.position.z += lastPoses[i].position.z * (1.0/poses);
	}
	
	return output;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ar_tag_localization");

	ArTagLocalization* loc = new ArTagLocalization();
	return 0;
}
