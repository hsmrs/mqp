#include "ros/ros.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "geometry_msgs/Twist.h"

ros::Publisher vel_pub;

void markerCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
	if (msg->markers.size() == 0) return;

	double linK = 0.4;
	double angK = 2;	
	double x = msg->markers[0].pose.pose.position.x;
	double y = msg->markers[0].pose.pose.position.y;
	double linVel = std::max(std::min(0.3, linK * x), 0.0);
	double angVel = std::max(std::min(0.5, angK * y), -0.5);

	if (x <= 1.0){
		linVel = 0;
	}

	geometry_msgs::Twist velMsg;
	velMsg.linear.x = linVel;
	velMsg.angular.z = angVel;
	vel_pub.publish(velMsg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "follow_tag");

	ros::NodeHandle n;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Subscriber marker_sub = n.subscribe("/ar_pose_marker", 1000, markerCallback);
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);

	ros::waitForShutdown();
	return 0;
}
