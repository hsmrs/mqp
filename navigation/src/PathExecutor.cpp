#include "navigation/PathExecutor.h"
#include "ros/console.h"

PlanExecutor::PlanExecutor(){
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();
		
	std::string poseTopic;
	std::string velTopic;
//	nh.getParam("pose_topic", poseTopic);
//	nh.getParam("vel_topic", velTopic);
//	nh.getParam("max_linear_velocity", maxLinearVel);
//	nh.getParam("max_angular_velocity", maxAngularVel);
	ros::param::param<std::string>("~pose_topic", poseTopic, "pose");
	ros::param::param<std::string>("~vel_topic", velTopic, "cmd_vel");
	ros::param::param<double>("~max_linear_velocity", maxLinearVel, 1.0);
	ros::param::param<double>("~max_angular_velocity", maxAngularVel, 1.0);
	
	ROS_INFO("Listenening for pose on: %s", poseTopic.c_str());
	ROS_INFO("Publishing velocity on: %s", velTopic.c_str());
	ROS_INFO("Max linear vel: %f", maxLinearVel);
	ROS_INFO("Max angular vel: %f", maxAngularVel);

	velPub = nh.advertise<geometry_msgs::Twist>(velTopic, 1000);
	progressPub = nh.advertise<std_msgs::String>("navigation/progress", 1000);
	poseSub = nh.subscribe(poseTopic, 1000, &PlanExecutor::poseCallback, this);
	pathSub = nh.subscribe("navigation/path", 1000, &PlanExecutor::pathCallback, this);
	cancelSub = nh.subscribe("navigation/cancel", 1000, &PlanExecutor::cancelCallback, this);
	nextPointPub = nh.advertise<geometry_msgs::PointStamped>("navigation/next_point", 1000);
	
	ros::spin();
}

void PlanExecutor::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg){
	currentPose = poseMsg->pose;
}

void PlanExecutor::pathCallback(const nav_msgs::Path::ConstPtr& pathMsg){
	ROS_INFO("Path received!");
	path.clear();
	for (geometry_msgs::PoseStamped pose : pathMsg->poses){
		path.push_back(pose.pose);
	}

	isCanceled = false;
	ROS_INFO("Executing!");
	executePath();
}

void PlanExecutor::cancelCallback(const std_msgs::String::ConstPtr cancelMsg){
	isCanceled = true;
}

void PlanExecutor::executePath(){
	double x1, x2, y1, y2, theta;
	double linearKp = 0.8, angularKp = 1.2;

	for (auto pose : path){
		//ROS_INFO("Next waypoint!");
		x1 = currentPose.position.x;
		x2 = pose.position.x;
		y1 = currentPose.position.y;
		y2 = pose.position.y;
		geometry_msgs::PointStamped nextPointMsg;
		nextPointMsg.header.frame_id = "map";
		nextPointMsg.point.x = x2;
		nextPointMsg.point.y = y2;
		nextPointPub.publish(nextPointMsg);
		ROS_INFO("Driving to: (%f, %f)", x2, y2);
		//ROS_INFO("Distance: %f", distance(x1, y1, x2, y2));
		while (distance(x1, y1, x2, y2) > 0.5 && !isCanceled && ros::ok()){
			//ROS_INFO("Driving to: (%f, %f)", x2, y2);
			//ROS_INFO("Distance: %f", distance(x1, y1, x2, y2));
			x1 = currentPose.position.x;
			y1 = currentPose.position.y;

			tf::Quaternion q;
			tf::quaternionMsgToTF(currentPose.orientation, q);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			theta = yaw;
			//ROS_INFO("Orientation: %f", theta);

			double linearError, angularError;
			double linearVelocity, angularVelocity;

			linearError = distance(x1, y1, x2, y2);
			angularError = atan2(y2-y1, x2-x1) - theta;
			//ROS_INFO("Raw angular error: %f", angularError*180/M_PI);
			if (abs(angularError) > M_PI){
				int sign = -1*(angularError/abs(angularError));
				angularError = 2*M_PI - abs(angularError);
				angularError *= sign;
			}
			
			//ROS_INFO("My angle: %f", theta*180/M_PI);
			//ROS_INFO("Angle to goal: %f", atan2(y2-y1, x2-x1)*180/M_PI);
			//ROS_INFO("Angular error: %f", angularError*180/M_PI);

			if (abs(angularError) >= 10*M_PI/180){
				linearVelocity = 0;
				angularVelocity = angularError * angularKp;
				//ROS_INFO("Facing target");
			}
			else {
				linearVelocity = linearError * linearKp;
				angularVelocity = angularError * angularKp;
				//ROS_INFO("Driving to target");
			}

			linearVelocity = std::min(linearVelocity, maxLinearVel);
			angularVelocity = std::max(std::min(angularVelocity, maxAngularVel), -maxAngularVel);
			
			//ROS_INFO("Linear: %f", linearVelocity);
			//ROS_INFO("Angular: %f", angularVelocity);

			geometry_msgs::Twist velMsg;
			velMsg.linear.x = linearVelocity;
			velMsg.angular.z = angularVelocity;

			velPub.publish(velMsg);
			ros::Duration(0.1).sleep();
		}

		if (isCanceled) break;
	}
	if (!isCanceled){
		ROS_INFO("Plan Executed!");
		std_msgs::String progressMsg;
		progressMsg.data = "complete";
		progressPub.publish(progressMsg);
	} else {
		ROS_INFO("Plan Execution Canceled!");
		std_msgs::String progressMsg;
		progressMsg.data = "canceled";
		progressPub.publish(progressMsg);
	}
}

double PlanExecutor::distance(double x1, double y1, double x2, double y2){
	return sqrt( pow( (x2 - x1) , 2 ) + pow( (y2 - y1) , 2 ) );
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "path_executor");

	PlanExecutor* executor = new PlanExecutor();
	return 0;
}
