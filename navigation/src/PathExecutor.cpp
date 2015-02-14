#include "navigation/PathExecutor.h"

PlanExecutor::PlanExecutor(){
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();
		
	std::string poseTopic;
	std::string velTopic;
	nh.param<std::string>("pose_topic", poseTopic, "pose");
	nh.param<std::string>("vel_topic", velTopic, "cmd_vel");
	nh.param<double>("max_linear_velocity", maxLinearVel, 1.0);
	nh.param<double>("max_angular_velocity", maxAngularVel, 1.0);

	velPub = nh.advertise<geometry_msgs::Twist>(velTopic, 1000);
	poseSub = nh.subscribe(poseTopic, 1000, &PlanExecutor::poseCallback, this);
	pathSub = nh.subscribe("navigation/path", 1000, &PlanExecutor::pathCallback, this);
	cancelSub = nh.subscribe("navigation/cancel", 1000, &PlanExecutor::cancelCallback, this);
}

void PlanExecutor::poseCallback(const geometry_msgs::Pose::ConstPtr& poseMsg){
	currentPose = *poseMsg;
}

void PlanExecutor::pathCallback(const nav_msgs::Path::ConstPtr& pathMsg){
	for (geometry_msgs::PoseStamped pose : pathMsg->poses){
		path.push_back(pose.pose);
	}

	isCanceled = false;
	executePath();
}

void PlanExecutor::cancelCallback(const std_msgs::String::ConstPtr cancelMsg){
	isCanceled = true;
}

void PlanExecutor::executePath(){
	double x1, x2, y1, y2, theta;
	double linearKp = 0.8, angularKp = 0.3;

	for (auto pose : path){
		x1 = currentPose.position.x;
		x2 = pose.position.x;
		y1 = currentPose.position.y;
		y2 = pose.position.y;
		while (distance(x1, y1, x2, y2) < 0.5 && !isCanceled){
			x1 = currentPose.position.x;
			y1 = currentPose.position.y;

			tf::Quaternion q;
			tf::quaternionMsgToTF(currentPose.orientation, q);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			theta = yaw;

			double linearError, angularError;
			double linearVelocity, angularVelocity;

			linearError = distance(x1, y1, x2, y2);
			angularError = atan2(y2-y1, x2-x1) - theta;

			if (abs(angularError) >= 45*M_PI/180){
				linearVelocity = 0;
				angularVelocity = angularError * angularKp;
			}
			else {
				linearVelocity = linearError * linearKp;
				angularVelocity = angularError * angularKp;
			}

			linearVelocity = std::min(linearVelocity, maxLinearVel);
			angularVelocity = std::min(angularVelocity, maxAngularVel);

			geometry_msgs::Twist velMsg;
			velMsg.linear.x = linearVelocity;
			velMsg.angular.z = angularVelocity;

			velPub.publish(velMsg);
		}

		if (isCanceled) break;
	}
}

double PlanExecutor::distance(double x1, double y1, double x2, double y2){
	sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "path_executor");

	PlanExecutor* executor = new PlanExecutor();
	return 0;
}