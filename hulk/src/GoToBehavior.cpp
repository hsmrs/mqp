#include "hulk/GoToBehavior.h"

GoToBehavior::GoToBehavior(Robot* parent, geometry_msgs::Pose goal, ros::NodeHandle n){
	this->parent = parent;
	ac = new MoveBaseClient("move_base", true);

	//wait for the action server to come up
	while(!ac->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

  	goalMsg.target_pose.header.frame_id = "map";
  	goalMsg.target_pose.header.stamp = ros::Time::now();

  	goalMsg.target_pose.pose = goal;
}

void GoToBehavior::goalCallback(const actionlib::SimpleClientGoalState& state,
          			const move_base_msgs::MoveBaseResult::ConstPtr& result) {
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Goal reached!");
	}
	else{
		ROS_INFO("Goal failed");
		parent->sendMessage("I was not able to go to my goal!");
		parent->callForHelp();
	}
}

void GoToBehavior::feebackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback){
	
}

void GoToBehavior::execute(){
	ROS_INFO("Sending goal");
  	ac->sendGoal(goalMsg,
                boost::bind(&GoToBehavior::goalCallback, this, _1, _2),
                MoveBaseClient::SimpleActiveCallback(),
                boost::bind(&GoToBehavior::feebackCallback, this, _1));
}

void GoToBehavior::resume(){

}

void GoToBehavior::pause(){

}

void GoToBehavior::stop(){

}