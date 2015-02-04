
#ifndef _GO_TO_BEHAVIOR_H_
#define _GO_TO_BEHAVIOR_H_

#include "ironman/Behavior.h"
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Pose.h"
#include "ironman/ironman_node.h"
//class Hulk;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoToBehavior : public Behavior{

private:
	
	Hulk* parent;
	MoveBaseClient* ac;
	bool isExecuting;
	move_base_msgs::MoveBaseGoal goalMsg;

	void goalCallback(const actionlib::SimpleClientGoalState& state,
              			const move_base_msgs::MoveBaseResult::ConstPtr& result);

	void feebackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr &feedback);

public:

	GoToBehavior(IronMan* parent, geometry_msgs::Pose goal, ros::NodeHandle n);

	virtual void execute();

	virtual void resume();

	virtual void pause();

	virtual void stop();
};


#endif