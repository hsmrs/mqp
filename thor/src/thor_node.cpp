#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "hsmrs_framework/BidMsg.h"
#include "hsmrs_framework/TaskMsg.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sstream>

#include "hsmrs_framework/Robot.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Thor: public Robot {

private:
	ros::Publisher registration_pub;
	ros::Publisher log_pub;
	ros::Publisher status_pub;
	ros::Publisher help_pub;
	ros::Publisher pose_pub;

	const std::string NAME;
	const std::string REGISTRATION_TOPIC;
	const std::string IMAGE_TOPIC;
	const std::string LOG_TOPIC;
	const std::string STATUS_TOPIC;
	const std::string HELP_TOPIC;
	const std::string POSE_TOPIC;

	Task* currentTask;

	/**
	 * Makes this Robot bid on the given task
	 */
	void bid(Task* task) {

	}

	/**
	 * Begins the Robot's execution of its current Task.
	 */
	virtual void executeTask() {
		std::string taskType = currentTask->getType();

		if (taskType == "Go to"){
			//This needs to be changed once tasks have been parameterized
			doGoToTask(10, 10);
		}
	}

	void doGoToTask(double x, double y){
		MoveBaseClient ac("move_base", true);

  		//wait for the action server to come up
  		while(!ac.waitForServer(ros::Duration(5.0))){
   			ROS_INFO("Waiting for the move_base action server to come up");
 		}

  		move_base_msgs::MoveBaseGoal goal;

  		goal.target_pose.header.frame_id = "map";
  		goal.target_pose.header.stamp = ros::Time::now();

  		goal.target_pose.pose.position.x = x;
  		goal.target_pose.pose.position.y = y;

 		ROS_INFO("Sending goal");
  		ac.sendGoal(goal);

  		ac.waitForResult();

  		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    		ROS_INFO("Go To task completed");
    		sendLog("Go To task completed");
    	}
  		else
  		{
    		ROS_INFO("Go To task failed, asking for helps");
    		sendLog("Go To task failed, asking for helps");
    		callForHelp();
    	}
	}

	/**
	 * Request for the given Task to be sent to the TaskList
	 * @param task The task to be queued
	 */
	virtual void requestTaskForQueue(Task* task) {

	}

	/**
	 * Send a help request to the Human supervisor.
	 */
	virtual void callForHelp() {
		std_msgs::String msg;
		msg.data = "true";
		help_pub.publish(msg);
	}

	void registerWithGUI() {
		//name;logTopic;imageTopic;poseTopic;statusTopic;helpTopic

		std_msgs::String msg;
		std::stringstream ss;

		ss << NAME << ";" << LOG_TOPIC << ";" << IMAGE_TOPIC << ";"
				<< POSE_TOPIC << ";" << STATUS_TOPIC << ";" << HELP_TOPIC;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
	}

	void sendLog(std::string logMessage){
		std_msgs::String msg;
		msg.data = logMessage;
		log_pub.publish(logMessage);
	}

public:
	Thor() : NAME("Thor"), REGISTRATION_TOPIC("hsmrs/robot_registration"), IMAGE_TOPIC("thor/camera"), 
			LOG_TOPIC("thor/log_messages"), STATUS_TOPIC("thor/status"), HELP_TOPIC("thor/help"), POSE_TOPIC("thor/pose")
			{
		ros::NodeHandle n;

		registration_pub = n.advertise<std_msgs::String>(REGISTRATION_TOPIC,
				100);
		log_pub = n.advertise<std_msgs::String>(LOG_TOPIC, 100);
		status_pub = n.advertise<std_msgs::String>(STATUS_TOPIC, 100);
		help_pub = n.advertise<std_msgs::String>(HELP_TOPIC, 100);

		pose_pub = n.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 100);

		ros::spinOnce();
		ros::Rate loop_rate(1);
		loop_rate.sleep();

		registerWithGUI();

		int i = 0;

		std_msgs::String msg;
		while (ros::ok()) {
			loop_rate.sleep();
		}
	}
	/**
	 * Returns the value of the specified attribute from this Robot's AgentState.
	 * @param attr The name of the attribute to get
	 * @return The value of the attribute
	 */
	virtual double getAttribute(std::string attr) {

	}

	/**
	 * Returns this Robot's utility for the specified Task.
	 * @param task A pointer to the task for which to get a utility.
	 * @return This Robot's utility for the given Task.
	 */
	virtual double getUtility(Task *task) {

	}

	/**
	 * Returns this Robot's AgentState.
	 * @return The AgentState representing the state of this Robot.
	 */
	virtual AgentState* getState() {

	}

	/**
	 * Checks if this Robot has the given attribute.
	 * @param attr The name of the target attribute
	 * @return True if the robot has the named attribute.
	 */
	virtual bool hasAttribute(std::string attr) {

	}

	/**
	 * Sets this Robot's currently active Task.
	 * @param A pointer to the Task to be set
	 */
	virtual void setTask(Task* task) {
		currentTask = task;
	}

	/**
	 * Handles the auctioning of Tasks by sending and receiving bids.
	 */
	virtual void handleBids() {

	}

	/**
	 * Verifies that an Agent claiming a Task has the highest utility for it. If not, informs the Agent of the Task's proper owner.
	 */
	virtual void verifyTaskClaim() {

	}

	/**
	 * Stops execution of the current Task and requests that
	 * the Task be returned to the TaskList.
	 */
	virtual void cancelTask() {

	}

	/**
	 * Asks the Agent to claim a task pointed to by \p task.
	 * @param task A pointer to the task object to be claimed.
	 */
	virtual void claimTask(Task* task) {
		currentTask = task;
	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "thor");

	Thor* robot = new Thor();
	return 0;
}
