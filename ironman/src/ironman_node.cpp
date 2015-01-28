#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

#include "ironman/Robot.h"

class IronMan: public Robot {

private:
	ros::Publisher registration_pub;
	ros::Publisher log_pub;
	ros::Publisher status_pub;
	ros::Publisher help_pub;
	ros::Publisher pose_pub;
	/**
	 * Makes this Robot bid on the given task
	 */
	void bid(Task* task) {

	}

	/**
	 * Begins the Robot's execution of its current Task.
	 */
	virtual void executeTask() {

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

public:
	IronMan() {
		ros::NodeHandle n;

		registration_pub = n.advertise<std_msgs::String>(
				"hsmrs/robot_registration", 100);
		log_pub = n.advertise<std_msgs::String>(
				"ironman/log_messages", 100);
		status_pub = n.advertise<std_msgs::String>(
				"ironman/status", 100);
		help_pub = n.advertise<std_msgs::String>("ironman/help",
				100);

		pose_pub = n.advertise<geometry_msgs::PoseStamped>(
				"ironman/pose", 100);

		ros::spinOnce();
		ros::Rate loop_rate(1);
		loop_rate.sleep();

		std_msgs::String msg;

		std::stringstream ss;

		//name;logTopic;imageTopic;poseTopic;statusTopic;helpTopic
		std::string name = "Iron Man";
		std::string logTopic = "ironman/log_messages";
		std::string imageTopic = "ironman/camera";
		std::string poseTopic = "ironman/pose";
		std::string statusTopic = "ironman/status";
		std::string helpTopic = "ironman/help";

		ss << name << ";" << logTopic << ";" << imageTopic << ";" << poseTopic
				<< ";" << statusTopic << ";" << helpTopic;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		int i = 0;

		while (ros::ok()) {
			if (i == 0) {
				registration_pub.publish(msg);
				i++;
			} else if (i == 1) {
				msg.data = "This is my first log!";
				log_pub.publish(msg);
				i++;
			} else if (i == 2) {
				msg.data = "Happy";
				status_pub.publish(msg);
				i++;
			} else if (i == 3) {
				callForHelp();
				i++;
			}
			/*
			 else if (i == 4){
			 registration_pub.publish(msg);
			 i++;
			 }
			 else if (i == 5){
			 registration_pub.publish(msg);
			 i++;
			 }*/
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

	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "iron_man");

	IronMan* robot = new IronMan();
	return 0;
}
