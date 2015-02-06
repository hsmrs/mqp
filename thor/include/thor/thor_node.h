#ifndef _THOR_NODE_H_
#define _THOR_NODE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "kobuki_msgs/BumperEvent.h"

#include <sstream>
#include <typeinfo>

#include "hsmrs_framework/Robot.h"
#include "hsmrs_framework/TaskList.h"
#include "hsmrs_implementations/GoToTask.h"
#include "hsmrs_implementations/FollowTagTask.h"
#include "hsmrs_implementations/MyTaskList.h"
#include "thor/Behavior.h"
#include "thor/FollowTagBehavior.h"
#include "thor/GoToBehavior.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Thor: public Robot {

private:
	friend class FollowTagBehavior;
	friend class GoToBehavior;

	ros::NodeHandle n;
	ros::Publisher registration_pub;
	ros::Publisher log_pub;
	ros::Publisher status_pub;
	ros::Publisher help_pub;
	ros::Publisher pose_pub;

	ros::Subscriber request_sub;
	ros::Subscriber teleOp_sub;
	ros::Subscriber new_task_sub;
	ros::Subscriber updated_task_sub;


	ros::Publisher vel_pub;
	ros::Subscriber bumper_sub;
	ros::Subscriber laser_sub;


	const std::string NAME;
	const std::string REGISTRATION_TOPIC;
	const std::string IMAGE_TOPIC;
	const std::string LOG_TOPIC;
	const std::string STATUS_TOPIC;
	const std::string HELP_TOPIC;
	const std::string POSE_TOPIC;
	const std::string REQUEST_TOPIC;
	const std::string TELE_OP_TOPIC;
	const std::string NEW_TASK_TOPIC;
	const std::string UPDATED_TASK_TOPIC;

	const std::string VEL_TOPIC;
	const std::string BUMPER_TOPIC;
	const std::string LASER_TOPIC;

	TaskList* taskList;
	Task* p_currentTask;
	std::string status;
	Behavior* p_currentBehavior;
	double linearSpeed;
	double angularSpeed;

	/**
	 * Makes this Robot bid on the given task
	 */
	void bid(Task* task);

	/**
	 * Begins the Robot's execution of its current Task.
	 */
	virtual void executeTask();

	virtual void pauseTask();

	virtual void resumeTask();

	void doGoToTask(double x, double y);

	/**
	 * Request for the given Task to be sent to the TaskList
	 * @param task The task to be queued
	 */
	virtual void requestTaskForQueue(Task* task);

	/**
	 * Send a help request to the Human supervisor.
	 */
	virtual void callForHelp();

	virtual void handleTeleop();

	void registerWithGUI();

	void sendMessage(std::string message);

	void requestCallback(const std_msgs::String::ConstPtr& msg);

	void teleOpCallback(const geometry_msgs::Twist::ConstPtr& msg);

	void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

	void newTaskCallback(const std_msgs::String::ConstPtr& msg);

	void updatedTaskCallback(const std_msgs::String::ConstPtr& msg);

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

public:
	Thor();

	virtual std::string getName();


	/**
	 * Returns the value of the specified attribute from this Robot's AgentState.
	 * @param attr The name of the attribute to get
	 * @return The value of the attribute
	 */
	virtual double getAttribute(std::string attr);

	/**
	 * Returns this Robot's utility for the specified Task.
	 * @param task A pointer to the task for which to get a utility.
	 * @return This Robot's utility for the given Task.
	 */
	virtual double getUtility(Task *task);

	/**
	 * Returns this Robot's AgentState.
	 * @return The AgentState representing the state of this Robot.
	 */
	virtual AgentState* getState();

	/**
	 * Checks if this Robot has the given attribute.
	 * @param attr The name of the target attribute
	 * @return True if the robot has the named attribute.
	 */
	virtual bool hasAttribute(std::string attr);

	/**
	 * Sets this Robot's currently active Task.
	 * @param A pointer to the Task to be set
	 */
	virtual void setTask(Task* task);

	/**
	 * Handles the auctioning of Tasks by sending and receiving bids.
	 */
	virtual void handleBids();

	/**
	 * Verifies that an Agent claiming a Task has the highest utility for it. If not, informs the Agent of the Task's proper owner.
	 */
	virtual void verifyTaskClaim();

	/**
	 * Stops execution of the current Task and requests that
	 * the Task be returned to the TaskList.
	 */
	virtual void cancelTask();

	/**
	 * Asks the Agent to claim a task pointed to by \p task.
	 * @param task A pointer to the task object to be claimed.
	 */
	virtual void claimTask(Task* task);

	virtual std::string getStatus();

	virtual void setStatus(std::string newStatus);

};

#endif