#ifndef _THOR_NODE_H_
#define _THOR_NODE_H_

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "kobuki_msgs/BumperEvent.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <sstream>
#include <typeinfo>
#include <mutex>

#include "hsmrs_framework/Robot.h"
#include "hsmrs_framework/Role.h"
#include "hsmrs_framework/TaskList.h"
#include "hsmrs_implementations/GoToTask.h"
#include "hsmrs_implementations/FollowTagTask.h"
#include "hsmrs_implementations/SearchTask.h"
#include "hsmrs_implementations/MyTaskList.h"
#include "hsmrs_implementations/MyUtilityHelper.h"
#include "hsmrs_implementations/MyTask.h"
#include "hsmrs_implementations/MyAgentState.cpp"
#include "hsmrs_implementations/MyRole.cpp"
#include "hsmrs_framework/TaskMsg.h"
#include "hsmrs_framework/RoleMsg.h"
#include "thor/Behavior.h"
#include "thor/FollowTagBehavior.h"
#include "thor/GoToBehavior.h"
#include "thor/SearchBehavior.h"

#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class AuctionTracker
{
private:

public:
    std::string topBidder;
    double topUtility;
    bool haveBidded;
    bool taskClaimed;
    int bidCount;
    
    AuctionTracker()
    {
        topBidder = "";
        topUtility = -1;
        haveBidded = false;
        taskClaimed = false;
        bidCount = 0;
    }
};

class Thor: public Robot {

private:
	friend class FollowTagBehavior;
	friend class GoToBehavior;

    //boost::mutex atMutex;
    //boost::mutex listMutex;
    //boost::mutex currentTaskMutex;

    std::mutex atMutex;
    std::mutex listMutex;
    std::mutex currentTaskMutex;
    std::mutex currentBehaviorMutex;

	ros::NodeHandle n;
	ros::Publisher registration_pub;
	ros::Publisher log_pub;
	ros::Publisher status_pub;
	ros::Publisher help_pub;
	ros::Publisher pose_pub;
	ros::Publisher updatedTaskPub;

	ros::Subscriber request_sub;
	ros::Subscriber teleOp_sub;
	ros::Subscriber updated_task_sub;
    ros::Subscriber tag_sub;
    
	ros::Publisher bidPub;
	ros::Publisher claimPub;
	
	ros::Subscriber bidSub;
	ros::Subscriber newTaskSub;
	ros::Subscriber claimSub;
	ros::Subscriber taskProgressSub;

	ros::Subscriber roleSub;

	ros::Publisher vel_pub;
	ros::Subscriber bumper_sub;
	ros::Subscriber laser_sub;
	ros::Subscriber pose_sub;

	tf::TransformBroadcaster br;
	tf::TransformListener listener;

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
	const std::string AUCTION_TOPIC;
	const std::string CLAIM_TOPIC;
	const std::string TASK_PROGRESS_TOPIC;
	const std::string ROLE_TOPIC;

	const std::string VEL_TOPIC;
	const std::string BUMPER_TOPIC;
	const std::string LASER_TOPIC;
	const std::string IN_POSE_TOPIC;
	const std::string MARKER_TOPIC;

	TaskList* taskList;
	Task* p_currentTask;
	UtilityHelper* utiHelp;
	AgentState* state;
	Role* p_currentRole;
		
	std::map<int, AuctionTracker> auctionList;
	
	std::string status;
	Behavior* p_currentBehavior;
	double linearSpeed;
	double angularSpeed;

	/**
	 * Makes this Robot bid on the given task
	 */
	double bid(const hsmrs_framework::BidMsg::ConstPtr& msg);
	
	double bid(const hsmrs_framework::TaskMsg::ConstPtr& msg);

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

	virtual void handleTeleop();

	void registerWithGUI();

	void requestCallback(const std_msgs::String::ConstPtr& msg);

	void teleOpCallback(const geometry_msgs::Twist::ConstPtr& msg);

	void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

	void updatedTaskCallback(const hsmrs_framework::TaskMsg::ConstPtr& msg);

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

	void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	
	void tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
	
	void handleNewTask(const hsmrs_framework::TaskMsg::ConstPtr& msg);
	
	void handleClaims(const hsmrs_framework::BidMsg::ConstPtr& msg);
	
	void claimWorker(hsmrs_framework::TaskMsg taskMsg, int id, double myBid);
	
	void handleProgress(const std_msgs::String::ConstPtr& msg);

	void handleRoleAssign(const hsmrs_framework::RoleMsg::ConstPtr& msg);

public:
	Thor(std::string name, double speed);

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
	virtual void handleBids(const hsmrs_framework::BidMsg::ConstPtr& msg);

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

	/**
	 * Send a help request to the Human supervisor.
	 */
	virtual void callForHelp();
	
	void sendMessage(std::string message);

};

#endif
