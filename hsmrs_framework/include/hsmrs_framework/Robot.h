/**************************************************************************
 * 						HSMRS Framework - Robot.h						***
 * 																		***
 * The Robot class represents a robotic agent of the system. Robots are ***
 * capable of performing tasks, auctioning tasks, and communicating 	***
 * with other agents.													***
 *************************************************************************/

#pragma once
#include <string>

#include "AgentState.h"
#include "Task.h"
#include "Agent.h"
#include <hsmrs_framework/BidMsg.h>

class Robot: public Agent
{
public:
	/**
	 * The constructor for the Robot object.
	 */
	Robot(void){}

	/**
	 * The destructor for the Robot object.
	 */
	~Robot(void){}

	/**
	* Returns the value of the specified attribute from this Robot's AgentState.
	*/
	virtual double getAttribute(std::string) = 0;

	/**
	* Returns this Robot's utility for the specified Task.
	*/
	virtual double getUtility(Task *task) = 0;

	/**
	* Returns this Robot's AgentState.
	*/
	virtual AgentState* getState() = 0;

	/**
	* Checks if this Robot has the given attribute.
	*/
	virtual bool hasAttribute(std::string attr) = 0;

	/**
	* Sets this Robot's currently acctive Task.
	*/
	virtual void setTask(Task* task) = 0;

	//callbacks
	/**
	* Handles the auctioning of Tasks by sending and receiving bids.
	*/
	virtual void handleBids(const hsmrs_framework::BidMsg::ConstPtr& msg) = 0;

	/**
	* Verifies that an Agent claiming a Task has the highest utility for it. If not, informs the Agent of the Task's proper owner.
	*/
	virtual void verifyTaskClaim() = 0;

	virtual void callForHelp() = 0;

	virtual void sendMessage(std::string message) = 0;
private:
	/**
	 * Makes this Robot bid on the given task
	 */
	virtual double bid(const hsmrs_framework::BidMsg::ConstPtr& msg) = 0;

	virtual void executeTask() = 0;

	virtual void handleTeleop() = 0;

	virtual void requestTaskForQueue(Task* task) = 0;
};

