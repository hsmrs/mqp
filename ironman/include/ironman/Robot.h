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
	* @param attr The name of the attribute to get
	* @return The value of the attribute
	*/
	virtual double getAttribute(std::string attr) = 0;

	/**
	* Returns this Robot's utility for the specified Task.
	* @param task A pointer to the task for which to get a utility.
	* @return This Robot's utility for the given Task.
	*/
	virtual double getUtility(Task *task) = 0;

	/**
	* Returns this Robot's AgentState.
	* @return The AgentState representing the state of this Robot.
	*/
	virtual AgentState* getState() = 0;

	/**
	* Checks if this Robot has the given attribute.
	* @param attr The name of the target attribute
	* @return True if the robot has the named attribute.
	*/
	virtual bool hasAttribute(std::string attr) = 0;

	/**
	* Sets this Robot's currently active Task.
	* @param A pointer to the Task to be set
	*/
	virtual void setTask(Task* task) = 0;

	/**
	* Handles the auctioning of Tasks by sending and receiving bids.
	*/
	virtual void handleBids() = 0;

	/**
	* Verifies that an Agent claiming a Task has the highest utility for it. If not, informs the Agent of the Task's proper owner.
	*/
	virtual void verifyTaskClaim() = 0;
private:
	/**
	 * Makes this Robot bid on the given task
	 */
	virtual void bid(Task* task) = 0;

	/**
	 * Begins the Robot's execution of its current Task.
	 */
	virtual void executeTask() = 0;

	/**
	 * Request for the given Task to be sent to the TaskList
	 * @param task The task to be queued
	 */
	virtual void requestTaskForQueue(Task* task) = 0;

	/**
	 * Send a help request to the Human supervisor.
	 */
	virtual void callForHelp() = 0;
};

