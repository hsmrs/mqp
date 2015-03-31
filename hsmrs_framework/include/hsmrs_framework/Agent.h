/**************************************************************************
 * 						HSMRS Framework - Agent.h						***
 * 																		***
 * 	This class represents an Agent. An Agent performs Tasks it is		***
 * 	assigned. 															***
 *************************************************************************/
#pragma once

#include <string>
#include "Task.h"

class Agent
{
public:

	/**
	 * The constructor for the Agent object.
	 */
	Agent(void){}

	/**
	 * The destructor for the Agent object.
	 */
	~Agent(void){}

	/**
	 * Stops execution of the current Task and requests that
	 * the Task be returned to the TaskList.
	 */
	virtual void cancelTask() = 0;

	/**
	 * Asks the Agent to claim a task pointed to by \p task.
	 * @param task A pointer to the task object to be claimed.
	 */
	virtual void claimTask(Task* task) = 0;

	virtual std::string getName() = 0;
};
