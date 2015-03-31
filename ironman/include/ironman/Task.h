/**************************************************************************
 * 						HSMRS Framework - Task.h						***
 * 																		***
 * The Task class represents a set of actions Agents that have claimed	***
 * it should perform.													***
 *************************************************************************/

#pragma once

#include <vector>
#include <map>

#include "Agent.h"

class Task
{
public:
	/**
	 * The constructor for the Task class
	 */
	Task(void);

	/**
	 * The destructor for the Task class
	 */
	~Task(void);

	/**
	 * Retrieves a vector of Agents who have claimed this Task
	 * @return A vector of Agents who have claimed this Task
	 */
	virtual std::vector<Agent*> getOwners() = 0;

	/**
	 * Determines the minimum number of Agents that need to claim this
	 * Task for the Task to be executed.
	 * @return The minimum number of owners for this Task.
	 */
	virtual int getMinOwners() = 0;

	/**
	 * Determines the maximum number of Agents that can claim this
	 * Task.
	 * @return The maximum number of owners for this task.
	 */
	virtual int getMaxOwners() = 0;

	/**
	 * Retrieves a mapping between Attributes and their weights.
	 * @return A map of strings and doubles
	 */
	virtual std::map<std::string, double> getAttirbuteWeights() = 0;

	/**
	 * Retrieves a vector of subtasks which need to be completed as part of
	 * this task
	 * @return A vector of Tasks which are a part of this Task.
	 */
	virtual std::vector<Task*> getSubtasks() = 0;

	/**
	 * Adds an Agent as an owner to this task, if another owner can be added
	 * @param agent The Agent to be added as an owner.
	 */
	virtual void addOwner(Agent* agent) = 0;

	/**
	 * Removes the given Agent as an owner.
	 * @param agent The Agent to be added as an owner.
	 */
	virtual void removeOwner(Agent* agent) = 0;
};

