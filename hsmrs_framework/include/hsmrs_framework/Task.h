/**************************************************************************
 * 						HSMRS Framework - Task.h						***
 * 																		***
 * The Task class represents a set of actions Agents that have claimed	***
 * it should perform.													***
 *************************************************************************/

#ifndef _TASK_H_
 #define _TASK_H_

#include <string>
#include <vector>
#include <map>

class Task{
public:
	/**
	 * The constructor for the Task class
	 */
	Task(void){}

	/**
	 * The destructor for the Task class
	 */
	~Task(void){}
	
	virtual std::string getType() = 0;
	
	/**
	 * Retrieves the type of this Task.
	 * @return A string containing the type of this Task.
	 */
	virtual std::string getType() = 0;

	/**
	 * Retrieves a vector of Agents who have claimed this Task
	 * @return A vector of Agents who have claimed this Task
	 */
	virtual std::vector<std::string> getOwners() = 0;

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
	virtual std::map<std::string, double> getAttributeWeights() = 0;

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
	virtual void addOwner(std::string agent) = 0;

	/**
	 * Removes the given Agent as an owner.
	 * @param agent The name of the Agent to be removed as an owner.
	 */
	 
	virtual void removeOwner(std::string name) = 0;

	virtual double getPriority() = 0;

	virtual void setPriority(double p) = 0;

	virtual bool isReady() = 0;

	virtual int getID() = 0;

	virtual void setProgress(double val) = 0;
};

#endif
