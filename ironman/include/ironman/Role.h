/**************************************************************************
 * 						HSMRS Framework - Role.h						***
 * 																		***
 * The Role class represents a set of behaviors an Agent should exhibit	***
 * when not assigned a specific Task. The behaviors are represented as	***
 * Tasks.																***
 *************************************************************************/

#pragma once
#include <vector>

#include "Task.h"

class Role
{
public:
	/**
	 * The constructor for the Role class
	 */
	Role(void);

	/**
	 * The destructor for the Role class
	 */
	~Role(void);

	/**
	 * Adds a Task to the Role
	 */
	virtual void addTask(Task* task, Role* role) = 0;
	virtual void removeTask(Task* task) = 0;
	virtual std::vector<Task*> getTasks() = 0;
};

