/**************************************************************************
 * 					HSMRS Framework - TaskList.h						***
 * 																		***
 * The TaskList class represents a container which holds, sorts, and	***
 * provides Task objects when needed.									***
 *************************************************************************/

#pragma once

#include <vector>

#include "Task.h"

class TaskList
{
public:
	/**
	 * The constructor for the TaskList class
	 */
	TaskList(void){}

	/**
	 * The destructor for the TaskList class
	 */
	~TaskList(void){}

	/**
	 * Adds a Task to the TaskList
	 * @param task The Task to be added
	 */
	virtual void addTask(Task* task) = 0;

	/**
		 * Retrieves the Tasks in the TaskList
		 * @return A vector of Tasks contained in this TaskList
		 */
	virtual std::vector<Task*> getTasks() = 0;

	/**
	 * Determines if the TaskList is empty
	 * @return True if the TaskList is empty
	 */
	virtual bool isEmpty() = 0;

	/**
	 * Retrieves the next task from the TaskList. The
	 * next task is one with all of its Prerequisites
	 * fulfilled and the highest priority.
	 * @return The next Task
	 */
	virtual Task* pullNextTask() = 0;

	virtual Task* getTask(int id) = 0;

/**
	 * Removes the given task from the TaskList.
	 * @return The removed Task
	 */
	virtual Task* removeTask(int id) = 0;

	/**
	 * Sets the priority of the given Task to the given priority
	 * @param task The Task whose priority will be changed
	 * @param priority The new priority for the task.
	 */
	virtual void setPriority(int task, double priority) = 0;
private:
	/**
	 * Sorts the TaskList in order of decreasing priority
	 */
	virtual void sortByPriority() = 0;
};

