/**************************************************************************
 * 						HSMRS Framework - Role.h						***
 * 																		***
 * The Role class represents a set of behaviors an Agent should exhibit	***
 * when not assigned a specific Task. The behaviors are represented as	***
 * Tasks.																***
 *************************************************************************/

#pragma once
#include <vector>
#include <string>

class Role
{
public:
	/**
	 * The constructor for the Role class
	 */
	Role(void){}

	/**
	 * The destructor for the Role class
	 */
	~Role(void){}

	/**
	 * Adds a Task to the Role
	 */
	virtual void addTask(std::string task) = 0;

	/**
	* Removes a Task from the Role
	*/
	virtual void removeTask(std::string task) = 0;

	/**
	* Returns all Tasks associated with this Role
	*/
	virtual std::vector<std::string> getTasks() = 0;
};

