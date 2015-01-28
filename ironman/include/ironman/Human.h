/**************************************************************************
 * 						HSMRS Framework - Human.h						***
 * 																		***
 * The Human class represents the human supervisor of the system and 	***
 * their user interface. The Human is capable of manipulating the 		***
 * TaskList, controlling robots, and sending and receiving messages		***
 * from all parts of the system.										***
 *************************************************************************/

#pragma once

#include "Task.h"
#include "Role.h"
#include "Robot.h"
#include "Agent.h"

class Human: public Agent
{
public:
	/**
	 * The constructor for the Human object.
	 */
	Human(void);

	/**
	 * The destructor for the Human object.
	 */
	~Human(void);

	/*
	 * Handles a request from a Robot to have a task added to the
	 * TaskList. Operates as a service for Robots to call.
	 * @param task A pointer to the requested task.
	 */
	virtual void handleTaskRequest(Task* task) = 0;

	/**
	 * Handles a request from a Robot to have the Human's focus
	 * set to that Robot.
	 * @param task A pointer to the Robot requesting help.
	 */
	virtual void handleHelpCall(Robot* source) = 0;

private:
	/**
	 * Assigns a given Task to a given Agent
	 * @param task The task to be assigned
	 * @param agent The Agent to be assigned \a Task
	 */
	virtual void assignTask(Task* task, Agent* agent) = 0;

	/**
	 * Remove the current task from the provided Agent.
	 * @param A pointer to the Agent whose Task is to be removed.
	 */
	virtual void unassignTask(Agent* agent);

	/**
	 * Assigns a given Role to a given Agent
	 * @param role A pointer to the Role to be assigned
	 * @param agent A pointer to the Agent to be assigned \a role
	 */
	virtual void assignRole(Role* role, Agent* agent) = 0;

	/**
	 * Place a Task on all of the Agent's TaskLists to be queued
	 * and auctioned.
	 * @param task A pointer to the Task to be queued
	 */
	virtual void queueTask(Task* task) = 0;

	/**
	 * Remove a Task from all of the Agent's TaskLists.
	 * @param task A pointer to the Task to be removed.
	 */
	virtual void deleteTask(Task* task) = 0;
};

