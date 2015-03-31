/**************************************************************************
 * 					HSMRS Framework - UtilityHelper.h					***
 * 																		***
 * The UtilityHelper class represents an object whose sole purpose is	***
 * to calculate the Utility of an Agent									***
 *************************************************************************/

#pragma once

#include "Agent.h"
#include "Task.h"

class UtilityHelper
{
public:
	/**
	 * This is the constructor for the UtilityHelper class
	 */
	UtilityHelper(void){}

	/**
	 * This is the destructor for the UtilityHelper class
	 */
	~UtilityHelper(void){}

	/**
	 * Calculates the utility of the given Robot for the given
	 * task.
	 * @param robot The Robot whose utility is being calculated
	 * @param task The Task used to determine the utility.
	 */
	virtual double calculate(Robot* robot, Task* task) = 0;
};
