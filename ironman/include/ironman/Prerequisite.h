/**************************************************************************
 * 					HSMRS Framework - Prerequisite.h					***
 * 																		***
 * The Prerequisite class represents the an abstract prerequisite that	***
 * can be fulfilled or not												***
 *************************************************************************/

#pragma once
class Prerequisite
{
public:
	/**
	 * The constructor for the Prerequisite class
	 */
	Prerequisite(void);

	/**
	 * The destructor for the Prerequisite class
	 */
	~Prerequisite(void);

	/**
	 * Determines if this Prerequisite has been fulfilled.
	 * @return True if the Prerequiste has been fulfilled.
	 */
	virtual bool isFulfilled() = 0;
};

