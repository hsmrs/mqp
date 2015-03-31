/**************************************************************************
 * 						HSMRS Framework - Progress.h					***
 * 																		***
 * The Progress class represents an abstract progress which can have	***
 * reports on the state of the progress									***
 *************************************************************************/

#pragma once
class Progress
{
public:
	/**
	 * The constructor for the Progress class
	 */
	Progress(void){}

	/**
	 * The destructor for the Progress class
	 */
	~Progress(void){}

	/**
	 * Determines the state of the Progress and returns it as
	 * a decimal percent.
	 * @return The state of the Progress as a decimal percent.
	 */
	virtual double report() = 0;
};

