/**************************************************************************
 * 						HSMRS Framework - AgentState.h					***
 * 																		***
 * 	This class represents an AgentState. The AgentState	is used to		***
 * 	keep track of certain important	qualities of an Agent, called		***
 * 	attributes.															***
 *************************************************************************/

#pragma once
#include <string>
#include <map>

class AgentState
{
public:
	/**
	 * The constructor for the AgentState object.
	 */
	AgentState(void){}

	/**
	 * The destructor for the AgentState object.
	 */
	~AgentState(void){}

	/**
	 * Returns the value of the given attribute.
	 * @param name The name of the atttribute
	 * @return The value of the attribute with the name \a name
	 */
	virtual double getAttribute(std::string name) = 0;
	
	virtual void setAttribute(std::string attr, double) = 0;

	/**
	 * Returns the mapping of attributes to values.
	 * @return The mapping of attributes to values.
	 */
	virtual std::map<std::string, double> getAttributes() const = 0;
};
