/**************************************************************************
 * 					HSMRS Framework - AttributeWeights.h				***
 * 																		***
 * 	This class represents an AttributeWeights is a container that		***
 * 	tracks the importance of each attribute (in a set of attributes) to	***
 * 	completing a certain Task. These weights are used in conjunction	***
 * 	with an AgentState to calculate an Agent’s utility for a Task.		***											***
 *************************************************************************/

#pragma once
#include <string>
#include <map>

class AttributeWeights
{
public:
	/**
	 * The constructor for the AttributeWeights object.
	 */
	AttributeWeights(void){}

	/**
	 * The destructor for the AttributeWeights object.
	 */
	~AttributeWeights(void){}

	/**
	 * Returns the weight of the attribute with the given name.
	 * @param name The attribute to get the weight of.
	 * @return The weight of the attribute with the name \a name.
	 */
	virtual double getWeight(std::string name) = 0;

	/**
	 * Returns a map of all contained attributes and weights.
	 * @return A map, with keys being attribute names and values
	 * 			being their respective weights.
	 */
	virtual std::map<std::string, double> getWeights() = 0;
};
