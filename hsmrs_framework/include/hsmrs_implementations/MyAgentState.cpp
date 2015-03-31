#include "../include/hsmrs_framework/AgentState.h"

class MyAgentState : public AgentState
{
private:
	std::map<std::string, double> state;
public:
	MyAgentState()
	{
		state = std::map<std::string, double>();
		state["attr1"] = 1.;
		state["attr2"] = 2.;
	}
	
	MyAgentState(const AgentState& other)
	{
	    this->state = other.getAttributes();
	}

	double getAttribute(std::string attr)
	{
		return state.at(attr);
	}
	
	void setAttribute(std::string attr, double val)
	{
	    state[attr] = val;
	}

	std::map<std::string, double> getAttributes() const
	{
		return std::map<std::string, double>(state);
	}
};
