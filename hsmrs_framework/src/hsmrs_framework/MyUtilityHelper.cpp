#include "hsmrs_implementations/MyUtilityHelper.h"

	MyUtilityHelper::MyUtilityHelper()
	{
	}

	double MyUtilityHelper::calculate(Agent* agent, Task* task)
	{
		if(typeid(*agent) == typeid(Human))
		{
			return 0;
		}
		else if(typeid(*agent) == typeid(Robot))
		{
			double utility = 0;
			Robot* robot = dynamic_cast<Robot*>(agent);
			std::map<std::string, double> robotAttributes = robot->getState()->getAttributes();
			std::map<std::string, double> taskAttributes = *(task->getAttributeWeights());
			for(std::map<std::string, double>::iterator i = robotAttributes.begin(); i != robotAttributes.end(); ++i)
			{
				if(taskAttributes.count(i->first) == 1)
				{
					utility += taskAttributes[i->first]*i->second;
				}
			}

			return utility;
		}
		else
		{
			throw;
		}
	}

	MyUtilityHelper::~MyUtilityHelper()
	{
	}