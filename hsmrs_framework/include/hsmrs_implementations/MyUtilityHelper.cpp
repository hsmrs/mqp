#include <hsmrs_framework/UtilityHelper.h>
#include <map>
#include <exception>

class MyUtilityHelper: public UtilityHelper
{
public:
	MyUtilityHelper()
	{
	}

	double calculate(Robot* robot, Task* task)
	{
		double utility = 0;
		std::map<std::string, double> robotAttributes = robot->getState()->getAttributes();
		std::map<std::string, double> taskAttributes = task->getAttributeWeights();
		for(std::map<std::string, double>::iterator i = robotAttributes.begin(); i != robotAttributes.end(); ++i)
		{
			if(taskAttributes.count(i->first) == 1)
			{
				utility += taskAttributes[i->first]*i->second;
			}
		}

		return utility;
	}

	~MyUtilityHelper()
	{
	}
};
