#include <hsmrs_implementations/MyUtilityHelper.h>

MyUtilityHelper::MyUtilityHelper()
{
}

double MyUtilityHelper::calculate(Robot* robot, Task* task)
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

MyUtilityHelper::~MyUtilityHelper()
{
}
	
