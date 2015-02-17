#include <hsmrs_implementations/MyUtilityHelper.h>

MyUtilityHelper::MyUtilityHelper()
{
}

double MyUtilityHelper::calculate(Robot* robot, Task* task)
{
	double utility = 0;
	std::map<std::string, double> robotAttributes = robot->getState()->getAttributes();
	std::map<std::string, double> taskAttributes = task->getAttributeWeights();
	
	if(task->getType() == "FollowTagTask")
	{
	    double speed = 0;
	    double distance = 0;
	    double time = 1000;
	    
	    if(robotAttributes.count("speed") > 0)
	        speed = robotAttributes["speed"];
	    if(robotAttributes.count("distance") > 0)
	        distance = robotAttributes["distance"];
        
        if(speed > 0)
        {
            time = distance/speed;
        }
        
        return time == 0 ? 0 : 1/time;
	}
	else
	{
	    for(std::map<std::string, double>::iterator i = robotAttributes.begin(); i != robotAttributes.end(); ++i)
	    {
		    if(taskAttributes.count(i->first) == 1)
		    {
			    utility += taskAttributes[i->first]*i->second;
		    }
	    }
	}

	return utility;
}

MyUtilityHelper::~MyUtilityHelper()
{
}
	
