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
	    double time = 0;
	    
	    if(robotAttributes.count("speed") > 0)
	        speed = robotAttributes["speed"];
	    if(robotAttributes.count("distance") > 0)
	        distance = robotAttributes["distance"];
        
        if(speed > 0)
        {
            time = distance/speed;
        }
        
        return time == 0 ? 0 : 1./time;
	}
	else if(task->getType() == "GoToTask")
	{
	    double speed = 0;
	    double distance = 0;
	    double time = 0;
	    hsmrs_framework::TaskMsg* msg = task->toMsg();
	    double destX = std::stod(msg->param_values[0]);
	    double destY = std::stod(msg->param_values[1]);
	    double robotX, robotY;
	    
	    if(robotAttributes.count("speed") > 0)
	        speed = robotAttributes["speed"];
	    if(robotAttributes.count("x") > 0 && robotAttributes.count("y") > 0)
	    {
	        robotX = robotAttributes["x"];
	        robotY = robotAttributes["y"];
	        distance = sqrt((robotX - destX)*(robotX - destX) + (robotY - destY)*(robotY - destY));
        }
        
        if(speed > 0)
        {
            time = distance/speed;
        }
        
        return time == 0 ? 0 : 1./time;
	}
	else if(task->getType() == "SearchTask")
	{
	    double speed = 0;
	    double distance = 0;
	    double time = 0;
	    hsmrs_framework::TaskMsg* msg = task->toMsg();
	    
		std::string temp = msg->param_values[1].substr(0, msg->param_values[1].find(";"));
		temp.erase(0, 1);
    	temp.erase(temp.size() - 1);
    	int pos = temp.find(",");
	    
	    double destX = std::stod(temp.substr(0, pos));
	    double destY = std::stod(temp.substr(pos + 1));
	    double robotX, robotY;
	    
	    if(robotAttributes.count("speed") > 0)
	        speed = robotAttributes["speed"];
	    if(robotAttributes.count("x") > 0 && robotAttributes.count("y") > 0)
	    {
	        robotX = robotAttributes["x"];
	        robotY = robotAttributes["y"];
	        distance = sqrt((robotX - destX)*(robotX - destX) + (robotY - destY)*(robotY - destY));
        }
        
        if(speed > 0)
        {
            time = distance/speed;
        }
        
        return time == 0 ? 0 : 1./time;
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
	
