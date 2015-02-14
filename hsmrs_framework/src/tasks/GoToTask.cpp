#include "hsmrs_implementations/GoToTask.h"

const int GoToTask::MIN_OWNERS = 1;
const int GoToTask::MAX_OWNERS = 1;

GoToTask::GoToTask(){
	goal = *(new geometry_msgs::Pose());
	//attributeWeights;
	priority = 0.0;
	id = -1; //unknown
	prerequisite = new MyPrerequisite();
	progress = new MyProgress();
}

GoToTask::GoToTask(hsmrs_framework::TaskMsg::ConstPtr& msg){

}

GoToTask::GoToTask(std::string strDelimitedTask){
	ROS_INFO("Creating go to task from colon delimited string");
	// ID;Name;[ParamName:ParamType=ParamValue, ...];[SubTask1, SubTask2, ...];[Owner1, Owner2, ...]
	priority = 0.0;
	std::vector<std::string> items;

	std::string delimiter = ";";
	size_t pos = 0;
	ROS_INFO("Splitting on colon");
	while ((pos = strDelimitedTask.find(delimiter)) != std::string::npos) {
    	items.push_back(strDelimitedTask.substr(0, pos));
    	strDelimitedTask.erase(0, pos + delimiter.length());
	}
	items.push_back(strDelimitedTask);

    id = std::stoi(items[0]);
	std::string paramList = items[2];

    std::vector<std::string> items2;
    paramList.erase(0, 1);
    paramList.erase(paramList.size() - 1);
    //paramList.pop_back();

    delimiter = ",";
	pos = 0;
	ROS_INFO("Splitting on comma for parameters");
	while ((pos = paramList.find(delimiter)) != std::string::npos) {
    	items2.push_back(paramList.substr(0, pos));
    	paramList.erase(0, pos + delimiter.length());
	}
	items2.push_back(paramList);

	std::vector<std::string> items3;
	delimiter = "=";
	pos = 0;
	ROS_INFO("Splitting on equals for x parameter value");
	while ((pos = items2[0].find(delimiter)) != std::string::npos) {
    	items3.push_back(items2[0].substr(0, pos));
    	items2[0].erase(0, pos + delimiter.length());
	}
	items3.push_back(items2[0]);

    double x = std::stoi(items3[1]);

    std::vector<std::string> items4;
    pos = 0;
	ROS_INFO("Splitting on equals for y parameter value");
	while ((pos = items2[1].find(delimiter)) != std::string::npos) {
    	items4.push_back(items2[1].substr(0, pos));
    	items2[1].erase(0, pos + delimiter.length());
	}
	items4.push_back(items2[1]);

    double y = std::stoi(items4[1]);

    ROS_INFO("Getting owner list");
    std::string ownerList = items[4];
    ownerList.erase(0, 1);
    ownerList.erase(ownerList.size() - 1);
    //ownerList.pop_back();

    std::vector<std::string> items5;
    delimiter = ",";
	pos = 0;
	ROS_INFO("Splitting on commas for owners");
	while ((pos = ownerList.find(delimiter)) != std::string::npos) {
    	items5.push_back(ownerList.substr(0, pos));
    	ownerList.erase(0, pos + delimiter.length());
	}
	items5.push_back(ownerList);

    for (std::vector<std::string>::iterator it = items2.begin(); it != items2.end(); ++it){
     	std::string owner = *it;
     	if (owner != ""){
     		owners.push_back(owner);
     	}
    }

    goal.position.x = x;
    goal.position.y = y;

    prerequisite = new MyPrerequisite();
	progress = new MyProgress();

	ROS_INFO("Go to task created from colon delimited string!");
}

std::string GoToTask::getType(){
	return "GoToTask";
}

geometry_msgs::Pose GoToTask::getGoal(){
	return goal;
}

	/**
	 * Retrieves a vector of Agents who have claimed this Task
	 * @return A vector of Agents who have claimed this Task
	 */
 std::vector<std::string> GoToTask::getOwners(){
	return owners;
}

	/**
	 * Determines the minimum number of Agents that need to claim this
	 * Task for the Task to be executed.
	 * @return The minimum number of owners for this Task.
	 */
 int GoToTask::getMinOwners(){
	return MIN_OWNERS;
}

	/**
	 * Determines the maximum number of Agents that can claim this
	 * Task.
	 * @return The maximum number of owners for this task.
	 */
 int GoToTask::getMaxOwners(){
	return MAX_OWNERS;
}

	/**
	 * Retrieves a mapping between Attributes and their weights.
	 * @return A map of strings and doubles
	 */
 std::map<std::string, double>* GoToTask::getAttributeWeights(){
	return &attributeWeights;
}

	/**
	 * Retrieves a vector of subtasks which need to be completed as part of
	 * this task
	 * @return A vector of Tasks which are a part of this Task.
	 */
 std::vector<Task*>* GoToTask::getSubtasks(){
	return &subTasks;
}

	/**
	 * Adds an Agent as an owner to this task, if another owner can be added
	 * @param agent The Agent to be added as an owner.
	 */
 void GoToTask::addOwner(std::string agent){
	owners.push_back(agent);
}

	/**
	 * Removes the given Agent as an owner.
	 * @param agent The name of the Agent to be removed as an owner.
	 */
 void GoToTask::removeOwner(std::string name){
	 std::vector<std::string>::iterator result = find(owners.begin(), owners.end(), name);

    if (result != owners.end()) owners.erase(result);        
}

 double GoToTask::getPriority(){
	return priority;
}

 void GoToTask::setPriority(double p){
	priority = p;
}

 bool GoToTask::isReady(){
	return prerequisite->isFulfilled();
}

 int GoToTask::getID(){
	return id;
}

 void GoToTask::setProgress(double val){
	//progress->set(val);
}
