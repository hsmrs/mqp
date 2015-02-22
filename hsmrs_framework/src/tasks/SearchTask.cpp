#include "hsmrs_implementations/SearchTask.h"

const int SearchTask::MIN_OWNERS = 1;
const int SearchTask::MAX_OWNERS = 2;

SearchTask::SearchTask(){
	//attributeWeights;
	priority = 0.0;
	id = -1; //unknown
	prerequisite = new MyPrerequisite();
	progress = new MyProgress();
}

SearchTask::SearchTask(const hsmrs_framework::TaskMsg::ConstPtr& msg){
	

	for(std::string owner : msg->owners){
		owners.push_back(owner);
	}

	priority = msg->priority;
	id = msg->id;
	prerequisite = new MyPrerequisite();
	progress = new MyProgress();
}

SearchTask::SearchTask(std::string strDelimitedTask){
	
}

std::string SearchTask::getType(){
	return "SearchTask";
}

std::vector<geometry_msgs::PointStamped> SearchTask::getBoundaryVertices(){
	return boundaryVertices;
}

	/**
	 * Retrieves a vector of Agents who have claimed this Task
	 * @return A vector of Agents who have claimed this Task
	 */
 std::vector<std::string> SearchTask::getOwners(){
	return owners;
}

	/**
	 * Determines the minimum number of Agents that need to claim this
	 * Task for the Task to be executed.
	 * @return The minimum number of owners for this Task.
	 */
 int SearchTask::getMinOwners(){
	return MIN_OWNERS;
}

	/**
	 * Determines the maximum number of Agents that can claim this
	 * Task.
	 * @return The maximum number of owners for this task.
	 */
 int SearchTask::getMaxOwners(){
	return MAX_OWNERS;
}

	/**
	 * Retrieves a mapping between Attributes and their weights.
	 * @return A map of strings and doubles
	 */
 std::map<std::string, double> SearchTask::getAttributeWeights(){
	return std::map<std::string, double>(attributeWeights);
}

	/**
	 * Retrieves a vector of subtasks which need to be completed as part of
	 * this task
	 * @return A vector of Tasks which are a part of this Task.
	 */
 std::vector<Task*> SearchTask::getSubtasks(){
	return std::vector<Task*>(subTasks);
}

	/**
	 * Adds an Agent as an owner to this task, if another owner can be added
	 * @param agent The Agent to be added as an owner.
	 */
 void SearchTask::addOwner(std::string agent){
	owners.push_back(agent);
}

	/**
	 * Removes the given Agent as an owner.
	 * @param agent The name of the Agent to be removed as an owner.
	 */
 void SearchTask::removeOwner(std::string name){
	 std::vector<std::string>::iterator result = find(owners.begin(), owners.end(), name);

    if (result != owners.end()) owners.erase(result);        
}

 double SearchTask::getPriority(){
	return priority;
}

 void SearchTask::setPriority(double p){
	priority = p;
}

 bool SearchTask::isReady(){
	return prerequisite->isFulfilled();
}

 int SearchTask::getID(){
	return id;
}

 void SearchTask::setProgress(double val){
	//progress->set(val);
}
