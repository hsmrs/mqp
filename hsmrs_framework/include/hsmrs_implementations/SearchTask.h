#ifndef _SEARCH_TASK_H_
#define _SEARCH_TASK_H_

#include "geometry_msgs/PointStamped.h"
#include "hsmrs_framework/Task.h"
#include "hsmrs_framework/TaskMsg.h"
#include "hsmrs_framework/Prerequisite.h"
#include "hsmrs_framework/Progress.h"
#include "hsmrs_implementations/MyPrerequisite.h"
#include "hsmrs_implementations/MyProgress.h"
#include <vector>

class SearchTask : public Task {
private:
	int id;
	double priority;
	int tagID;
	std::string boundaryVerticesString;
	std::vector<geometry_msgs::PointStamped> boundaryVertices;
	std::vector<std::string> owners;
	std::map<std::string, double> attributeWeights;
	std::vector<Task*> subTasks;
	Prerequisite* prerequisite;
	Progress* progress;

	const static int MIN_OWNERS;
	const static int MAX_OWNERS;

public:
	SearchTask();

	SearchTask(const hsmrs_framework::TaskMsg::ConstPtr& msg);
	
	SearchTask(hsmrs_framework::TaskMsg msg);

	SearchTask(std::string strDelimitedTask);
	
	virtual std::string getType();

	int getTagID();

	std::vector<geometry_msgs::PointStamped> getBoundaryVertices();

	/**
	 * Retrieves a vector of Agents who have claimed this Task
	 * @return A vector of Agents who have claimed this Task
	 */
	virtual std::vector<std::string> getOwners();

	/**
	 * Determines the minimum number of Agents that need to claim this
	 * Task for the Task to be executed.
	 * @return The minimum number of owners for this Task.
	 */
	virtual int getMinOwners();

	/**
	 * Determines the maximum number of Agents that can claim this
	 * Task.
	 * @return The maximum number of owners for this task.
	 */
	virtual int getMaxOwners();

	/**
	 * Retrieves a mapping between Attributes and their weights.
	 * @return A map of strings and doubles
	 */
	virtual std::map<std::string, double> getAttributeWeights();

	/**
	 * Retrieves a vector of subtasks which need to be completed as part of
	 * this task
	 * @return A vector of Tasks which are a part of this Task.
	 */
	virtual std::vector<Task*> getSubtasks();

	/**
	 * Adds an Agent as an owner to this task, if another owner can be added
	 * @param agent The Agent to be added as an owner.
	 */
	virtual void addOwner(std::string agent);

	/**
	 * Removes the given Agent as an owner.
	 * @param agent The name of the Agent to be removed as an owner.
	 */
	virtual void removeOwner(std::string name);

	virtual double getPriority();

	virtual void setPriority(double p);

	virtual bool isReady();

	virtual int getID();

	virtual void setProgress(double val);
	
	virtual hsmrs_framework::TaskMsg* toMsg();

};


#endif
