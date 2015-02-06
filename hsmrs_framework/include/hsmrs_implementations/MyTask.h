#ifndef _MY_TASK_H_
#define _MY_TASK_H_

#include "hsmrs_framework/Task.h"
#include "hsmrs_framework/Agent.h"
#include "hsmrs_implementations/MyAttributeWeights.h"
#include "hsmrs_implementations/MyProgress.h"

class MyTask : public Task {
private:
	int id;
	double priority;
	std::vector<Task*>* subtasks;
	std::vector<std::string>* owners;
	AttributeWeights* weights;
	MyProgress* progress;

public:
	MyTask(int id, double priority);

	void addOwner(std::string agent);

	std::map<std::string, double>* getAttributeWeights();

	int getID();

	int getMaxOwners();

	int getMinOwners();

	//TODO this implementation is memory leaky
	std::vector<std::string> getOwners();

	double getPriority();

	std::vector<Task*>* getSubtasks();

	bool isReady();

	void removeOwner(std::string name);

	void setPriority(double val);

	void setProgress(double val);
};

#endif