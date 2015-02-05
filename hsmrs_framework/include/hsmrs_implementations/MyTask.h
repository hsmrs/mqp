#ifndef _MY_TASK_H_
#define _MY_TASK_H_

#include "hsmrs_framework/Task.h"
#include "hsmrs_framework/Agent.h"
#include "hsmrs_implementations/MyAttributeWeights.h"
#include "hsmrs_implementations/MyProgress.h"

class MyTask : public hsmrs_framework::Task {
private:
	int id;
	double priority;
	std::vector<hsmrs_framework::Task*>* subtasks;
	std::vector<Agent*>* owners;
	AttributeWeights* weights;
	MyProgress* progress;

public:
	MyTask(int id, double priority);

	void addOwner(Agent* agent);

	std::map<std::string, double>* getAttributeWeights();

	int getID();

	int getMaxOwners();

	int getMinOwners();

	//TODO this implementation is memory leaky
	std::vector<Agent*>* getOwners();

	double getPriority();

	std::vector<hsmrs_framework::Task*>* getSubtasks();

	bool isReady();

	void removeOwner(std::string name);

	void setPriority(double val);

	void setProgress(double val);
};

#endif