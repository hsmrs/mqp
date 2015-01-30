#include "Task.h"
#include "MyAttributeWeights.cpp"
#include "MyProgress.cpp"

class MyTask : public Task
{
private:
	int id;
	double priority;
	std::vector<Task*>* subtasks;
	std::vector<Agent*>* owners;
	AttributeWeights* weights;
	MyProgress* progress;

public:
	MyTask::MyTask(int id, double priority)
	{
		this->id = id;
		this->priority = priority;
		subtasks = new std::vector<Task*>();
		owners = new std::vector<Agent*>();
		weights = new MyAttributeWeights();
		progress = new MyProgress();
	}

	void MyTask::addOwner(Agent* agent)
	{
		owners->push_back(agent);
	}

	std::map<std::string, double>* MyTask::getAttributeWeights()
	{
		return weights->getWeights();
	}

	int MyTask::getID()
	{
		return id;
	}

	int MyTask::getMaxOwners()
	{
		return 1;
	}

	int MyTask::getMinOwners()
	{
		return 1;
	}

	//TODO this implementation is memory leaky
	std::vector<Agent*>* MyTask::getOwners()
	{
		return new std::vector<Agent*>(*owners);
	}

	double MyTask::getPriority()
	{
		return priority;
	}

	std::vector<Task*>* MyTask::getSubtasks()
	{
		return new std::vector<Task*>(*subtasks);
	}

	bool MyTask::isReady()
	{
		return true;
	}

	void MyTask::removeOwner(std::string name)
	{
		for (int i = 0; i < owners->size(); i++)
		{
			if ((*owners)[i]->getName() == name)
			{
				owners->erase(owners->begin() + i);
				return;
			}
		}
	}

	void MyTask::setPriority(double val)
	{
		priority = val;
	}

	void MyTask::setProgress(double val)
	{
		progress->set(val);
	}
};
