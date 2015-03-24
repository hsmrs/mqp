#include "thor/StdTaskList.h"

	StdTaskList::StdTaskList()
	{
		list = new std::vector<Task*>;
		sorted = true;
	}

	void StdTaskList::addTask(Task* task)
	{
		for (std::vector<Task*>::iterator i = list->begin(); i != list->end(); ++i)
		{
			if(i[0]->getPriority() < task->getPriority())
			{
				list->insert(i, task);
				return;
			}
		}

		list->push_back(task);
	}

	std::vector<Task*> StdTaskList::getTasks()
	{
		std::vector<Task*> ret(*list);
		return ret;
	}

	bool StdTaskList::isEmpty()
	{
		return list->size() == 0;
	}

	Task* StdTaskList::pullNextTask()
	{
		for (int i = 0; i < list->size(); i++)
		{
			if (list->at(i)->isReady())
				return list->at(i);
		}

		return NULL;
	}

	Task* StdTaskList::removeTask(int id)
	{
		for (int i = 0; i < list->size(); i++)
		{
			if ((*list)[i]->getID() == id)
			{
				Task* ret = (*list)[i];
				list->erase(list->begin() + i);
				return ret;
			}
		}

		return NULL;
	}

	void StdTaskList::setPriority(int task, double priority)
	{
		getTask(task)->setPriority(priority);
	}

	Task* StdTaskList::getTask(int id)
	{
		for (int i = 0; i < list->size(); i++)
		{
			if ((*list)[i]->getID() == id)
				return (*list)[i];
		}

		return NULL;
	}