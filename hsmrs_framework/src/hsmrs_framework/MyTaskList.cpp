#include <hsmrs_implementations/MyTaskList.h>
#include <algorithm>

MyTaskList::MyTaskList()
{
	list = new std::vector<Task*>;
	sorted = true;
}

void MyTaskList::addTask(Task* task)
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

std::vector<Task*> MyTaskList::getTasks()
{
	std::vector<Task*> ret(*list);
	return ret;
}

bool MyTaskList::isEmpty()
{
	return list->size() == 0;
}

Task* MyTaskList::pullNextTask()
{
	for (int i = 0; i < list->size(); i++)
	{
		if (list->at(i)->isReady())
			return list->at(i);
	}

	return NULL;
}

void MyTaskList::removeTask(int id)
{
	for (int i = 0; i < list->size(); i++)
	{
		if ((*list)[i]->getID() == id)
		{
			list->erase(list->begin() + i);
		}
	}
}

void MyTaskList::setPriority(int task, double priority)
{
	getTask(task)->setPriority(priority);
}

Task* MyTaskList::getTask(int id)
{
	for (int i = 0; i < list->size(); i++)
	{
		if ((*list)[i]->getID() == id)
			return (*list)[i];
	}

	return NULL;
}

bool comparePriorities(Task* a, Task* b)
{
    return a->getPriority() > b->getPriority();
}

void MyTaskList::sortByPriority()
{
    std::sort(list->begin(), list->end(), comparePriorities);
}

