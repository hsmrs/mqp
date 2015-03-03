#include <hsmrs_implementations/MyTaskList.h>
#include <algorithm>
#include "ros/console.h"

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
    ROS_INFO("MyTaskList: removeTask");
    ROS_INFO("Getting list size: %d", list->size());
    ROS_INFO("Getting ID of 0th element: %d", (*list)[0]->getID());
    
	for (int i = 0; i < list->size(); i++)
	{
	    ROS_INFO("%d", i);
		if ((*list)[i]->getID() == id)
		{
		    ROS_INFO("Found matching task");
			//Task* ret = (*list)[i];
			list->erase(list->begin() + i);
		}
	}
	ROS_INFO("Exiting remove task");
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

