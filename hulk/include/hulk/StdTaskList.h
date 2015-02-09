#include "hsmrs_framework/TaskList.h"

class StdTaskList : public TaskList
{
private:
	std::vector<Task*>* list;
	bool sorted;
public:
	StdTaskList();

	void addTask(Task* task);

	std::vector<Task*> getTasks();

	bool isEmpty();

	Task* pullNextTask();

	Task* removeTask(int id);

	void setPriority(int task, double priority);

	Task* getTask(int id);
};