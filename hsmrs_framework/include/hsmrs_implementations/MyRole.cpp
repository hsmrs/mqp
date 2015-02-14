#include "../include/hsmrs_framework/Role.h"

class MyRole : public Role
{
private:
	std::vector<std::string> tasks;
public:
	MyRole::MyRole()
	{
		tasks = std::vector<std::string>();
	}

	void Role::addTask(std::string task)
	{
		tasks.push_back(task);
	}

	void Role::removeTask(std::string task)
	{
		for (int i = 0; i < tasks.size(); i++)
		{
			if (tasks[i] == task)
			{
				tasks.erase(tasks.begin() + i);
				return;
			}
		}
	}

	std::vector<std::string> Role::getTasks()
	{
		return std::vector<std::string>(tasks);
	}
};
