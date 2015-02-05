#ifndef _MY_UTILITY_HELPER_H_
#define _MY_UTILITY_HELPER_H_

#include "hsmrs_framework/UtilityHelper.h"
#include "hsmrs_framework/Human.h"
#include "hsmrs_framework/Task.h"
#include "hsmrs_framework/Agent.h"
#include <map>
#include <exception>
#include <typeinfo>

class MyUtilityHelper: public UtilityHelper {
	MyUtilityHelper();

	double calculate(Agent* agent, Task* task);

	~MyUtilityHelper();
};

#endif