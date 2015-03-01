#ifndef _MY_UTILITY_HELPER_H_
#define _MY_UTILITY_HELPER_H_

#include "hsmrs_framework/UtilityHelper.h"
#include "hsmrs_framework/Robot.h"
#include "hsmrs_framework/Task.h"
#include <hsmrs_framework/TaskMsg.h>
#include "ros/ros.h"
#include <map>
#include <exception>
#include <typeinfo>

class MyUtilityHelper: public UtilityHelper {
public:
	MyUtilityHelper();

	double calculate(Robot* robot, Task* task);

	~MyUtilityHelper();
};

#endif
