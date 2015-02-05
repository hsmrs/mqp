#ifndef _MY_PREREQUISITE_H_
#define _MY_PREREQUISITE_H_

#include "hsmrs_framework/Prerequisite.h"

class MyPrerequisite: public Prerequisite
{
public:
	MyPrerequisite();

	virtual bool isFulfilled();

	~MyPrerequisite();
};

#endif