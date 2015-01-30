#include "Prerequisite.h"

class MyPrerequisite: public Prerequisite
{
	MyPrerequisite::MyPrerequisite()
	{
	}

	bool Prerequisite::isFulfilled()
	{
		return true;
	}

	MyPrerequisite::~MyPrerequisite()
	{
	}
};
