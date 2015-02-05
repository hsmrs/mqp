#include "hsmrs_implementations/MyProgress.h"

MyProgress::MyProgress()
	{
		val = 0;
	}

	MyProgress::MyProgress(double val)
	{
		this->val = val;
	}

	double MyProgress::report()
	{
		return val;
	}

	void MyProgress::set(double val)
	{
		this->val = val;
	}
