#include "Progress.h"

class MyProgress : public Progress
{
private:
	double val;
	
public:
	MyProgress::MyProgress()
	{
		val = 0;
	}

	MyProgress::MyProgress(double val)
	{
		this->val = val;
	}

	double Progress::report()
	{
		return val;
	}

	void MyProgress::set(double val)
	{
		this->val = val;
	}
};