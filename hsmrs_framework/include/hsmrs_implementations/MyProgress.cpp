#include <hsmrs_framework/Progress.h>

class MyProgress : public Progress
{
private:
	double val;
	
public:
	MyProgress()
	{
		val = 0;
	}

	MyProgress(double val)
	{
		this->val = val;
	}

	double report()
	{
		return val;
	}

	void set(double val)
	{
		this->val = val;
	}
};
