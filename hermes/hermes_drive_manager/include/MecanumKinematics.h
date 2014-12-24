#include <math>

#define WHEEL_A 0
#define WHEEL_B 1
#define WHEEL_C 2
#define WHEEL_D 3

class MecanumKinematics
{
public:
	double getPowerForWheel(int wheel, double x, double y, double rot);
};