#include "MecanumKinematics.h"

double MecanumKinematics::getSpeedForWheel(int wheel, double x, double y, double rot)
{    
    return this.getPowerForWheel(wheel, x/MAX_VEL_X, y/MAX_VEL_Y, rot/MAX_VEL_R)*MAX_VEL_WHEEL;
}

double MecanumKinematics::getPowerForWheel(int wheel, double x, double y, double rot)
{
	double power = 0;
    double angle = 0; //radians counterclockwise from y-axis
    double xScaled = x; //positive x is right
    double yScaled = y; //positive y is forward
    double norm = sqrt(x*x + y*y);
    double translation = 0;
    double angleMult = 0;
    
    if(sqrt(x*x + y*y) > 1)
    {
        xScaled /= norm;
        yScaled /= norm;
    }
	
    //find angle of translation
    if(x == 0 && y > 0)
    {
        angle = 0;
    }
    else if(x == 0 && y < 0)
    {
        angle = PI/2;
    }
    else if(x > 0)
    {
        angle = PI/2 + math::atan(y/x);
    }
    else if(x < 0)
    {
        angle = 3*PI/2 + math::atan(y/x);
    }
    
    //find norm multiplier and rotation power based on angle and wheel
	if(wheel == WHEEL_A || wheel == WHEEL_C)
	{
		angleMult = sqrt(2)*math::cos(angle - PI/4); //guarantees full power at cardinal directions and in wheel facing direction, no power perpendicular to wheel facing direction
        power = -rot;
	}
    else if(wheel == WHEEL_B || wheel == WHEEL_D)
    {
        angleMult = sqrt(2)*math::cos(angle + PI/4);
        power = rot;
    }
    
    //normalize angleMult to -1 to 1 range
    if(angleMult > 1)
    {
        angleMult = 1;
    }
    else if(angleMult < -1)
    {
        angleMult = -1;
    }
    
    //get translation-only power and add to rotation power
    translation = norm*angleMult;
    power += translation;
    
    //normalize motor power to -1 to 1 range
    if(power > 1)
    {
        power = 1;
    }
    else if(power < -1)
    {
        power = -1;
    }
    
    //adjust for motor orientation
    if(wheel == WHEEL_B || wheel == WHEEL_C)
    {
        power = -power;
    }
    
    return power;
}
