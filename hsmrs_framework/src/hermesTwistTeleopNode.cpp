#include "Definitions.h"
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ar_track_alvar/AlvarMarkers.h>
//#include <Xinput.h>

#define VEL_COEF 4000

#define WHEEL_A 1
#define WHEEL_B 2
#define WHEEL_C 3
#define WHEEL_D 4

#define PI 3.1415926535

#define FALSE 0
#define TRUE 1

typedef void* HANDLE;
typedef int BOOL;
typedef unsigned int DWORD;
typedef unsigned short WORD;

BOOL showErrorInformation(DWORD p_ulErrorCode);
void enable(HANDLE handle, WORD node);
double getPowerForWheel(int wheel, double x, double y, double rot);
void controlCallback(const sensor_msgs::Joy::ConstPtr& joy);
void tagCallback(ar_track_alvar::AlvarMarkersConstPtr markerList);

DWORD errorCode;
HANDLE port;
float markerX, markerZ;
double lastMarkerTime;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "hermesTeleopNode");
	ros::NodeHandle nodeHandle;
	ros::Subscriber sub = nodeHandle.subscribe<sensor_msgs::Joy>("joy", 4, controlCallback);
    ros::Subscriber tagSub = nodeHandle.subscribe("/ar_pose_marker", 1, tagCallback);

	int i;
	long vel;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, "EPOS2");
	strcpy(pProtocolStackName, "MAXON SERIAL V2");
	strcpy(pInterfaceName, "USB");
	strcpy(pPortName, "USB0");

	port = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &errorCode);
	if(port == 0)
	{
		showErrorInformation(errorCode);
	}
	else
	{
		printf("SUCCESS: got port %p\n", port);
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	//setup loop
	for(i = 1; i <= 4; i++)
	{		
		if(!VCS_ActivateVelocityMode(port, i, &errorCode))
		{
			showErrorInformation(errorCode);
		}
		else
		{
			printf("Motor %d set to velocity mode\n", i);
		}

		enable(port, (WORD) i);

		VCS_GetVelocityMust(port, i, &vel, &errorCode);
		printf("Motor %d vel %ld\n", i, vel);
	}
	/*
	//test code
	system("PAUSE");
	
	printf("Starting motor movement test...\n");
	for(i = 1; i <= 4; i++)
	{
		printf("Motor %d\n", i);
		vel = 5000;
		if(!VCS_SetVelocityMust(port, i, vel, &errorCode))
		{
			showErrorInformation(errorCode);
		}
		else
		{
			printf("Motor %d set to velocity %d\n", i, vel);
		}
		sleep(1);

		vel = 0;
		if(!VCS_SetVelocityMust(port, i, vel, &errorCode))
		{
			showErrorInformation(errorCode);
		}
		else
		{
			printf("Motor %d set to velocity %d\n", i, vel);
		}
		system("PAUSE");
	}
	*/
	
	printf("Starting teleop...\n");
	
	ros::spin();
	
	return 0;
}

BOOL showErrorInformation(DWORD p_ulErrorCode)
{
    char* pStrErrorInfo;

    if((pStrErrorInfo = (char*)malloc(200)) == NULL)
    {
        return FALSE;
    }

    if(VCS_GetErrorInfo(p_ulErrorCode, pStrErrorInfo, 200))
    {
        printf("ERROR: %s\n", pStrErrorInfo);

        free(pStrErrorInfo);

        return TRUE;
    }
    else
    {
        free(pStrErrorInfo);
        printf("ERROR: Can't read error info, code %u\n", p_ulErrorCode);

        return FALSE;
    }
}

void enable(HANDLE handle, WORD node)
{
    BOOL oFault = FALSE;
	DWORD error;

    if(!VCS_GetFaultState(handle, node, &oFault, &error))
    {
        showErrorInformation(error);
        return;
    }

    if(oFault)
    {
        if(!VCS_ClearFault(handle, node, &error))
        {
            showErrorInformation(error);
            return;
        }
    }

    if(!VCS_SetEnableState(handle, node, &error))
    {
        showErrorInformation(error);
    }
	else
	{
		printf("Motor %hd enabled\n", node);
	}
}

double getPowerForWheel(int wheel, double x, double y, double rot)
{
	double power = 0;
    double angle = 0; //radians counterclockwise from y-axis
    double xScaled = x; //positive x is right
    double yScaled = y; //positive y is forward
    double norm = sqrt(x*x + y*y);
    double translation = 0;
    double angleMult = 0;
    
    if(norm > 1)
    {
        xScaled /= norm;
        yScaled /= norm;
    }
	
    //find angle of translation
    if(x == 0 && y > 0)
    {
        angle = PI;
    }
    else if(x == 0 && y < 0)
    {
        angle = 0;
    }
    else if(x > 0)
    {
        angle = PI/2 + atan(y/x);
    }
    else if(x < 0)
    {
        angle = 3*PI/2 + atan(y/x);
    }
    
    //find norm multiplier and rotation power based on angle and wheel
	if(wheel == WHEEL_A || wheel == WHEEL_C)
	{
		angleMult = -sqrt(2.0)*cos(angle - PI/4); //guarantees full power at cardinal directions and in wheel facing direction, no power perpendicular to wheel facing direction
	}
    else if(wheel == WHEEL_B || wheel == WHEEL_D)
    {
        angleMult = -sqrt(2.0)*cos(angle + PI/4);
    }

	if(wheel == WHEEL_A || wheel == WHEEL_B)
	{
		power = -rot;
	}
	else if(wheel == WHEEL_C || wheel == WHEEL_D)
	{
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
    if(wheel == WHEEL_A || wheel == WHEEL_B)
    {
        power = -power;
    }
    
    return power;
}

void controlCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    double x = -twist->linear.x;
    double y = twist->linear.y;
    double r = twist->angular.z;

    int wheel;
    long vel;

    for(wheel = 1; wheel <= 4; wheel++)
    {
	    vel = getPowerForWheel(wheel, x, y, r)*VEL_COEF;
	    if(!VCS_SetVelocityMust(port, wheel, vel, &errorCode))
	    {
		    showErrorInformation(errorCode);
	    }
    }
}


