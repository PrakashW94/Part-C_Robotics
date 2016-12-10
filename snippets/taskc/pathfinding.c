/*
TO DO LIST
- mDist and dDist as percentages?
- Test recursion loop
- Suitable LEDs
*/

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "uart/e_uart_char.h"

#include "custom_util/utility.h"
#include "constants.c"

#include "math.h"
#include "string.h"
#include <stdio.h>

/*
PI: Pi (duh)
Rd: Diameter of the robot (wheel axle) in metres
Wd: Diameter of the wheel in metres
*/

#define PI 3.14159
#define Rd 0.052
#define Wd 0.041 

/*
x: current x value
y: current y value
xg: x goal value
yg: y goal value
r: number of steps rotated in segment of movement
h: number of steps moved linearly in a segment of movement
rCurrent: total number of steps rotated
q[]: running counter for percentage complete
d[]: running counter for distance from m-line
i: counter for q and d
j: program steps counter
fp: front(wide) prox value
*/

int x, y, xg, yg, r, h, rCurrent, i, j, fp, hCurrent;
int q[100];
double d[100];

/*
Sensor ranges
NOT USED
*/

int frontwide[] = {6, 7, 0, 1};
int front[] = {0, 1};
int left[] = {0, 7, 6, 5};
int right[] = {0, 1, 2};

/*
Other
*/

int checkMline = 0;

double distToGoal(double x, double y, double xg, double yg)
{
	return sqrt(pow((xg-x), 2) + pow((yg-y),2));
}

double angleToRotate(double steps)
{
	return ((steps/1000)*Wd*PI*2*PI)/(Rd*PI);
}

int stepsToRotate(double angle)
{
	return floor((1000 * angle * Rd * PI)/(Wd * PI * 2 * PI));
}

double angleFromGoal(double x, double y, double xg, double yg)
{
	double diffX = xg-x;
	double diffY = yg-y;
	double a = pow(diffX, 2);
	double b = pow(diffY, 2);
	double linDist = sqrt(a + b);
	double angle = acos(diffY / linDist);

	if (diffX < 0)
	{	
		angle = (2 * PI) - angle;
	}
	// Take into account how we're currently oriented.
	angle -= angleToRotate(rCurrent);
	if (angle < 0)
	{
		angle = 2*PI + angle;
	}
	return angle;
}

double distToMline(double x, double y, double xg, double yg)
{
	double res = (y*xg-x*yg)/sqrt(pow(xg, 2)+pow(yg, 2));
	if (res < 0)
	{
		res*= -1;
	}
	return res;
}

int RADtoDEG(double rad)
{
    return rad*180/PI;
}

double DEGtoRAD(int angle)
{
    return angle/180*PI;
}

void waitForSteps(int steps)
{
	int startSteps = e_get_steps_left();
	e_set_steps_left(0);
	
	int endSteps = e_get_steps_left() + steps;
	while( abs(e_get_steps_left()) < endSteps );
	
	e_set_steps_left(startSteps);
}

void clearSteps()
{
	e_set_steps_left(0);
	e_set_steps_right(0);
	h = 0;
	r = 0;
}

void setGoal(int gx, int gy)
{
	x = 0;
	y = 0;
	r = 0;
	xg = gx;
	yg = gy;
	fp = 0;
	h = 0;
	r = 0;
	rCurrent = 0;
	
	i = 1;
	j = 0;
	
	d[0] = distToGoal(x, y, xg, yg);
	q[0] = q[i] = 0;
}

void setSpeed(int l, int r)
{
	e_set_speed_left(l);
	e_set_speed_right(r);
}

void updateProgress()
{
	rCurrent += r;
	double angle = angleToRotate(rCurrent);
	if (angle > 2*PI)
	{
		angle = angle - 2 * PI;
		rCurrent = rCurrent - stepsToRotate(2*PI);
	}
	x += floor(h * sin(angle));
	y += floor(h * cos(angle));
	
	d[i] = distToGoal(x, y, xg, yg);
	q[i] = 100 - floor((d[i]/d[0])*100);
	j++;
	clearSteps();	
}

void moveToGoal()
{
	double angle = angleFromGoal(x, y, xg, yg);
	
	if (angle > PI)
	{
		angle = 2*PI - angle; 
		r = stepsToRotate(angle);
		reportValue("rotating ACW", RADtoDEG(angle));
		setSpeed(-s, s);
		waitForSteps(r);
		setSpeed(s, s);
		r = stepsToRotate(2 * PI - angle);
	}
	else
	{
		r = stepsToRotate(angle);
		reportValue("rotating CW", RADtoDEG(angle));
		setSpeed(s, -s);
		waitForSteps(r);
		setSpeed(s, s);
	}
	LED0 = 0;
	updateProgress();
}

int getProx(int sensors[], int noOfSensors)
{
	int k;
	double prox;
	for (k = 0;k < noOfSensors; k++)
	{
		prox = prox + e_get_prox(sensors[k]);
	}
	return (int)(prox/noOfSensors);
}

void progressReport()
{
	reportValue("****START PROGRESS REPORT****", -1);
	reportValue("Front Proximity", fp);
	reportValue("Current x", x);
	reportValue("Goal x", xg);
	reportValue("Current y", y);
	reportValue("Goal y", yg);
	reportValue("Distance from Goal", d[i]);
	reportValue("Percentage Complete", q[i]);
	reportValue("Current Direction (steps)", (rCurrent));
	reportValue("Current Direction", RADtoDEG(angleToRotate(rCurrent)));
	reportValue("Number of states", i);
	reportValue("Number of updates", j);
	reportValue("****END PROGRESS REPORT****", -1);
}

void scanObject()
{
	int frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
	while (frontProx < 1000)
	{
		//turn left until object is in front
		reportValue("Turning left", -1);
		reportValue("frontProx", frontProx);
		setSpeed(-s, s);
		frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
		wait(delayTimer);
	}
	/*
	int tempSteps = angleToRotate(DEGtoRAD(90));
	setSpeed(-s, s);
	waitForSteps(tempSteps);
	setSpeed(0,0);
	*/
	
	reportValue("Stopping", -1);
	setSpeed(0, 0);
	while(1){}
	
	/*
	//r = e_get_steps_left();
	
	int rightProx = e_get_prox(2);
	while (rightProx > 350)
	{
	//move foward until edge of object
		setSpeed(s, s);
		rightProx = e_get_prox(2);
	}
	
	int leftProx = e_get_prox(5);
	while (leftProx < 350)
	{
		//rotate 180 degrees
		setSpeed(-s, s);
		leftProx = e_get_prox(5);
	}	
	*/
}

/*
Assume you're at the object at startSteps
Reset orientation to face the object at a perpendicular angle
	rotate 360 degrees to find orientation at which the sensor value is the highest. 
	greedy? Rotate until maximum is found
Rotate 90 degrees left
Move forward until object is no longer detected on right sensors
Rotate 180 degrees and move forward until object is no longer detected on left sensors
Draw prediction of object and find optimum points to push from?
*/
