#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "uart/e_uart_char.h"

#include "avoid_objects.h"
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
r: number of steps rotated from horiz 
q[]: running counter for percentage complete
d[]: running counter for distance from m-line
i: counter for q and d
j: program steps counter
*/

int x, y, xg, yg, r, i, j, fp;
int q[100];
double d[100];

/*
Sensor ranges
*/

int frontwide[] = {6, 7, 0, 1};
int front[] = {0, 1};
int left[] = {0, 7, 6, 5};
int right[] = {0, 1, 2};

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

double angleFromGoal(double x, double y, double xg, double yg, int stepsRotated)
{
    double alpha;
    alpha  = atan((yg-y)/(xg-x));
    double theta;
    theta = angleToRotate(stepsRotated);
    return alpha - theta;
}

double distToM_line(double x, double y, double xg, double yg)
{
	return (x*yg-y*xg)/sqrt(pow(xg, 2)+pow(yg, 2));
}

int RADtoDEG(double rad)
{
    return rad*180/PI;
}

void waitForSteps(int steps)
{
	int endSteps = e_get_steps_left() + steps;
	while( e_get_steps_left() < endSteps );
}

void clearSteps()
{
	e_set_steps_left(0);
	e_set_steps_right(0);
}

void setGoal()
{
	x = 0;
	y = 0; 
	r = 0;
	xg = 1000;
	yg = 1000;
	fp = 0;
	
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

void rotateToGoal()
{
	double angle = angleFromGoal(x, y, xg, yg, r);
	r = stepsToRotate(angle);
	setSpeed(s, -s);
	waitForSteps(r);
	setSpeed(0, 0);
	clearSteps();
}

void updateProgress()
{
	int h = e_get_steps_left();
	double angle = angleToRotate(r);
	x = floor(h * cos(angle));
	y = floor(h * sin(angle));
	d[i] = distToGoal(x, y, xg, yg);
	q[i] = 100 - floor((d[i]/d[0])*100);
	j++;
}

void moveToGoal()
{
	rotateToGoal();
	setSpeed(s, s);
}
/*
int getProx(int sensors[], int noOfSensors)
{
	int k;
	int prox;
	for (k = 0;k < noOfSensors; k++)
	{
		prox = prox + e_get_prox(sensors[k]);
	}
	return (int)(prox/noOfSensors);
}*/

void progressReport()
{
	reportValue("****START PROGRESS REPORT****", -1);
	reportValue("Front Proximity", fp);
	reportValue("Current x", x);
	reportValue("Goal x", xg);
	reportValue("Current y", y);
	reportValue("Goal y", yg);
	reportValue("Steps h", e_get_steps_left());
	reportValue("Distance from Goal", d[i]);
	reportValue("Percentage Complete", q[i]);
	reportValue("Steps rotated", r);
	reportValue("Number of states", j);
	reportValue("****END PROGRESS REPORT****", -1);
}
/*
void avoidBoundary()
{
	
	//int front[] = {7, 0};
	int frontLeds[] = {0};
	//int right[] = {0, 1, 2};
	int rightLeds[] = {1, 2, 3};
	//int left[] = {0, 7, 6, 5};
	int leftLeds[] = {5, 6, 7};
	
	int frontProx = getProx(front, 2);
	
	//left led on
	setLeds(leftLeds, 3);
	while (frontProx > 400)
	{
		//turn left until front prox doesn't detect object
		reportValue("turning left", frontProx);
		updateLeft(-s);
		updateRight(s);
		frontProx = getProx(front, 2);
		wait(delayTimer);
	}
	//left led off
	setLeds(leftLeds, 3);
	
	int rightProx = getProx(right, 3);
	wait(delayTimer); //wait to get correct prox value
	int leftProx = getProx(left, 4);
	wait(delayTimer); //wait to get correct prox value
	while (rightProx > 350)
	{
		//go straight, if right senses object, turn left a bit
		switch(rightProx)
		{
			case 350 ... 650: 
			{//go straight, increasing right
				setLeds(frontLeds, 1);
				while ((rightProx < 650) && (rightProx > 350))
				{
					if(leftProx > 400)
					{
						reportValue("entering recursion loop", leftProx);
						avoidBoundary();
					}
					else
					{	
						reportValue("going straight, object on right", rightProx);
						updateLeft(s);
						updateRight(s);
					}
					rightProx = getProx(right, 3);
					wait(delayTimer); //wait to get correct prox value
					leftProx = getProx(left, 4);
					wait(delayTimer); //wait to get correct prox value
				}
				setLeds(frontLeds, 1);
				break;
			}
			case 651 ... 2000:
			{//object found, turn left, decreasing right
				setLeds(leftLeds, 3);
				while (rightProx > 650)
				{
					reportValue("turning left a bit", rightProx);
					updateLeft(-s);
					updateRight(s);
					rightProx = getProx(right, 3);
				}
				setLeds(leftLeds, 3);
				break;
			}
			default: 
			{
				reportValue("outside range", rightProx);
				break;
			}
		}
		rightProx = getProx(right, 3);
		wait(delayTimer);
	}

	setLeds(rightLeds, 3);
	//this need to be initialised as 0 as first value is incorrect.
	leftProx = 0;
	while (leftProx < 150)
	{
		//turn right until left senses object
		reportValue("turning right", leftProx);
		updateLeft(s);
		updateRight(-s);
		leftProx = getProx(left, 4);
		wait(delayTimer);
		//TO DO: bug where this loop breaks prematurely?
	}
	//left leds off
	setLeds(rightLeds, 3);
}
*/
void pathfinder()
{	
	setGoal();
	while(1)
	{
		moveToGoal();
		reportValue("Frontwide", fp);
		while (fp < 400 && q[i] < 90)
		{
			fp = getProx(frontwide, 4);
			waitForSteps(100); //wait for time?
			updateProgress();
			reportValue("Frontwide", fp);
			reportValue("q[i]", q[i]);
			//progressReport();
		}
		if (q[i] > 90)
		{
			e_set_led(8, 2);
			setSpeed(0,0);
			while(1){}
		}
/*
		int qb = q[i];
		int db = d[i];
		int mDist = 0;
		int onMline;
		while
		(
			q[i] < 90 &&
			q[i] != qb &&
			(
				onMline &&
				db < d[i] &&
				fp < 400
			)
		)
		{
			onMline = 0;
			avoidBoundary();
			updateProgress();
			mDist = distToM_line(x, y, xg, xg);
			if (mDist < 50)
			{
				onMline = 1;
			}
		}
		
		if (q[i] > 90)
		{
			e_set_led(8, 2);
			setSpeed(0,0);
			while(1){}
		}
		if (q[i] == qb)
		{
			e_set_led(1,2);
			setSpeed(0,0);
			while(1){}
		}
		i++;
*/
	}
}