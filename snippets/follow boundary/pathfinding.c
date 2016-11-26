#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "uart/e_uart_char.h"

#include "custom_util/utility.h"

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
*/

int x, y, xg, yg, r;
int q[];
double d[];
int i = 1;

int distToGoal(double x, double y, double xg, double yg)
{
	return floor(sqrt( pow((xg-x), 2) + pow((yg-y),2) ) );
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

void init()
{
	e_init_port();
	e_init_prox();
	e_init_motors();
	e_init_uart1();
}

void setGoal()
{
	x = 0;
	y = 0; 
	r = 0;
	xg = 1500;
	yg = 1500;
	
	d[0] = distToGoal(x, y, xg, yg);
	q[0] = floor(100 - ((d[i]/d[0])*100));
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
	setSpeed(500, -500);
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
	q[i] = floor(100 - ((d[i]/d[0])*100));
}

void moveToGoal()
{
	rotateToGoal();
	setSpeed(300, 300);
}

void progressReport()
{
	reportValue("****START PROGRESS REPORT****", -1);
	reportValue("Current x", x);
	reportValue("Goal x", xg);
	reportValue("Current y", y);
	reportValue("Goal y", yg);
	reportValue("Percentage Complete", q[i-1]);
	reportValue("Distance from Goal", d[i-1]);
	reportValue("Steps rotated", r);
	reportValue("Number of states", i-1);
	reportValue("****END PROGRESS REPORT****", -1);
}

void pathfinder()
{
	init();
	setGoal();
	moveToGoal();
	
	waitForSteps(300);
	updateProgress();
	progressReport();
	waitForSteps(300);
	updateProgress();
	progressReport();
	waitForSteps(300);
	updateProgress();
	progressReport();
	waitForSteps(300);
	updateProgress();
	progressReport();
	waitForSteps(300);
	updateProgress();
	progressReport();
	while(1);
	
	//while(1)
	//{
/*		
	moveToGoal();
	while(q[i] < 100)
	{
		waitForSteps(300);
		updateProgress();
		progressReport();
	}
	setSpeed(0,0);
	while(1);
*/
	//}
}