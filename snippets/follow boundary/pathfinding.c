/*
TO DO LIST
- Fix condition for entering/exiting avoidBoundary
- Add updateProgress while going straight within avoidBoundary
- Fix reporting(?)
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
    alpha  = atan((xg-x)/(yg-y));
	if (xg - x < 0 && yg - y < 0)
	{
		alpha += PI;
	}
    double theta;
    theta = angleToRotate(stepsRotated);
    return alpha - theta;
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

void waitForSteps(int steps)
{
	int endSteps = e_get_steps_left() + steps;
	while( abs(e_get_steps_left()) < endSteps );
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
	xg = 0;
	yg = 5000;
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

void updateProgress()
{
	int h = e_get_steps_left();
	double angle = angleToRotate(r);
	if (angle < 0)
	{
		angle += PI;
		reportValue("Added PI to angle", RADtoDEG(angle));
	}
	if (r == 0)
	{
		x = floor(h * sin(angle));
		y = floor(h * cos(angle));
	}
	else
	{
		
		x = floor(h * cos(angle));
		y = floor(h * sin(angle));
	}
	d[i] = distToGoal(x, y, xg, yg);
	q[i] = 100 - floor((d[i]/d[0])*100);
	j++;
}

void moveToGoal()
{
	double angle = angleFromGoal(x, y, xg, yg, r);
	
	if (angle < 0)
	{
		angle *= -1; 
		r = stepsToRotate(angle);
		int h = e_get_steps_left();
		setSpeed(-s, s);
		waitForSteps(r);
		e_set_steps_left(h);
		setSpeed(s, s);
	}
	else
	{
		r = stepsToRotate(angle);
		int h = e_get_steps_left();
		setSpeed(s, -s);
		waitForSteps(r);
		e_set_steps_left(h);
		setSpeed(s, s);
	}
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
	reportValue("Steps h", e_get_steps_left());
	reportValue("Distance from Goal", d[i]);
	reportValue("Percentage Complete", q[i]);
	reportValue("Steps rotated", r);
	reportValue("Number of states", j);
	reportValue("****END PROGRESS REPORT****", -1);
}

int avoidBoundary()
{	
	//int frontLeds[] = {0};
	//int rightLeds[] = {1, 2, 3};
	//int leftLeds[] = {5, 6, 7};
	
	//int frontProx = getProx(front, 2);
	int frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
	
	int r0, rStore, h;
	r0 = 0;
	
	int mDist = 0;
	
	//left led on
	//setLeds(leftLeds, 3);
	rStore = r;
	r0 = e_get_steps_right();
	h = e_get_steps_left();
	while (frontProx > 150)
	{
		//turn left until front prox doesn't detect object
		//reportValue("turning left", frontProx);
		setSpeed(-s, s);
		frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
		wait(delayTimer);
	}
	//left led off
	//setLeds(leftLeds, 3);
	r -= e_get_steps_right() - r0;
	e_set_steps_left(h);
	
	int rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
	//wait(delayTimer); //wait to get correct prox value
	int leftProx =  (int)((e_get_prox(0) + e_get_prox(7) + e_get_prox(6) + e_get_prox(5))/4);
	//wait(delayTimer); //wait to get correct prox value
	while (rightProx > 100)
	{
		//go straight, if right senses object, turn left a bit
		switch(rightProx)
		{
			case 100 ... 650: 
			{//go straight, increasing right
				//setLeds(frontLeds, 1);
				while ((rightProx < 650) && (rightProx > 100))
				{
					if(leftProx > 400)
					{
						reportValue("entering recursion loop", leftProx);
						avoidBoundary();
					}
					else
					{
						//reportValue("going straight, object on right", rightProx);
						setSpeed(s, s);
						updateProgress();
						mDist = distToMline(x, y, xg, yg);
						reportXY(x, y, mDist);
						if (mDist < 50)
						{
							reportValue("ON MLINE, returning to main code", -1);
							return 1;
						}
					}
					rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
					leftProx = (int)((e_get_prox(0) + e_get_prox(7) + e_get_prox(6) + e_get_prox(5))/4);
					wait(delayTimer);
				}
				//setLeds(frontLeds, 1);
				break;
			}
			case 651 ... 2000:
			{//object found, turn left, decreasing right
				//setLeds(leftLeds, 3);
				r0 = e_get_steps_right();
				h = e_get_steps_left();
				while (rightProx > 650)
				{
					//reportValue("turning left a bit", rightProx);
					setSpeed(-s, s);
					rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
					wait(delayTimer);
				}
				//setLeds(leftLeds, 3);
				r -= e_get_steps_right() - r0;
				e_set_steps_left(h);
				break;
			}
			default: 
			{
				reportValue("outside range", rightProx);
				break;
			}
		}
		rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
	}

	//setLeds(rightLeds, 3);
	r0 = e_get_steps_left();
	reportValue("r0", r0);
	h = e_get_steps_left();
	//this need to be initialised as 0 as first value is incorrect.
	leftProx = (int)((e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/3);
	while (leftProx < 80)
	{
		//turn right until left senses object
		//reportValue("turning right", leftProx);
		setSpeed(s, -s);
		leftProx = (int)((e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/3);
		//reportValue("left steps", e_get_steps_left());
		wait(delayTimer);
	}
	//left leds off
	//setLeds(rightLeds, 3);
	reportValue("left steps", e_get_steps_left());
	r += e_get_steps_left() - r0;
	e_set_steps_left(h);
	reportValue("change in r", r - rStore);
	reportValue("lin distance travelled", h);
	setSpeed(s, s);
	return 0;
}

void pathfinder()
{	
	setGoal();
	while(1)
	{
		moveToGoal();
		while (fp < 150 && q[i] < 95)
		{
			//fp = getProx(frontwide, 4);
			fp = (int)((e_get_prox(6) + e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/4);
			waitForSteps(50); //wait for time?
			updateProgress();
			//reportValue("Frontwide", fp);
			//reportValue("q[i]", q[i]);
			//progressReport();
		}
		if (q[i] > 95)
		{
			e_set_led(8, 2);
			setSpeed(0,0);
			reportValue("Finished, Success!", -1);
			while(1){}
		}
		
		progressReport();
		//int qb = q[i];
		double db = d[i];
		int onMline = 0;
		
		do 
		{
			onMline = avoidBoundary();
/*			
			updateProgress();
			mDist = distToMline(x, y, xg, yg);
			reportValue("Distance to M-line", mDist);
*/
			progressReport();
			fp = (int)((e_get_prox(6) + e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/4);
			if (onMline)
			{
				reportValue("ON MLINE, Main Prog", -1);
				reportValue("db", db);
				reportValue("d[i]", d[i]);
				reportValue("fp", fp);
			}
		} while
		(
			q[i] < 95 &&
			//q[i] == qb &&
			!(onMline && db > d[i] && fp < 200)
		);
		
		if (q[i] > 95)
		{
			e_set_led(8, 2);
			setSpeed(0,0);
			reportValue("Finished, Success!", -1);
			while(1){}
		}
		
/*
		if (q[i] == qb)
		{
			e_set_led(1,2);
			setSpeed(0,0);
			reportValue("Finished, Failure!", -1);
			while(1){}
		}
*/
		i++;
	}
}
