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
r: number of steps rotated in segment of movement
h: number of steps moved linearly in a segment of movement
rCurrent: total number of steps rotated
q[]: running counter for percentage complete
d[]: running counter for distance from m-line
i: counter for q and d
j: program steps counter
fp: front(wide) prox value
*/

int x, y, xg, yg, r, h, rCurrent, i, j, fp;
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
	angle -= angleToRotate(stepsRotated);;
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
	double angle = angleToRotate(r);

	x += floor(h * sin(angle));
	y += floor(h * cos(angle));
	
	d[i] = distToGoal(x, y, xg, yg);
	q[i] = 100 - floor((d[i]/d[0])*100);
	j++;
	clearSteps();
}

void moveToGoal()
{
	double angle = angleFromGoal(x, y, xg, yg, rCurrent);
	
	if (angle > PI)
	{
		angle = 2*PI - angle; 
		setSpeed(-s, s);
	}
	else
	{
		setSpeed(s, -s);
	}
	r = stepsToRotate(angle);
	waitForSteps(r);
	setSpeed(s, s);
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
	reportValue("Current Direction (steps)", (rCurrent));
	reportValue("Current Direction", RADtoDEG(angleToRotate(rCurrent)));
	reportValue("Number of states", j);
	reportValue("****END PROGRESS REPORT****", -1);
}

int avoidBoundary()
{	
	//int frontLeds[] = {0};
	//int rightLeds[] = {1, 2, 3};
	//int leftLeds[] = {5, 6, 7};
	
	//reportValue("Avoiding object IN", -1);
	int mDist;
	
	clearSteps();
	r = 0;

	int frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
	while (frontProx > 400)
	{
		//turn left until front prox doesn't detect object
		setSpeed(-s, s);
		frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
		wait(delayTimer);
	}

	r += e_get_steps_left();
	reportValue("rotated", RADtoDEG(angleToRotate(r)));
	
	int rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
	int leftProx =  (int)((e_get_prox(0) + e_get_prox(7) + e_get_prox(6) + e_get_prox(5))/4);
	while (rightProx > 300)
	{
		switch(rightProx)
		{
			case 300 ... 650: 
			{//go straight with the object on the right
				while ((rightProx < 650) && (rightProx > 300))
				{
					if(leftProx > 300)
					{//detect object on left, loop function to avoid
						reportValue("entering recursion loop", leftProx);
						avoidBoundary();
					}
					else
					{//go straight and check for mline
						setSpeed(s, s);
						h = e_get_steps_left();
						updateProgress();
						mDist = distToMline(x, y, xg, yg);
						//reportXY(x, y, mDist);
						//if (mDist < 50)
						//{
							//reportValue("ON MLINE, returning to main code", -1);
							//return 1;
						//}
					}
					//reportValue("linear movement", h);
					rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
					leftProx = (int)((e_get_prox(0) + e_get_prox(7) + e_get_prox(6) + e_get_prox(5))/4);
					wait(delayTimer);
				}
				break;
			}
			case 651 ... 2000:
			{//moving towards object, turn left slightly
				clearSteps();
				while (rightProx > 650)
				{
					setSpeed(-s, s);
					rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
					wait(delayTimer);
				}
				r += e_get_steps_left();
				reportValue("rotated", RADtoDEG(angleToRotate(r)));
				e_set_steps_left(0);
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
	
	clearSteps();
	leftProx = (int)((e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/3);
	while (leftProx < 150)
	{//turn right until object is on the right
		setSpeed(s, -s);
		leftProx = (int)((e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/3);
		wait(delayTimer);
	}
	r += e_get_steps_left();
	wait(delayTimer);
	setSpeed(s, s);
	progressReport();
	return 0;
}

void pathfinder()
{
	setGoal();
	while(1)
	{
		moveToGoal();
		e_set_steps_left(0);
		while (fp < 300 && q[i] < 95)
		{
			//fp = getProx(frontwide, 4);
			fp = (int)((e_get_prox(6) + e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/4);
			waitForSteps(50);
			h = e_get_steps_left();
			updateProgress();
			//reportValue("Frontwide", fp);
			reportValue("q[i]", q[i]);
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
			fp = (int)((e_get_prox(6) + e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/4);
			/*if (onMline)
			{
				progressReport();
				reportValue("ON MLINE, Main Prog", -1);
				reportValue("db", db);
				reportValue("d[i]", d[i]);
				reportValue("fp", fp);
			}*/
		} while
		(
			q[i] < 95 &&
			//q[i] == qb &&
			!(onMline && db > d[i] && fp < 300)
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
