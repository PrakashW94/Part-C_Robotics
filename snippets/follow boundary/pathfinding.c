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

int avoidBoundary(int db);

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

void pathfinder()
{
	setGoal(0, 1000);
	while(1)
	{
		moveToGoal();
		clearSteps();
		while (fp < 400 && q[i] <= 98)
		{
			fp = (int)((e_get_prox(6) + e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/4);
			waitForSteps(10);
			h = e_get_steps_left();
			updateProgress();
			reportXY(x, y, RADtoDEG(angleToRotate(rCurrent)), q[i]);
		}
		if (q[i] >= 98)
		{
			e_set_led(8, 2);
			setSpeed(0,0);
			reportValue("Finished, Success!", -1);
			progressReport();
			while(1){}
		}
		
		progressReport();
		int qb = q[i];
		hCurrent = 0;
		double db = d[i];
		int onMline = 0;
		
		do 
		{
			onMline = avoidBoundary(db);
			fp = (int)((e_get_prox(6) + e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/4);
			
			//DEBUGGING
			if (onMline)
			{//Check whether or not the point on mline is improvement
				progressReport();
				reportValue("qb", qb);
				reportValue("qi", q[i]);
				
				//Check if qb is the same?
				//Check if d[i] has decreased by enough?
			}
			//END DEBUGGING
			
		} while
		(
			q[i] < 98 &&
			//q[i] == qb &&
			!(onMline && db > d[i] && fp < 300)
		);
		
		if (q[i] >= 98)
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

int avoidBoundary(int db)
{
	int frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
	clearSteps();
	while (frontProx > 300)
	{
		//turn left until front prox doesn't detect object
		setSpeed(-s, s);
		frontProx = (int)((e_get_prox(7) + e_get_prox(0))/2);
		wait(delayTimer);
	}

	r = e_get_steps_left();
	updateProgress();

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
					if(leftProx > 400)
					{//detect object on left, loop function to avoid
						reportValue("entering recursion loop", leftProx);
						avoidBoundary(db);
					}
					else
					{//go straight and check for mline
						setSpeed(s, s);
						h = e_get_steps_left();
						hCurrent += h;
						updateProgress();
						
						int mDist = distToMline(x, y, xg, yg);
						reportXY(x, y, checkMline, mDist);
						if (checkMline)
						{
							if (mDist < 10)
							{
								LED0 = 1;
								return 1;
							}
						}
						else
						{
							if (hCurrent > 300)
							{
								reportValue("hCurrent", hCurrent);
								reportValue("Now searching for m-line", -1);
								checkMline = 1;
							}
						}
					}
					rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
					leftProx = (int)((e_get_prox(0) + e_get_prox(7) + e_get_prox(6) + e_get_prox(5))/4);
					waitForSteps(10);
				}
				break;
			}
			case 651 ... 5000:
			{//moving towards object, turn left slightly
				clearSteps();
				while (rightProx > 650)
				{
					setSpeed(-s, s);
					rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
					wait(delayTimer);
				}
				r = e_get_steps_left();
				
				updateProgress();
				break;
			}
			default: 
			{
				break;
			}
		}
		rightProx = (int)((e_get_prox(0) + e_get_prox(1) + e_get_prox(2))/3);
	}
	
	leftProx = (int)((e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/3);
	clearSteps();
	while (leftProx < 250)
	{//turn right until object is on the right
		setSpeed(s, -s);
		leftProx = (int)((e_get_prox(7) + e_get_prox(0) + e_get_prox(1))/3);
		wait(delayTimer);
	}
	r = e_get_steps_left();
	updateProgress();
	clearSteps();
	setSpeed(s, s);
	h = e_get_steps_left();
	updateProgress();
	wait(delayTimer);
	return 0;
}
