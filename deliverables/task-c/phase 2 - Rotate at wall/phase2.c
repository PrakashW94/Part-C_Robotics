//to use this, include phase2.h and call rotateAtWall()

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "uart/e_uart_char.h"

#include "custom_util/utility.h"
#include "constants.c"

#include "math.h"
#include "string.h"
#include <stdio.h>

#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"

#include "agendacontrol.h"

/*
PI: Pi (duh)
Rd: Diameter of the robot (wheel axle) in metres
Wd: Diameter of the wheel in metres
*/

#define PI 3.14159
#define Rd 0.052
#define Wd 0.041 

int stepsToRotate(double angle)
{
	return floor((1000 * angle * Rd * PI)/(Wd * PI * 2 * PI));
}

double DEGtoRAD(double angle)
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

void setSpeed(int l, int r)
{
	e_set_speed_left(l);
	e_set_speed_right(r);
}

int direction = 1;
//direction is current movement direction 
//up = 1 (default)
//down = 0

void rotateAtWall()
{
	//function to rotate the robot 
	int stepsToTurn = stepsToRotate(DEGtoRAD(90));
	int lMod = 0;
	int rMod = 0;
	if(direction)
	{
		reportValue("moving up, rotating clockwise", -1);
		lMod = 1;
		rMod = -1;
		direction = 0;
	}
	else
	{
		reportValue("moving down, rotating anticlockwise", -1);
		lMod = -1;
		rMod = 1;
		direction = 1;
	}
	setSpeed(s*lMod, s*rMod);
	waitForSteps(stepsToTurn);
	
	setSpeed(s, s);
	waitForSteps(500);
	
	setSpeed(s*lMod, s*rMod);
	waitForSteps(stepsToTurn);
	
	setSpeed(s, s);
}
