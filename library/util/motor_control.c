#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include <math.h>

#define PI 3.14159265

void waitForSteps( int steps )
{
	while( e_get_steps_left() < steps && e_get_steps_right() < steps )
	{
		// do nothing..
	}
}

void clearSteps()
{
	e_set_steps_left( 0 );
	e_set_steps_right( 0 );
}

void rotateClockwiseDegrees( int degrees )
{
	// 1000 steps = 270 degrees
	// 1 step = 0.27 degrees;
	// 1 degree = 3.7037 degrees;

	rotateClockwise( degrees * 3.7037 );
}

//  Rotates the robot clockwise.
//  333 steps = 90 degrees
void rotateClockwise( float steps )
{
	clearSteps();

	BODY_LED = 1;

	e_set_speed_left( 500 );
	e_set_speed_right( -500 );

	waitForSteps( steps );

	BODY_LED = 0;
}

void rotateAntiClockwiseDegrees( int degrees )
{
	// 1000 steps = 270 degrees
	// 1 step = 0.27 degrees;
	// 1 degree = 3.7037 degrees;

	rotateAntiClockwise( degrees * 3.7037 );
}

void rotateAntiClockwise( float steps )
{
	clearSteps();

	BODY_LED = 1;

	e_set_speed_left( -500 );
	e_set_speed_right( 500 );

	waitForSteps( steps );

	BODY_LED = 0;
}

void moveForwards( int steps )
{
	clearSteps();

	e_set_speed_left( 1000 );
	e_set_speed_right( 1000 );

	waitForSteps( steps );
}

void moveToPoint(int stepsRight, int stepsForward)
{
	float a = pow(stepsRight, 2);
	float b = pow(stepsForward, 2);
	float h = sqrt(a + b);
	float angle = acos(stepsForward / h);
	angle = angle * 180 / PI;

	if (stepsRight < 0)
	{
		rotateAntiClockwiseDegrees(angle);
	}
	else
	{
		rotateClockwiseDegrees(angle);
	}
	
	moveForwards(h);
}

//This function gets the position that the selector is in. 
int getSelector() 
{
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}