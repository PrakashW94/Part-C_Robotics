#include "e_epuck_ports.h"
#include "e_init_port.h"
#include "e_motors.h"

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
void rotateClockwise( int steps )
{
	clearSteps();

	BODY_LED = 1;

	e_set_speed_left( 500 );
	e_set_speed_right( -500 );

	waitForSteps( steps );

	BODY_LED = 0;
}



void moveForwards( int steps )
{
	clearSteps();

	e_set_speed_left( 300 );
	e_set_speed_right( 300 );

	waitForSteps( steps );
}
