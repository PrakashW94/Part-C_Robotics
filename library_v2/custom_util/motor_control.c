#include "math.h"

#include "btcom/btcom.h"

#include "high_level/global.h" 

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#define PI 3.14159
#define Rd 0.052
#define Wd 0.041 

int stepsToRotate( double angle )
{
	return floor((1000 * angle * Rd * PI)/(Wd * PI * 2 * PI));
}

double DEGtoRAD(double angle)
{
    return ( angle * PI ) / 180;
}


int normalise_speed( int speed )
{
	if( speed > 1000 ) 
		speed = 1000;
	
	if( speed < -1000 )
		speed = -1000;

	return speed;
}

void set_speed( int side, int speed )
{
	speed = normalise_speed( speed );
		
	switch( side ) 
	{
		case LEFT:
				if( global.speed[0] != speed )
				{
					e_set_speed_left( speed );
					global.speed[0] = speed;
					//btcomSendString( "Changed left speed. \r\n" );
				}
				else
				{
					//btcomSendString( "Old speed" );
				}
				//	btcomSendString( "Left Speed - ");
				//	btcomSendInt( speed );
			break;

		case RIGHT:
				if( global.speed[1] != speed )
				{
					e_set_speed_right( speed );
					global.speed[1] = speed;
					//btcomSendString( "Changed right speed. \r\n" );
				}
				else
				{
					//btcomSendString( "Old speed" );
				}
				//	btcomSendString( "Right Speed - ");
				//	btcomSendInt( speed );
			break;
	}
}

void set_wheel_speeds( int left, int right )
{
	set_speed( LEFT, left );
	set_speed( RIGHT, right );
}

int stepsOver( int steps )
{
	// Below
	if( e_get_steps_left() < steps && e_get_steps_right() < steps )
	{
		return 0;
	}
	// Over
	return 1;
}

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


//  Rotates the robot anti-clockwise.
//  333 steps = 90 degrees
void rotateAntiClockwise( int steps )
{
	clearSteps();

//	BODY_LED = 1;
	
	set_wheel_speeds( -500, 500 );

	waitForSteps( steps );
}


//  Rotates the robot clockwise.
//  333 steps = 90 degrees
void rotateClockwise( int steps )
{	
	clearSteps();

//	BODY_LED = 1;

	set_wheel_speeds( 500, -500 );

	waitForSteps( steps );

//	BODY_LED = 0;
}


void rotateClockwiseDegrees( int degrees )
{
	// 1000 steps = 270 degrees
	// 1 step = 0.27 degrees;
	// 1 degree = 3.7037 degrees;

	rotateClockwise( degrees * 3.7037 );
}

void followWallOn( int side, int stepsTofollowFor, int prox, int closest, int initial_speed )
{
	clearSteps();

	int diff = closest - prox;

	switch( side )
	{
		case RIGHT:		
			// Closer than we want to be on right.
			if( diff < 0 )	
			{
				// Slow down left side to move away
				set_speed( LEFT, initial_speed - diff );
			}	
			// Further away than we want to be on left.
			else
			{
				// Slow down on the right side.
				set_speed( RIGHT, initial_speed - diff );
			}
			break;

		case LEFT:
			// Closer than we want to be on left.
			if( diff < 0 )	
			{
				// Slow down right side to move away
				set_speed( RIGHT, initial_speed - diff );
			}	
			// Further away than we want to be on right.
			else
			{
				// Slow down on the left side.
				set_speed( LEFT, initial_speed - diff );
			}
			break;
	}
}

void moveForwards( int speed, int steps )
{
	clearSteps();

	set_wheel_speeds( speed, speed );
	
	//	e_set_speed_left( speed );
	//e_set_speed_right( speed );

	waitForSteps( steps );
}

void turn90DegreesTo( int side )
{
	int angleSteps;
	angleSteps = stepsToRotate( DEGtoRAD( 90 ) );
	//angleSteps = 333;
	
	btcomSendString( "90 degree steps: " );
	btcomSendInt( angleSteps );

	switch( side )
	{	
		case RIGHT:
			rotateClockwise( angleSteps );
			break;
		case LEFT: 
			rotateAntiClockwise( angleSteps );
			break;		
	}
}

