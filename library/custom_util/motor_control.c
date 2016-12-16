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

/**
* This file contains various utility functions to manage the motors.
*/


/*
* Get the steps to rotate in a given angle.
* Angle should be in radians.
*/ 
int stepsToRotate( double angle )
{
	return floor((1000 * angle * Rd * PI)/(Wd * PI * 2 * PI));
}



/*
* Covert degrees to radians.
*/
double DEGtoRAD(double angle)
{
    return ( angle * PI ) / 180;
}



/*
* Bound a speed inbetween the maximum supported values of the e-puck.
* This is 1000 forward and -1000 for reverse.
*/
int normalise_speed( int speed )
{
	if( speed > 1000 ) 
		speed = 1000;
	
	if( speed < -1000 )
		speed = -1000;

	return speed;
}


/*
* Set the speed of a side.
* 
* This code allows us to take a lazy approach of set wheel speed by 
* checking if the wheels are already running at the specified speed.
*/
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
			//	btcomSendString( "Changed left speed. \r\n" );
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
			//	btcomSendString( "Changed right speed. \r\n" );
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



/*
* "Lazily" set the wheel speeds for left and right.
*/
void set_wheel_speeds( int left, int right )
{
	set_speed( LEFT, left );
	set_speed( RIGHT, right );
}



/*
* An evaluation to see if the e puck step counter is over a given number of steps
*/ 
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



/*
* Wait for the puck steps counter to be over a given number of steps.
*/
void waitForSteps( int steps )
{
	while( e_get_steps_left() < steps && e_get_steps_right() < steps )
	{	
		// do nothing..
	}
}



/*
* Set the E-puck step counters to 0.
*/
void clearSteps()
{
	e_set_steps_left( 0 );
	e_set_steps_right( 0 );
}


/*  
* Rotates the robot anti-clockwise.
* 333 steps = 90 degrees
*/
void rotateAntiClockwise( int steps )
{
	clearSteps();

	set_wheel_speeds( -500, 500 );

	waitForSteps( steps );
}



/* 
* Rotates the robot clockwise.
* 333 steps = 90 degrees
*/
void rotateClockwise( int steps )
{	
	clearSteps();

	set_wheel_speeds( 500, -500 );

	waitForSteps( steps );
}



/*
* Rotate the robot clockwise a certain number of degrees.
* 1000 steps = 270 degrees
* 1 step = 0.27 degrees;
*
* 1 degree = 3.7037 steps;
*/
void rotateClockwiseDegrees( int degrees )
{
	rotateClockwise( degrees * 3.7037 );
}



/*
* Move forwards at a given speed for a given amount of steps.
*/
void moveForwards( int speed, int steps )
{
	clearSteps();

	set_wheel_speeds( speed, speed );
	
	//	e_set_speed_left( speed );
	//e_set_speed_right( speed );

	waitForSteps( steps );
}



/*
* Turn 90 degrees in a given direction.
*/
void turn90DegreesTo( int side )
{
	int angleSteps;
	//angleSteps = stepsToRotate( DEGtoRAD( 90 ) );
	angleSteps = 333;
	
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
	
	set_wheel_speeds( 0, 0 );
	clearSteps();
}

