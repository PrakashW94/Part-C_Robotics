#include "btcom/btcom.h"

#include "custom_util/motor_control.h"

#include "high_level/global.h"

#include "motor_led/advance_one_timer/e_motors.h"

#include "ircom.h"
#include "ircomReceive.h"
#include "ircomUtil.h"

#include "math.h"

int sensor_sides[2][4] = {	{ 0,1,2,3 },
						 	{ 7,6,5,4 } };

int traverse_weights[4] = { 200,100,0,150 };


/**
* Move robot speed to near the sensor that message was received from.
*
* This function is braitenberg-style. This is meant to emulate "love"/following.
*/
void moveToSensor( int base_speed, IrcomMessage imsg )
{
	int weights[8] = { 50,100,300,400,400,300,100,50 };
	float CLOSEST_DISTANCE = 8.0;

	double left_speed = base_speed;
	double right_speed = base_speed;
	
	double distance = imsg.distance;
	double speed_scale;
	
	int receivingSensor = imsg.receivingSensor;

	if( distance < CLOSEST_DISTANCE )
	{
		speed_scale = 0;
	}
	else
	{	
		double diff = distance - CLOSEST_DISTANCE;
		speed_scale = diff * 0.2;
	}

	// Right Side
	if( imsg.receivingSensor <= 3 )
	{
		left_speed += weights[receivingSensor];
		right_speed -= weights[receivingSensor];
	}
	// Left Side
	else
	{
		right_speed += weights[receivingSensor];
		left_speed -= weights[receivingSensor];	
	}

	right_speed *= speed_scale;
	left_speed *= speed_scale;
		
	int i_right_speed = normalise_speed( floor( right_speed ) );
	int i_left_speed = normalise_speed( floor( left_speed ) );
	
	set_wheel_speeds( i_left_speed, i_right_speed );
}


/**
* Set the speed of each robot during side-by-side traversal.
*
* The idea of this function is to keep the two robots roughly aligned.
*/
void setSideTraverseSpeed( int base_speed, IrcomMessage imsg )
{	
	int i, j;
	
	float CLOSEST_DISTANCE = 12.0;
	float FURTHEST_DISTANCE = 14.0;

	int left_speed = 0;
	int right_speed = 0;

	for( i = 0; i < 2; i++ )
	{	
		for( j = 0; j < 4; j++ )
		{
			if( imsg.receivingSensor == sensor_sides[i][j] )
			{
				left_speed = base_speed + traverse_weights[j];
				right_speed = base_speed + traverse_weights[j];
				
				if( imsg.distance > FURTHEST_DISTANCE )
				{
					if( j == 2 ) 
					{
						if( global.traverseSide == LEFT )
						{
							left_speed = base_speed + traverse_weights[j];
							right_speed = base_speed - traverse_weights[j];
						}
						else
						{	
							left_speed = base_speed - traverse_weights[j];
							right_speed = base_speed + traverse_weights[j];
						}
					}
				}
				else if( imsg.distance < CLOSEST_DISTANCE )
				{
					if( j == 2 )
					{
						if( global.traverseSide == LEFT )
						{
							left_speed = base_speed - traverse_weights[j];
							right_speed = base_speed + traverse_weights[j];
						}
						else
						{	
							left_speed = base_speed + traverse_weights[j];
							right_speed = base_speed - traverse_weights[j];
						}	
					}			
				}			
			}
	
		}
	}

	btcomSendString("Wheels: \r\n");
	btcomSendInt( left_speed );
	btcomSendInt( right_speed );

	set_wheel_speeds( left_speed, right_speed );
}


/*
* Process a received message.
*/
void process( IrcomMessage imsg )
{
	//btcomSendString( "Message received: " );
	//btcomSendInt( imsg.value );		

	switch( imsg.value )
	{
		// Change wheels to move to sensor.
		case MSG_FOLLOW:	
			moveToSensor( BASE_SPEED, imsg );		
			break;
		case MSG_SIDE_TRAVERSE:
			setSideTraverseSpeed( BASE_SPEED, imsg );
			break;
		default:
			btcomSendString( "Unknown message received. \r\n" );
			break;
			
	}

}


/**
* Set motors to be basic speed defined in
* global header file.
*/
void startSpeed()
{
	e_set_speed_left( BASE_SPEED );
	e_set_speed_right( BASE_SPEED );
}


/**
* Check to see if a message has been received.
*/
void receive()
{
	int error;

	IrcomMessage imsg;

	ircomPopMessage( &imsg );
	
	error = imsg.error;
	
	if( error == 0 )
	{	
		process( imsg );
	}
}
