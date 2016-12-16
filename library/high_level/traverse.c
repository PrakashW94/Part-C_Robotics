#include "btcom/btcom.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"

#include "custom_util/motor_control.h"

#include "high_level/global.h"
#include "high_level/findGreen.h"
#include "high_level/wall_follow.h"

#include "ircom/e_ad_conv.h"
#include "ircom/emitter.h"
#include "ircom/ircom.h"

#include "traverse.h"

#include "custom_util/utility.h"

#define NEAR_WALL_THRESHOLD 200
#define NEAR_OBJECT_THRESHOLD 150
#define APPROACH_WALL_THRESHOLD 80
#define WALL_CLOSEST_THRESHOLD 200

/**
* This file contains the functions to perform the "traverse" behaviour. 
* This behaviour is used to move around the arena until we find a valid object.
* 
* This behaviour is made up of 3 sub-behaviours which execute with the following precedence hierarchy:
*	1. Avoid wall.
*	2. Locate object.
* 	3. Cruise ( move in a linear line or slowly align with wall if really near. ).
*
* If locate object succeeds, then this behaviour exits as we want to transiiton to a new phase of the program.
*/

int steps_one_side[2] = { -1 , -1 };
int front_right_prox;
int front_left_prox;



/**
* Initialise the search behaviours.
*/ 
void initTraverse()
{
	global.phase = PHASE_SEARCH;
	
	// Send the follow packet to other robots now.
	setPacketToEmit( CMD_SET_STATE, STATE_FOLLOW );

	do
	{
	//	btcomSendInt( PHASE_SEARCH );
		traverse();
	}
	while( global.phase < PHASE_SEARCH_COMPLETE );		
}



/*
* Finish the search behaviour
*/
void endTraverse()
{
	set_wheel_speeds( 0,0 );
	global.phase = PHASE_SEARCH_COMPLETE;
}



/*
* Perform the traverse adjustment once.
*/
void traverse()
{	
	e_led_clear();

	if( global.phase < PHASE_SEARCH_COMPLETE )
	{
			
			// Get front proximity values.
			front_right_prox = e_get_calibrated_prox( 0 );
			front_left_prox = e_get_calibrated_prox( 7 );
			
			if( foundObject() == 1 )
			{
				btcomSendString( "Found object, finishing traverse. \r\n" );
				endTraverse();
			}
			else if( triggerAvoidWall() == 1 )
			{	
				btcomSendString( "Avoiding wall... \r\n" );
				avoidWall();
				btcomSendString( "Completed avoiding wall... \r\n" );
			}
			else
			{	
				cruise();
			}
			wait( 20000 );				
	}
}



/**
* Cruise Behaviour.
*
* This moves the robot in a linear direction.
*
* The behaviour also will move the robot slowly to align with a wall if it believes it is near one.
* This allows us to get an accurate reading for other potential behaviours.
*/
void cruise()
{
	int left_speed = BASE_SPEED;
	int right_speed = BASE_SPEED;
	
	if( approachingWall() == 1 )
	{
		BODY_LED = 1;

		// We need to adjust to get good readings for other behaviours.
		approachWall( &left_speed, &right_speed );
	}
	else
	{
		BODY_LED = 0;
	}	

	set_wheel_speeds( left_speed, right_speed );
}


/*
* Avoid the wall depending on the side we are on.
*/
void avoidWall()
{	
	// We record the steps for future object identification.
	if( steps_one_side[0] == -1 ||
		steps_one_side[1] == -1 )
	{
		steps_one_side[0] = e_get_steps_left();
		steps_one_side[1] = e_get_steps_right();
	}	

	clearSteps();


	switch( global.traverseDirection )
	{

		case RIGHT:
			turn90DegreesTo( LEFT );
			runWallFollow( RIGHT, 1000 ); 
			turn90DegreesTo( LEFT );
			break;

		case LEFT:
			turn90DegreesTo( RIGHT );
			runWallFollow( LEFT, 1000 ); 
			turn90DegreesTo( RIGHT );
			break;

	}
	btcomSendString( "Switching: " );
	switchTraverseSide();
	btcomSendInt( global.traverseDirection );
	
}



/**
* The criteria to attempt avoiding a wall.
*/
int triggerAvoidWall()
{
//	btcomSendString( "Prox: " );
//	btcomSendInt( front_prox );

	if( front_right_prox > NEAR_WALL_THRESHOLD && front_left_prox > NEAR_WALL_THRESHOLD )
	{
		return 1;
	}
	return 0;
}



/**
* "Face Wall" Behaviour.
*	This allows us to get a more accurate turn.
*/
int approachingWall()
{
	int approach_threshold = APPROACH_WALL_THRESHOLD;

	if( front_right_prox > approach_threshold ||
		front_left_prox > approach_threshold )
	{
		return 1;
	}
	return 0;
}



/*
* Slowly align the robot to face the wall, 
* in order to get a better reading for other behaviours.
*/
void approachWall( int *left_speed, int *right_speed )
{	
	int diff = front_right_prox - front_left_prox;

	// The right side is closer to the wall.
	if( diff > 0 )
	{
		// Left needs to go faster.
		*right_speed = *right_speed - ( diff * 2 ) ;
	}	
	// The left side is closer to the wall.
	else if( diff < 0 )
	{
		// Right needs to go faster.
		*left_speed = *left_speed - ( -diff * 2 );
	}
}


/**
* "Search" Behaviour
*/
int foundObject()
{
	if( front_right_prox > NEAR_OBJECT_THRESHOLD && front_left_prox > NEAR_OBJECT_THRESHOLD )
	{
		btcomSendString( "[TRAVERSE] Checking steps: \r\n");
		btcomSendInt( e_get_steps_left() );
		btcomSendInt( e_get_steps_right() );
		
		if( e_get_steps_left() < ( steps_one_side[0] - 200 ) && e_get_steps_right() < ( steps_one_side[1] - 200 ) )
		{	
			return 1;
		}	
	}

	return 0;
}
