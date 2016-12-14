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
#define APPROACH_WALL_THRESHOLD 40
#define WALL_CLOSEST_THRESHOLD 200

int TURNING = 0;

int steps_one_side[2] = { -1 , -1 };

int front_right_prox;
int front_left_prox;

void disableCameraTimers();
void enableCameraTimers();

/**
* Initialise the search behaviours.
*/ 
void initTraverse()
{
	global.phase = PHASE_SEARCH;
	
	// Set up camera
	btcomSendString( "Starting camera..." ); 
	//initGreen();
	btcomSendString( "Camera started." );

	setPacketToEmit( CMD_SET_STATE, STATE_FOLLOW );

	do
	{
	//	btcomSendInt( PHASE_SEARCH );
		traverse();
	}
	while( global.phase < PHASE_SEARCH_COMPLETE );		
}

void endTraverse()
{
	set_wheel_speeds( 0,0 );
	global.phase = PHASE_SEARCH_COMPLETE;
}

void traverse()
{	
	e_led_clear();

	if( global.phase < PHASE_SEARCH_COMPLETE )
	{
			btcomSendString( "Runnning traverse.." );

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
				//btcomSendString( "Cruising. \r\n" );
				cruise();
			}
			wait( 20000 );				
	}
}

/**
* Cruise Behaviour
*/

void cruise()
{
	int left_speed = BASE_SPEED;
	int right_speed = BASE_SPEED;

	if( approachingWall() == 1 )
	{
		BODY_LED = 1 ;
		btcomSendString( "Approaching wall... \r\n" );
		
	//	LED2 = 1;
		
		btcomSendInt( left_speed );
		btcomSendInt( right_speed );
		btcomSendString( "After \r\n " );
		
		approachWall( &left_speed, &right_speed );
	
		btcomSendInt( left_speed );
		btcomSendInt( right_speed );
		btcomSendString( "Done.\r\n");
	}
	else
	{
		BODY_LED = 0;
	}	

	set_wheel_speeds( left_speed, right_speed );
}


/**
* "Avoid wall" Behaviour
*/
int stillTurning()
{	
	if( TURNING > 0 )
	{
		if( ( e_get_steps_left() < TURNING ) && ( e_get_steps_right() < TURNING ) )
		{	
			btcomSendString( "Turning" );
			return 1;
		}
		else
		{
			// Completed turn.
			btcomSendString( "Not Turning (2)" );
			TURNING = 0;
			return 0;
		}
	}
	// Not turning.
	else
	{	
		btcomSendString( "Not Turning." );
		return 0;
	}	
}


void followWall( int side, int stepsToFollowFor )
{
	int prox;
	int prox_value;

	if( side == LEFT )
	{
		// Left side prox.
		prox = 5;
	}
	else
	{
		// Right side prox.
		prox = 2;
	}
	
	clearSteps();
	
	while( stepsOver( stepsToFollowFor ) != 1 )
	{
		prox_value = e_get_calibrated_prox( prox );
		
		followWallOn( side, prox_value, WALL_CLOSEST_THRESHOLD , BASE_SPEED );
	}
}


void avoidWall()
{	
	btcomSendString( "Traverse Direction: " );
	btcomSendInt( global.traverseDirection );

	if( steps_one_side[0] == -1 ||
		steps_one_side[1] == -1 )
	{
		steps_one_side[0] = e_get_steps_left();
		steps_one_side[1] = e_get_steps_right();
		
		btcomSendString( "[TRAVERSE] Setting steps: \r\n");
		btcomSendInt( steps_one_side[0] );
		btcomSendInt( steps_one_side[1] );
	}	

	clearSteps();


	switch( global.traverseDirection )
	{

		case RIGHT:
			turn90DegreesTo( LEFT );
			runWallFollow( RIGHT, 1000 ); 
			//followWall( RIGHT, 1000 );
			//moveForwards( BASE_SPEED, 1000 );
			turn90DegreesTo( LEFT );
			break;

		case LEFT:
			turn90DegreesTo( RIGHT );
			runWallFollow( LEFT, 1000 ); 
			//followWall( LEFT, 1000 ); 
			//moveForwards( BASE_SPEED, 1000 );
			turn90DegreesTo( RIGHT );
			break;

	}
	btcomSendString( "Switching: " );
	switchTraverseSide();
	btcomSendInt( global.traverseDirection );
	
}

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


void approachWall( int *left_speed, int *right_speed )
{	
	int diff = front_right_prox - front_left_prox;

	// Right prox is closer.
	if( diff > 0 )
	{
		// Left needs to go faster.
		*right_speed = *right_speed - ( diff * 2 ) ;
		//*left_speed = *left_speed + diff ;
	}	
	// Left prox is closer.
	else if( diff < 0 )
	{
		// Right needs to go faster.
		*left_speed = *left_speed - ( -diff * 2 );
		//*right_speed = *right_speed + - diff;	
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
