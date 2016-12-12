#include "btcom/btcom.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"

#include "custom_util/motor_control.h"

#include "high_level/global.h"

#include "ircom/e_ad_conv.h"
#include "ircom/emitter.h"

#include "traverse.h"

#include "custom_util/utility.h"

int TURNING = 0;


/**
* Initialise the search behaviours.
*/ 
void initTraverse()
{
	global.phase = PHASE_SEARCH;

	e_activate_agenda( emitFollow, 10000 );

	do
	{
		traverse();
	}
	while( global.phase == PHASE_SEARCH );		
}

void endTraverse()
{
	set_wheel_speeds( 0,0 );
	global.phase = PHASE_SEARCH_COMPLETE;
}

void traverse()
{	
	e_led_clear();

	if( global.phase == PHASE_SEARCH )
	{
			if( triggerAvoidWall() == 1 )
				{	
					btcomSendString( "Avoiding wall... \r\n" );
					LED1 = 1;
					avoidWall();
					btcomSendString( "Completed avoiding wall... \r\n" );
				}
				else if( foundObject() == 1 )
				{	
					btcomSendString( "Found object, finishing traverse. \r\n" );
					LED2 = 1;
					endTraverse();
				}
				else
				{	
				//	btcomSendString( "Cruising. \r\n" );
					LED3 = 1;
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
	set_wheel_speeds( BASE_SPEED, BASE_SPEED );
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

void avoidWall()
{	
	btcomSendString( "Traverse Direction: " );
	btcomSendInt( global.traverseDirection );

	switch( global.traverseDirection )
	{
		case RIGHT:
			turn90DegreesTo( LEFT );
			moveForwards( BASE_SPEED, 1000 );
			turn90DegreesTo( LEFT );
			break;
		case LEFT:
			turn90DegreesTo( RIGHT );
			moveForwards( BASE_SPEED, 1000 );
			turn90DegreesTo( RIGHT );
			break;
	}
	btcomSendString( "Switching..." );
	switchTraverseSide();
	btcomSendInt( global.traverseDirection );
	
}

int triggerAvoidWall()
{	
	int front_prox = e_get_calibrated_prox(0);

	btcomSendString( "Prox: " );
	btcomSendInt( front_prox );

	if( front_prox > 300 )
	{
		return 1;
	}
	return 0;
}


/**
* "Search" Behaviour
*/
int foundObject()
{
	return 0;
}
