#include "stdlib.h"

#include "btcom/btcom.h" 

#include "custom_util/utility.h"

#include "high_level/global.h"
#include "high_level/packet.h" 
#include "high_level/traverse.h"
#include "high_level/positionAroundObject.h"

#include "ircom/ircom.h"

#include "ircom/emitter.h"
#include "ircom/receiver.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"

void init()
{
	// Initialise global vars
	initGlobal( RIGHT );

	// Start agenda processing
	e_start_agendas_processing();

	// Prepare IR for comms.
	e_calibrate_ir(); 
    ircomStart();
	ircomEnableContinuousListening();
    ircomListen();

	// Prepare comms agenda.
	// Emit needs to be long (10000) for follow msg to be received by follower.
	e_activate_agenda( emit, 10000 );
//	e_activate_agenda( emitPos, 50000 );	
	e_activate_agenda( receive, 1000 );			
}

void initMaster()
{
	e_destroy_agenda( emit );
	e_activate_agenda( emit, 3000 );

	e_destroy_agenda( receive );
	e_activate_agenda( receive, 10000 );
}


void initFollower()
{	
	e_destroy_agenda( receive );
	e_activate_agenda( receive, 1000 );	
}


void initHighLevelMaster( int isMaster )
{	
	init();

	btcomSendString( "Setting SET_STATE -> PROPOSE_MASTER packet emit. \r\n" );
	setPacketToEmit( CMD_SET_STATE, STATE_INIT_BOX_FOLLOW );	
	
	btcomSendString( "Waiting to handshake with follower... \r\n" );
	while( global.phase < PHASE_INIT_COMPLETE );	
	btcomSendString( "Finished initialising. \r\n" );

	initMaster();

	/**
	* Phase 1 - Searching for box in "lawnmower" fashion.
	**/

	btcomSendString( "Start searching... \r\n" );
	initTraverse();
	btcomSendString( "Waiting to complete search phase... \r\n" );
	while( global.phase < PHASE_SEARCH_COMPLETE );
	btcomSendString( "Completed traversing. \r\n" );
	
	/**
	* Phase 2 - Found the box. Now to go round box.
	**/
	
	setPacketToEmit( CMD_SET_STATE, STATE_INIT_BOX_FOLLOW );
	wait( 200000 );
	btcomSendString( "Start box follow... \r\n" );
	initBoxFollow( 1 );
	btcomSendString( "Waiting to complete box follow... \r\n" );
	while( global.phase < PHASE_BOX_FOLLOW_COMPLETE );
	btcomSendString( "Completed box follow." );	

	BODY_LED = 1;
	btcomSendString( "Waiting to finish... \r\n" );
	while( global.phase != PHASE_FINISH );
}

void initHighLevelFollower()
{
	init();

	initFollower();
	
	btcomSendString( "Waiting to handshake with master... \r\n" ); 		
	while( global.phase < PHASE_INIT_COMPLETE );
	btcomSendString( "Broadcasting ack for a while... \r\n" );
	emitMasterAcks(); 

	btcomSendString( "Completed handshake. \r\n" );

	e_destroy_agenda( emit );	

	/**
	* Phase 1 - Do nothing but listen to follow commands.
	**/
	BODY_LED = 1;

	/**
	* Phase 2 - Wait for SET_STATE -> PHASE_BOX_FOLLOW
	**/
	while ( global.phase < PHASE_BOX_FOLLOW );

	/**
	* Phase 3 - Perform the "follower" box follow
	**/
	BODY_LED = 0;
	
	btcomSendString( "Received a phase box follow message.. Preparing box follow \r\n" );

	initBoxFollow( 1 ); // init box follow for second epuck.	
	while( global.phase < PHASE_BOX_FOLLOW_COMPLETE );
	btcomSendString( "Completed box follow." );	

	BODY_LED = 1;

	btcomSendString( "Waiting to finish... \r\n" );
	while( global.phase != PHASE_FINISH );

	
	btcomSendString( "Finished \r\n" );

	LED7 = 1;
	while( 1 );
	 
}

void followSide()
{	
	// Allow this much distance apart.
	int allowance = 100;
	
	int diffX = getOtherRobotPosX() - getRobotPosX();
		
	if( abs(diffX) < allowance )
	{
			// all is fine, in boundary
	}
	else
	{	
		// We are further right that the other robot
		if( diffX < 0 )
		{
			switch( global.traverseDirection )
			{	
				// Moving to left side
				case LEFT:
					// Speed up.
					break;
				case RIGHT:
					// Slow down.
					break;
			}		
		}
		// We are further left that the other robot
		else
		{
			switch( global.traverseDirection )
			{	
				// Moving to left side
				case LEFT:
					// Slow down.
					break;
				case RIGHT:
					// Speed up.
					break;;
			}
		}	
	}
}

/**
* Initialises side follow
*/
void initSideFollow()
{
	e_activate_agenda( followSide, 5000 );
}
