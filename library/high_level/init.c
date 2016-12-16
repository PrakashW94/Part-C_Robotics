/**
* This file is used to intialise the high level behaviour.
*/

#include "stdlib.h"

#include "btcom/btcom.h" 

#include "custom_util/utility.h"
#include "custom_util/motor_control.h"

#include "high_level/global.h"
#include "high_level/packet.h" 
#include "high_level/traverse.h"
#include "high_level/positionAroundObject.h"
#include "high_level/boxPush.h"

#include "ircom/ircom.h"

#include "ircom/emitter.h"
#include "ircom/receiver.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"

/**
* Initialise the environment for the high level behaviour on this robot.
*
* The initialisation changes depending on the direction that we are starting.
*/
void init( int direction )
{
	// Initialise global vars
	initGlobal( direction );

	// Start agenda processing
	e_start_agendas_processing();

	// Prepare IR for comms.
	//e_calibrate_ir(); 
    ircomStart();
	ircomEnableContinuousListening();
    ircomListen();

	// Prepare comms agenda.
	// Emit needs to be long (10000) for follow msg to be received by follower.
	e_activate_agenda( emit, 15000 );
//	e_activate_agenda( emitPos, 50000 );	
	e_activate_agenda( receive, 1000 );			
}

/**
* Initialise any environment items that the master needs.
*/
void initMaster()
{
	e_destroy_agenda( emit );
	e_activate_agenda( emit, 3000 );

	e_destroy_agenda( receive );
	e_activate_agenda( receive, 10000 );
}


/**
* Initialise any environment items that the follower needs.
*/
void initFollower()
{	
	e_destroy_agenda( receive );
	e_activate_agenda( receive, 1000 );	
}


/**
* Start the high level behaviour for the "leader" robot.
*
* The following sequence occurs in this behaviour:
*
* 1. Handshake with the "follower" robot and let it know that you are the leader.
* 2. Search for the box in performing a "lawnmower" traverse sequence of the arena.
*		a. Constantly emit "follow" messages via IR to the other robot whilst we perform this.
* 3. After finding an object to push, emit a message via IR to the other robot indicating we have found an object.
* 4. Traverse round the object to an optimal push location, so that the robot will be pushing the box south, back to near the origin location on init.
* 5. Wait for signal from the other robot to know it is also in position to push
* 6. Push the box
* 
* END 
*
* <---- E-PUCK COMMUNICATION ---->
*  Communication between messages is done via IR messages using the ircom library supplied by the e-puck company on their website.
*  See the packet.h and messages.h files for more information about this.
*
*/
void initHighLevelMaster( int isMaster, int direction )
{	
	init( direction );


	/**
	* Handshaking: 
	*	Keep emitting a PROPOSE_MASTER message until we get a MASTER_ACK back
	**/ 

	btcomSendString( "Setting SET_STATE -> PROPOSE_MASTER packet emit. \r\n" );
	setPacketToEmit( CMD_SET_STATE, STATE_PROPOSE_MASTER );	
	btcomSendString( "Waiting to handshake with follower... \r\n" );
	while( global.phase < PHASE_INIT_COMPLETE );	

	initMaster();
	btcomSendString( "Finished initialising. \r\n" );


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
	set_wheel_speeds( 0, 0 );
	wait( 1000000 );


	if( global.isMaster == 1 )
	{
		if ( global.traverseDirection == LEFT )
		{
			setPacketToEmit( CMD_SET_STATE, STATE_DIRECTION_LEFT );
		}
		else
		{
			setPacketToEmit( CMD_SET_STATE, STATE_DIRECTION_RIGHT );
		}
	}	

	wait( 1000000 );
	btcomSendString( "Start box follow... \r\n" );
	initBoxFollow( 1 );
	btcomSendString( "Waiting to complete box follow... \r\n" );
	while( global.phase < PHASE_BOX_FOLLOW_COMPLETE );
	btcomSendString( "Completed box follow." );	

	e_led_clear();
	while( global.phase < PHASE_PUSH_BOX );
	btcomSendString( "Starting box push");
	BODY_LED = 1;
	startBoxPush();
		
	while( global.phase != PHASE_PUSH_BOX_COMPLETE );
 
	btcomSendString( "Waiting to finish... \r\n" );
	while( global.phase != PHASE_FINISH );
}


/**
* Start the high level behaviour for the "follower" robot.
*
* The following sequence occurs in this behaviour:
*
* 1. Handshake with the "leader" robot.
* 2. Follow the leader robot by listening for follow messages emitted by its IR. 
* 3. Continuing following until the leader robot signals that it has found an object.
* 4. Wait for a bit ( roughly 10 seconds ) for the leader robot to move to the optimal push position on the box.
* 5. Move to the optimal push position on the box, next to the leader robot.
* 6. Signal the leader robot that you are in the push position.
* 7. Move the box 
* END
*
**/
void initHighLevelFollower()
{
	init( RIGHT );
	initFollower();
	
	/**
	* Handshaking: 
	*	Wait to receive a PROPOSE_MASTER message.
	*	On receiving this, send a MASTER_ACK message, and complete the handshake procedure.
	**/ 

	btcomSendString( "Waiting to handshake with master... \r\n" ); 		
	while( global.phase < PHASE_INIT_COMPLETE );
	btcomSendString( "Broadcasting ack for a while... \r\n" );
	emitMasterAcks(); 

	e_destroy_agenda( emit );	
	btcomSendString( "Completed handshake. \r\n" );
	

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
	set_wheel_speeds( 0, 0 );
	wait( 10000000 );
	moveToObject();
	initBoxFollow( 1 ); // init box follow for second epuck.	
	while( global.phase < PHASE_BOX_FOLLOW_COMPLETE );
	btcomSendString( "Completed box follow." );	
	
	emitPushBox();
	
	BODY_LED = 1;
	startBoxPush();
	while( global.phase < PHASE_PUSH_BOX_COMPLETE );

	btcomSendString( "Waiting to finish... \r\n" );
	while( global.phase != PHASE_FINISH );

	
	btcomSendString( "Finished \r\n" );

	LED7 = 1;
	while( 1 );
	 
}
