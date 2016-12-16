#include "btcom/btcom.h"

#include "custom_util/utility.h"

#include "high_level/global.h"
#include "high_level/packet.h"

#include "ircom.h"
#include "ircomSend.h"


/*
* This is a custom file we have added to handle emitting of our epuck packet protocol
*
* The emit function is intended to be run in an agenda so that message can be constantly emitted "asynchronously". 
*
* The emit funciton will constantly "emit" the packet set for the global state of that epuck.
* 
* We constantly emit messages as IR can sometimes send "bad" messages, so we need to ensure other robots 
* get a message at least once when we require it.
*
* There are also various other functions for speciifc message sending.
*
* For more information on the understanding of how our e puck protocol works. See the "high_level/packet.c" file comments.
*/
void emit()
{	
	// CHANGED
	if( ircomSendDone() == 1 )
	{
		//btcomSendString("Sending IR message... \r\n");
		
		int packet = toBinFromInts( global.commandToEmit, global.payloadToEmit );
		
		ircomSend( packet );
			
		btcomSendString("Sending...");
		
		//	while ( ircomSendDone() == 0 );
		
		//btcomSendString("Sent IR Message. \r\n");	
	}	
}


/**
* Send several master acks out quickly.
*/
void emitMasterAcks()
{
	btcomSendString( "Emitting master acks." );

	int i;
		
	for( i = 0; i < 10 ; i++ )
	{
		int packet = toBinFromInts( CMD_SET_STATE, STATE_ACK_MASTER );
	
		ircomSend( packet );

		while( ircomSendDone() == 0 );
		
		wait( 10000 );
	}	
}

/**
* Emit a message to other robots to follow this robot.
*/
void emitFollow()
{
	int packet = toBinFromInts( CMD_SET_STATE, STATE_FOLLOW );
	
	ircomSend( packet );

	while( ircomSendDone() == 0 );
}


/*
* Send the push box message.
* Several are sent to reduce possible erroneous messages.
*/
void emitPushBox()
{
	btcomSendString( "Emitting start box push." );

	int i;
		
	for( i = 0; i < 10 ; i++ )
	{
		int packet = toBinFromInts( CMD_SET_STATE, STATE_PUSH_BOX );
	
		ircomSend( packet );

		while( ircomSendDone() == 0 );
		
		wait( 10000 );
	}	
}

/*
* Send the current X and Y postion of the robot in its space.
*/
void emitPos()
{	
	if( global.robot_pos[0] < 0 || global.robot_pos[1] < -1 )
	{	
		// do nothing
		return;
	}

	int packetX = toBinFromInts( CMD_BROADCAST_POS_X, global.robot_pos[0] );
	int packetY = toBinFromInts( CMD_BROADCAST_POS_Y, global.robot_pos[1] );
	
	ircomSend( packetX );
	while ( ircomSendDone() == 0 );
	
	ircomSend( packetY );
	while ( ircomSendDone() == 0 );
}
