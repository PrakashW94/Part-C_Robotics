#include "btcom/btcom.h"

#include "custom_util/utility.h"

#include "high_level/global.h"
#include "high_level/packet.h"

#include "ircom.h"
#include "ircomSend.h"

void emit()
{	
	//btcomSendString("Sending IR message... \r\n");
	int packet = toBinFromInts( global.commandToEmit, global.payloadToEmit );

	ircomSend( packet );
	
	//btcomSendString("Sending...");
	
	while ( ircomSendDone() == 0 );

	//btcomSendString("Sent IR Message. \r\n");	
}

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

void emitFollow()
{
	int packet = toBinFromInts( CMD_SET_STATE, STATE_FOLLOW );
	
	ircomSend( packet );

	while( ircomSendDone() == 0 );
}


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
