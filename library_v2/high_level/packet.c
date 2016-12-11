#include "packet.h"

 /**
* A packet is defined as the following from an integer (left to right position):
* ---------- 
*	Bit		|	Description
*	1-2		|	The command ( 0-3 )
*	3-8		|	The payload for that command ( 0-63 )
*/

void toPacket( Packet* packet, int bin )
{
	// XAND 1100 0000
	packet->command = ( bin & 0xC0 ) >> 6;
	
	// XAND 0011 1111
	packet->payload = bin & 0x3F;
}

int toBin( Packet packet )
{
	// Shift 6
	int command = packet.command << 6;
	int payload = packet.payload;
	
	return command + payload;
}


// Command is 0-4, Payload is 0-63.
int toBinFromInts( int command, int payload )
{
	return ( command << 6 ) + payload;
}
