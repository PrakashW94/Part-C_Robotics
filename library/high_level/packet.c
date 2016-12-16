#include "packet.h"

/*
* The functionality to transform packets and integers is here.
*
* The idea of the packet system is to allow us to utilize the 1-byte message limit of the IR communication.
* 
* Since we can only send one byte as an integer ( a value of 0-255 ), we need to try and define this as a useful message for communication.
* Therefore, we define different bits of the byte to mean different things, in attempt to create a high-level communication protocol.
*
* A packet is defined as the following from an integer (left to right position):
* ---------- 
*	Bit		|	Description
*	1-2		|	The command ( 0-3 )
*	3-8		|	The payload for that command ( 0-63 )
*
* The context of each message can be seen in the "messages.h" file.
*
* These commands are handled in the "receiver.c" file. 
*
*/


/*
* Convert a binary value to a said packet.
*/
void toPacket( Packet* packet, int bin )
{
	// XAND 1100 0000
	packet->command = ( bin & 0xC0 ) >> 6;
	
	// XAND 0011 1111
	packet->payload = bin & 0x3F;
}


/*
* Conver the packet to a binary value (int)
*/
int toBin( Packet packet )
{
	// Shift 6
	int command = packet.command << 6;
	int payload = packet.payload;
	
	return command + payload;
}


/*
* Convert a command and payload to a binary packet.
*
* Command is 0-4, Payload is 0-63.
*/
int toBinFromInts( int command, int payload )
{
	return ( command << 6 ) + payload;
}
