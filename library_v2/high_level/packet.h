/**
* The packet file.
*
* Allows functionality for packet serialisation.
*/
#ifndef _PACKET
#define _PACKET 

typedef struct
{	
	int command;
	int payload;
} Packet;

void toPacket( Packet* packet, int bin );

int toBin( Packet packet );

// Command is 0-4, Payload is 0-63.
int toBinFromInts( int command, int payload );


#endif
