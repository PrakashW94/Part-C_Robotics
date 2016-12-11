#include "global.h"

struct Globals global;

void initGlobal( int side )	
{
	global.messageToEmit = MSG_NOP;
	global.traverseSide = side;
}

void setMessageToEmit( int msg )
{
	global.messageToEmit = msg;
}

void setTraverseSide( int side )
{
	global.traverseSide = side;
}

void switchTraverseSide()
{
	if( global.traverseSide == LEFT )
	{
		global.traverseSide == RIGHT;
	}	
	else
	{
		global.traverseSide == LEFT;
	}
}


