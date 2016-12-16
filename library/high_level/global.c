#include "btcom/btcom.h"

#include "global.h"
#include "packet.h"


/**
* This file is used to hold and modify any given state that we want to hold during
* the high level behaviour.
*
* The "global" variable is used in order to access this global state.
*/

volatile struct Globals global;

void initGlobal( int direction )	
{
	
	/*
	* Packet emit information.
	*/
	global.commandToEmit = CMD_SET_STATE;
	global.payloadToEmit = STATE_NOP;
	
	/*
	* Current program state.
	*/
	global.phase = PHASE_INIT_START;	

	global.masterProposed = 0;
	global.isMaster = 0;

	global.traverseDirection = direction;

	global.robot_pos[0] = -1;
	global.robot_pos[1] = -1;
	
	global.totalRotationSteps = 0;
	global.totalLinearSteps = 0;

	global.rotationSteps = 0;
	global.linearSteps = 0;

	global.other_robot_pos[0] = -1;
	global.other_robot_pos[1] = -1;	
}

void outputGlobals()
{
	btcomSendString(" ---- Globals ---- (p,isM,x,y) \r\n" );
	btcomSendInt( global.phase );
	btcomSendInt( global.isMaster );
	btcomSendInt( global.robot_pos[0] );
	btcomSendInt( global.robot_pos[1] );
}

void setPacketToEmit( int command, int payload )
{
	global.commandToEmit = command;
	global.payloadToEmit = payload;
}

void setTraverseSide( int direction )
{
	global.traverseDirection = direction;
}

void switchTraverseSide()
{
	if( global.traverseDirection == LEFT )
	{
		btcomSendString( "Switched to RIGHT (1)." );
		global.traverseDirection = RIGHT;
	}	
	else
	{	
		btcomSendString( "Switched to LEFT (0)." );
		global.traverseDirection = LEFT;
	}
}


void setRobotPos( int x, int y )
{
	global.robot_pos[0] = x;
	global.robot_pos[1] = y;
}

int getRobotPosX()
{
	return global.robot_pos[0];
}

int getRobotPosY()
{
	return global.robot_pos[1];
}

void setOtherRobotPos( int x, int y)
{
	global.other_robot_pos[0] = x;
	global.other_robot_pos[1] = y;
} 

int getOtherRobotPosX()
{
	return global.other_robot_pos[0];
}

int getOtherRobotPosY()
{
	return global.other_robot_pos[1];
}
