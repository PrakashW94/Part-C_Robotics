#include "btcom/btcom.h"

#include "custom_util/motor_control.h"

#include "high_level/global.h"
#include "high_level/packet.h"
#include "high_level/init.h"

#include "motor_led/advance_one_timer/e_motors.h"

#include "ircom.h"
#include "ircomReceive.h"
#include "ircomUtil.h"

#include "math.h"

int sensor_sides[2][4] = {	{ 0,1,2,3 },
						 	{ 7,6,5,4 } };

int traverse_weights[4] = { 200,100,0,150 };


/**
* Move robot speed to near the sensor that message was received from.
*
* This function is braitenberg-style. This is meant to emulate "love"/following.
*/
void moveToSensor( int base_speed, IrcomMessage imsg )
{
	//btcomSendString( "Following \r\n" );
	
	int weights[8] = { 50,100,300,400,400,300,100,50 };
	float CLOSEST_DISTANCE = 8.5;

	double left_speed = base_speed;
	double right_speed = base_speed;
	
	double distance = imsg.distance;
	double speed_scale;
	
	int receivingSensor = imsg.receivingSensor;

	btcomSendString( "\r\n --> DISTANCE: \r\n" );
	btcomSendInt( distance ); 

	if( distance < CLOSEST_DISTANCE )
	{
		speed_scale = 0;
	}
	else
	{	
		double diff = distance - CLOSEST_DISTANCE;
		speed_scale = diff * 0.2;
	}

	// Right Side
	if( imsg.receivingSensor <= 3 )
	{
		left_speed += weights[receivingSensor];
		right_speed -= weights[receivingSensor];
	}
	// Left Side
	else
	{
		right_speed += weights[receivingSensor];
		left_speed -= weights[receivingSensor];	
	}

	right_speed *= speed_scale;
	left_speed *= speed_scale;
		
	int i_right_speed = normalise_speed( floor( right_speed ) );
	int i_left_speed = normalise_speed( floor( left_speed ) );
	
	set_wheel_speeds( i_left_speed, i_right_speed );
}


/**
* Set the speed of each robot during side-by-side traversal.
*
* The idea of this function is to keep the two robots roughly aligned.
*/
void setSideTraverseSpeed( int base_speed, IrcomMessage imsg )
{	
	int i, j;
	
	float CLOSEST_DISTANCE = 12.0;
	float FURTHEST_DISTANCE = 14.0;

	int left_speed = 0;
	int right_speed = 0;

	for( i = 0; i < 2; i++ )
	{	
		for( j = 0; j < 4; j++ )
		{
			if( imsg.receivingSensor == sensor_sides[i][j] )
			{
				left_speed = base_speed + traverse_weights[j];
				right_speed = base_speed + traverse_weights[j];
				
				if( imsg.distance > FURTHEST_DISTANCE )
				{		
					if( global.traverseDirection == LEFT )
					{
						left_speed = base_speed + 200;
						right_speed = base_speed - 200;
					}
					else
					{	
						left_speed = base_speed - 200;
						right_speed = base_speed + 200;
					}
				}
			}
			else if( imsg.distance < CLOSEST_DISTANCE )
			{
				if( global.traverseDirection == LEFT )
				{
					left_speed = base_speed - 200;
					right_speed = base_speed + 200;
				}
				else
				{	
					left_speed = base_speed + 200;
					right_speed = base_speed - 200;
				}	
						
			}			
		}
	}

	btcomSendString("Wheels: \r\n");
	btcomSendInt( left_speed );
	btcomSendInt( right_speed );

	set_wheel_speeds( left_speed, right_speed );
}

/**
* Set the robots position.
* The origin is always the master robot. This should be the bottom robot.
* 
* Robots should start side-by-side, facing east
*/
void setOrigins( int isMaster )
{
	if( isMaster == 1 )
	{
		setRobotPos( 0, 0 );
		setOtherRobotPos( 0, 100 );
	}
	else
	{
		setRobotPos( 0, 100 );
		setOtherRobotPos( 0, 0 );	
	}
}


// Only able to ack state requests
void processAck()
{	
	switch( global.payloadToEmit )
	{
		case STATE_PROPOSE_MASTER:	
			if( global.phase != PHASE_INIT_COMPLETE )
			{
				btcomSendString( "Master ACK received by other epuck. \r\n" );
				global.isMaster = 1;
				setRobotPos( 0, 0 );
				setOtherRobotPos( 0, 100 );
				global.phase = PHASE_INIT_COMPLETE;
			}
			else
			{
				btcomSendString( "Init already completed." );
			}
			break;
			

	}
}


void processStateChange( IrcomMessage imsg, Packet packet )
{	
	// btcomSendInt( packet.payload );

	switch( packet.payload )
	{	
		case STATE_NOP:
			asm("nop");
			break;
		
		case STATE_ACK:
		case STATE_ACK_MASTER:
			processAck();
			break;
		
		case STATE_TEST_SIDE_FOLLOW:
			setSideTraverseSpeed( BASE_SPEED, imsg );
			break;
		case STATE_SIDE_FOLLOW:
			if( global.payloadToEmit == STATE_ACK_MASTER )
			{
				global.payloadToEmit == STATE_NOP;
			}
			
			if( global.phase < PHASE_SEARCH )
			{
				global.phase = PHASE_SEARCH;
				initSideFollow();
			}	
			break;
		
		
		case STATE_PROPOSE_MASTER:
			if( global.masterProposed == 0 )
			{	
				global.masterProposed = 1;
				global.isMaster = 0;
				setRobotPos( 0, 100 );
				setOtherRobotPos( 0, 0 );
				setPacketToEmit( CMD_SET_STATE, STATE_ACK_MASTER );
				global.phase = PHASE_INIT_COMPLETE;
				btcomSendString( "Master proposed. \r\n" );
				btcomSendString( "Given some time to send ack..." );
			}
			btcomSendString( "Master already proposed. \r\n" );
			break;

		// Move to direction signal was received from.
		case STATE_FOLLOW:
			moveToSensor( BASE_SPEED, imsg );
			break;
			
		default:
			btcomSendString( "Unrecognized state change message:\r\n" );
			btcomSendInt( packet.payload );
			break;
					
	}
}

/*
* Process a received message.
*/
void process( IrcomMessage imsg )
{	
	Packet packet;
	toPacket( &packet, imsg.value);
	
/*	
	btcomSendString( "=== PACKET === \r\n" );	
	btcomSendInt( packet.command );
	btcomSendInt( packet.payload );
	btcomSendString( "============== \r\n" );
*/

	switch( packet.command )
	{
		case CMD_SET_STATE:
			processStateChange( imsg, packet );
			break;
		case CMD_BROADCAST_POS_X:
			btcomSendString( "Got X broadcast \r\n" );
			global.other_robot_pos[0] = packet.payload;
			break;
		case CMD_BROADCAST_POS_Y:
			btcomSendString( "Got Y broadcast \r\n" );
			global.other_robot_pos[1] = packet.payload;
			break;
		case CMD_FINISH:
			btcomSendString( "Got finish messsage \r\n" );
			break;
	}	
}


/**
* Set motors to be basic speed defined in
* global header file.
*/
void startSpeed()
{
	e_set_speed_left( BASE_SPEED );
	e_set_speed_right( BASE_SPEED );
}


/**
* Check to see if a message has been received.
*/
void receive()
{
	int error;

	IrcomMessage imsg;

	ircomPopMessage( &imsg );
	
	error = imsg.error;
	
	if( error == 0 )
	{	
		process( imsg );
	}
}
