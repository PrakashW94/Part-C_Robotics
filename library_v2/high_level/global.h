#ifndef _GLOBAL
#define _GLOBAL

#define BASE_SPEED 400

#define LEFT 0
#define RIGHT 1

// NO-OP message
#define MSG_NOP 0

// Keep in line with other robot during traversal.
#define MSG_SIDE_TRAVERSE 50

// Follow the robot emitting the message
#define MSG_FOLLOW 55

/**
* High level phases
*/
#define PHASE_INIT_START 0
#define PHASE_INIT_COMPLETE 1

#define PHASE_SEARCH 2
#define PHASE_SEARCH_COMPLETE 3

#define PHASE_BOX_FOLLOW 4
#define PHASE_BOX_FOLLOW_COMPLETE 5

#define PHASE_FINISH 10


/*
* A structure to define the various states
* the epuck during the high level behaviour.
*/
struct Globals
{
	// The packet info to emit from the epuck.
	volatile int commandToEmit;
	volatile int payloadToEmit;
	
	// Program phase we are in 
	volatile int phase;

	// Defines whether epuck is master or not
	volatile int masterProposed;
	volatile int isMaster;

	// The side the robot is in the traverse phase.
	volatile int traverseDirection;	
	
	/*
	* Robot Localization
	*/
	volatile int speed[2]; // Wheel speeds (left/right);
	volatile int robot_pos[2]; 	// Robot positions in [x,y] ( stored as steps ).
	
	volatile int totalRotationSteps;
	volatile int totalLinearSteps;

	volatile int rotationSteps;
	volatile int linearSteps; 

	volatile int other_robot_pos[2]; // Other Robot positions in [x,y] ( stored as steps ).
};

// Global var
extern volatile struct Globals global;

void initGlobal();

void setPacketToEmit( int command, int payload );

void setTraverseDirection( int side );

void switchTraverseSide();

void setRobotPos( int x, int y );
int getRobotPosX();
int getRobotPosY();

void setOtherRobotPos( int x, int y );
int getOtherRobotPosX();
int getOtherRobotPosY();

void startSpeed();

#include "high_level/packet.h"

#endif
