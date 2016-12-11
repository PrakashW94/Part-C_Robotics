#ifndef _GLOBAL
#define _GLOBAL

#define BASE_SPEED 500

#define LEFT 0
#define RIGHT 1

// NO-OP message
#define MSG_NOP 0

// Keep in line with other robot during traversal.
#define MSG_SIDE_TRAVERSE 50

// Follow the robot emitting the message
#define MSG_FOLLOW 55

/*
* A structure to define the various states
* the epuck during the high level behaviour.
*/
struct Globals
{

	// The message state to emit from the epuck
	volatile int messageToEmit;
	
	// The side the robot is in the traverse phase.
	volatile int traverseSide;	

	// Wheel speeds (left/right);
	int speed[2];
};

// Global var
extern struct Globals global;

void initGlobal();

void setMessageToEmit( int msg );

void setTraverseSide( int side );

void switchTraverseSide();

void startSpeed();

#endif
