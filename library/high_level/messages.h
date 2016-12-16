/*
* This file will contain all the different messages that can be used
* within a Packet file.
*
* See "packet.c" and "receiver.c" for how these are used.
*/


/*
* Command Options
*
* Values 0-4
*/
#define CMD_SET_STATE 0				// Set a state message
#define CMD_BROADCAST_POS_X 1		// Broadcast e-puck X position ( not used now )
#define CMD_BROADCAST_POS_Y 2		// Broadcast e-puck Y position ( not used now )
#define CMD_FINISH 3				// Indicate we have finished.


/* 
* State Payload options
*
* Values 0 - 63
*/


#define STATE_NOP 0					// A no-op command

/*
* Handshaking states
*/
#define STATE_ACK 1					// Acknowledge a request
#define STATE_NACK 2				// Nak a request
#define STATE_PROPOSE_MASTER 3		// Tell other robots we want to be the master
#define STATE_ACK_MASTER 4			// Acknowledge the master
#define STATE_SETUP_COMPLETE 5		// Indicate setup is complete


/*
* Side Follow states
*/
#define STATE_SIDE_FOLLOW 10 		// Tell other robot to follow on side.
#define STATE_LEFT 11 				// Tell other robot that you are left.
#define STATE_RIGHT 12 				// Tell other robot that you are right.
#define STATE_TEST_SIDE_FOLLOW 19	// Follow Test


/*
* Follow me states
*/
#define STATE_FOLLOW 20 			// Tell other robot to follow this robot


/*
* Init box states
*/
#define STATE_INIT_BOX_FOLLOW 30 	// Tell other robot to start box follow.


/*
* Direction indication states
*/
#define STATE_DIRECTION_LEFT 40		// Tell other robot we are now traversing in the left direction
#define STATE_DIRECTION_RIGHT 41	// Tell other robot we are now traversing in the right direction


/*
* Push box states
*/
#define STATE_PUSH_BOX 50			// Indicate to other robot that we are now ready to push the box
