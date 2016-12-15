/*
* This file will contain all the different messages that can be used
* within a Packet file.
*/

/**
* Command Options
*/

#define CMD_SET_STATE 0
#define CMD_BROADCAST_POS_X 1
#define CMD_BROADCAST_POS_Y 2
#define CMD_FINISH 3


/* 
* State Payload options
* Values 0 - 63
*/

// Defined no op state
#define STATE_NOP 0

// Handshaking states
#define STATE_ACK 1
#define STATE_NACK 2
#define STATE_PROPOSE_MASTER 3
#define STATE_ACK_MASTER 4
#define STATE_SETUP_COMPLETE 5

// Side Follow states
#define STATE_SIDE_FOLLOW 10 // Tell other robot to follow on side.
#define STATE_LEFT 11 // Tell other robot that you are left.
#define STATE_RIGHT 12 // Tell other robot that you are right.
#define STATE_TEST_SIDE_FOLLOW 19 // Follow Test


// Follow me state
#define STATE_FOLLOW 20 // Tell other robot to follow this robot


#define STATE_INIT_BOX_FOLLOW 30 // Tell other robot to start box follow.

#define STATE_DIRECTION_LEFT 40
#define STATE_DIRECTION_RIGHT 41

#define STATE_PUSH_BOX 50
