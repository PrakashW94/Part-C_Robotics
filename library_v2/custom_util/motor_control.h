#ifndef _MOTOR_CONTROL
#define _MOTOR_CONTROL


int normalise_speed( int speed );

int stepsOver( int steps );

void clearSteps();

void set_speed( int side, int speed );

void set_wheel_speeds( int left, int right );

void followWallOn( int side, int prox, int closest, int initial_speed );

void moveForwards( int speed, int steps );

void turn90DegreesTo( int side );

#endif

