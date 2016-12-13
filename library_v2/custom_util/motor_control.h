#ifndef _MOTOR_CONTROL
#define _MOTOR_CONTROL


int normalise_speed( int speed );

int stepsOver( int steps );

void clearSteps();

void set_speed( int side, int speed );

void set_wheel_speeds( int left, int right );

<<<<<<< HEAD
void followWallOn( int side, int prox, int closest, int initial_speed );

=======
>>>>>>> 72b18d279e37cdb4884232228eb6abe0dd67a99f
void moveForwards( int speed, int steps );

void turn90DegreesTo( int side );

#endif

