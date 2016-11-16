#ifndef _MOTOR_CONTROL
#define _MOTOR_CONTROL


int normalise_speed( int speed );
void set_speed( int side, int speed );
void rotateClockwiseDegrees( int degrees );
void rotateClockwise( int steps );
void moveForwards( int steps );

#endif

