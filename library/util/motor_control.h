void waitForSteps(int steps);
void clearSteps();

void rotateClockwiseDegrees( int degrees );
void rotateClockwise( int steps );
void rotateAntiClockwiseDegrees( int degrees );
void rotateAntiClockwise(int steps);

void moveForwards( int steps );
void moveToPoint(int stepsRight, int stepsForward);

void setGoal(int x, int y);
void moveToGoal();

#ifndef _MOTOR_CONTROL
#define _MOTOR_CONTROL


int normalise_speed( int speed );

void set_speed( int side, int speed );


#endif
