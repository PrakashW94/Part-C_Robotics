#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include "custom_util/constants.c"
#include "custom_util/terminal_reporting.h"
#include <math.h>

// Global Vars
double GOAL[] = {500, 500};
int currentDegrees = 0;
double currentPosition[] = {0, 0};

// Explicit Function Declarations
void waitForStepsAndDetect( int steps);

void waitForSteps( int steps )
{
	while( e_get_steps_left() < steps && e_get_steps_right() < steps )
	{

	}
}

void clearSteps()
{
	e_set_steps_left( 0 );
	e_set_steps_right( 0 );
}

void updateDegrees(int degrees)
{
	currentDegrees += degrees;

	if (currentDegrees < 0)
		currentDegrees += 360;
	else if (currentDegrees > 360)
		currentDegrees -= 360;
}

void updateCurrentPosition()
{
	double stepsAvg = (e_get_steps_left() + e_get_steps_right()) / 2;
	double currentAngle = currentDegrees * PI / 180;
	
	double newX = stepsAvg * sin(currentAngle);
	double newY = stepsAvg * cos(currentAngle);

	currentPosition[0] += newX;
	currentPosition[1] += newY;

	//reportValue("X: ", currentPosition[0]);
	//reportValue("Y: ", currentPosition[1]);

	//reportValue("GOALX", GOAL[0]);
	//reportValue("GOALY", GOAL[1]);

	// Lights up when it reaches GOAL, allowing for small margin of error
	if (currentPosition[0] > GOAL[0] - 50 && currentPosition[0] < GOAL[0] + 50
		&& currentPosition[1] > GOAL[1] - 50 && currentPosition[1] < GOAL[1] + 50)
	{
		LED0 = 1;
	}
	else
	{
		LED0 = 0;
	}

	// Light up LED when on m-line, for testing
	int val = detectMLine();
	if (val)
	{
		LED6 = 1;
	}
	else
	{
		LED6 = 0;
	}

	clearSteps();
}

//  Rotates the robot clockwise.
//  333 steps = 90 degrees
void rotateClockwise( float steps )
{
	clearSteps();

	e_set_speed_left( 500 );
	e_set_speed_right( -500 );

	waitForSteps( steps );
}

void rotateClockwiseDegrees( int degrees )
{
	// 1000 steps = 270 degrees
	// 1 step = 0.27 degrees;
	// 1 degree = 3.7037 degrees;
	rotateClockwise( degrees * 3.7037 );

	updateDegrees(degrees);
}

void rotateAntiClockwise( float steps )
{
	clearSteps();

	e_set_speed_left( -500 );
	e_set_speed_right( 500 );

	waitForSteps( steps );
}

void rotateAntiClockwiseDegrees( int degrees )
{
	// 1000 steps = 270 degrees
	// 1 step = 0.27 degrees;
	// 1 degree = 3.7037 degrees;
	rotateAntiClockwise( degrees * 3.7037 );
	
	updateDegrees(360 - degrees);
}

// 1 to use detection loop
void moveForwards( int steps, int detect )
{
	clearSteps();

	e_set_speed_left( 500 );
	e_set_speed_right( 500 );

	if (detect)
		waitForStepsAndDetect( steps );
	else
		waitForSteps(steps);

	updateCurrentPosition();
}

void move(int stepsRight, int stepsForward)
{
	// Pythagoras & Trig
	float a = pow(stepsRight, 2);
	float b = pow(stepsForward, 2);
	float h = sqrt(a + b);
	float angle = acos(stepsForward / h);
	angle = angle * 180 / PI;

	if (stepsRight < 0)
		angle = 360 - angle;

	// Take into account how we're currently oriented.
	angle -= currentDegrees;
	if (angle < 0)
		angle = 360 + angle;
	
	if (angle > 180)
	{
		rotateAntiClockwiseDegrees(360 - angle);	
	}
	else 
	{
		rotateClockwiseDegrees(angle);
	}

	moveForwards(h, 1);
}

void moveToPoint(int x, int y)
{
	int stepsRight = x - currentPosition[0];
	int stepsForward = y - currentPosition[1];

	move(stepsRight, stepsForward);
}

int getGoalX()
{
	return GOAL[0];
}

int getGoalY()
{
	return GOAL[1];
}

void setGoal(int x, int y)
{
	GOAL[0] = x;
	GOAL[1] = y;
}

void moveToGoal()
{
	moveToPoint(GOAL[0], GOAL[1]);
}

// Returns 1 for true if on m-line
// Returns 0 for false if not on m-line
// Allows for slight deviation
// Assumes start pos is always (0, 0)
int detectMLine()
{
	// Calculate vector lengths
	double currentDistance = sqrt(pow(currentPosition[0], 2) + pow(currentPosition[1], 2));
	double currentToGoal = sqrt(pow(currentPosition[0] - GOAL[0], 2) + pow(currentPosition[1] - GOAL[1], 2));
	double goalDistance = sqrt(pow(GOAL[0], 2) + pow(GOAL[1], 2));	

	if (currentDistance + currentToGoal > goalDistance - 50
		&& currentDistance + currentToGoal < goalDistance + 50)
		return 1;
	else
		return 0;
}

void waitForStepsAndDetect(int steps)
{
	int objectDetected = 0;
	while( e_get_steps_left() < steps && e_get_steps_right() < steps )
	{		
		objectDetected = updateDetector();
		if (objectDetected == 1)
		{			
			updateCurrentPosition();
			break;
		}	
	}
	
	if (objectDetected)
	{
		// Rotate anticlockwise until it is perpendicular to object
		int d = updateDetector();
		//while (d != 2)
		//{				
			//reportValue("Rotating", d);
			rotateAntiClockwiseDegrees(90);
			//d = updateDetector();
		//}

		traceObject();
		moveToGoal();	
	}
}

void traceObject()
{
	int d = updateDetector();
	while (d == 2)
	{		
		//reportValue("Moving Forwards", d);
		moveForwards(100, 0);
		d = updateDetector;
	}

	// Move forwards a little offset to clear the robot length
	//moveForwards(300, 1);
}

int normalise_speed( int speed )
{
	if( speed > 1000 ) 
		return 1000;
	
	if( speed < -1000 )
		return -1000;
}

int wheel_speeds[2] = { 0 , 0 };

void set_speed( int side, int speed )
{
	speed = normalise_speed( speed );
		
	switch( side ) 
	{
		case LEFT:
			if( wheel_speeds[0] != speed ){
				e_set_speed_left( speed );
				wheel_speeds[0] = speed;
			}
			break;
		case RIGHT:
			if( wheel_speeds[1] != speed ){
				e_set_speed_right( speed );
				wheel_speeds[1] = speed;
			}
			break;
		case BOTH:
			set_speed( LEFT, speed );
			set_speed( RIGHT, speed );
			break;
	}
}

void beginBug()
{
	
}