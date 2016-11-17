#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_motors.h"
#include <math.h>

#define PI 3.14159265

int GOAL[] = {-500, -500};
int currentDegrees = 0;
double currentPosition[] = {0, 0};

int getGoalX()
{
	return GOAL[0];
}

int getGoalY()
{
	return GOAL[1];
}

void waitForSteps( int steps )
{
	while( e_get_steps_left() < steps && e_get_steps_right() < steps )
	{
		// do nothing..
	}

	// If Obstacle Found 
	
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
	currentPosition[0] += stepsAvg * cos(currentAngle);
	currentPosition[1] += stepsAvg * sin(currentAngle);

	// Lights up when it reaches GOAL, allowing for small margin of error
	if (currentPosition[0] > GOAL[0] - 50 && currentPosition[0] < GOAL[0] + 50
		&& currentPosition[1] > GOAL[1] - 50 && currentPosition[1] < GOAL[1] + 50)
	{
		LED0 = 1;
	}
}

//  Rotates the robot clockwise.
//  333 steps = 90 degrees
void rotateClockwise( float steps )
{
	clearSteps();

	BODY_LED = 1;

	e_set_speed_left( 500 );
	e_set_speed_right( -500 );

	waitForSteps( steps );

	BODY_LED = 0;
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

	BODY_LED = 1;

	e_set_speed_left( -500 );
	e_set_speed_right( 500 );

	waitForSteps( steps );

	BODY_LED = 0;
}

void rotateAntiClockwiseDegrees( int degrees )
{
	// 1000 steps = 270 degrees
	// 1 step = 0.27 degrees;
	// 1 degree = 3.7037 degrees;
	rotateAntiClockwise( degrees * 3.7037 );
	
	updateDegrees(360 - degrees);
}

void moveForwards( int steps )
{
	clearSteps();

	e_set_speed_left( 1000 );
	e_set_speed_right( 1000 );

	waitForSteps( steps );

	updateCurrentPosition();

	// Detect Obstacle
}

void moveToPoint(int stepsRight, int stepsForward)
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

	moveForwards(h);
}