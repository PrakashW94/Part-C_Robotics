
/*
* Rotation 0 = 90 degrees right.
*
*/ 

/*
PI: Pi (duh)
Rd: Diameter of the robot (wheel axle) in metres
Wd: Diameter of the wheel in metres
*/

#define PI 3.14159
#define Rd 0.052
#define Wd 0.041 
 
void setSpeed(int l, int r)
{
	e_set_speed_left(l);
	global.speed[0] = l;
	e_set_speed_right(r);
	global.speed[1] = r;
}

double angleToRotate(double steps)
{
	return ((steps/1000)*Wd*PI*2*PI)/(Rd*PI);
}


int stepsToRotate(double angle)
{
	return floor((1000 * angle * Rd * PI)/(Wd * PI * 2 * PI));
}

double DEGtoRAD( double deg )
{
	return ( deg * PI ) / 180;
}

int RADtoDEG(double rad)
{
    return rad*180/PI;
}


void waitForSteps(int steps)
{
	int startSteps = e_get_steps_left();
	e_set_steps_left(0);
	
	int endSteps = e_get_steps_left() + steps;
	while( abs(e_get_steps_left()) < endSteps );
	
	e_set_steps_left(startSteps);
}


void clearSteps()
{
	e_set_steps_left(0);
	e_set_steps_right(0);

	global.rotationSteps = 0;
	global.linearSteps = 0;
}

void rotate( int angle )
{	
	double rad_angle = DEGtoRAD( angle );
	int stepsToWait = stepsToRotate( rad_angle );
	setSpeed( 500, 500 );
	waitForSteps( stepsToWait );
	
}


void performTranslation( int x, int y )
{
	double angle = angleFromGoal(x, y, getRobotPosX(), getRobotPosY() );
	
}

