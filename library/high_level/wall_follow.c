#include "btcom/btcom.h"

#include "custom_util/utility.h"
#include "custom_util/motor_control.h"

#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "high_level/global.h"

#include "ircom/e_ad_conv.h"

#include "wall_follow.h"


#define LEFT_FOLLOW			0		// behaviors IDs	
#define RIGHT_FOLLOW		1 

#define NB_SENSORS          8		// number of sensors
#define BIAS_SPEED      	200		// robot bias speed
#define SENSOR_THRESHOLD	300		// discount sensor noise below threshold
#define MAXSPEED 			800		// maximum robot speed

/**
* This file contains the "follow wall" functionality.
*
* This allows the epuck to follow alongside a wall.
*
* We use this as part of the "search" behaviour.
*/

int follow_sensorzero[8];
int follow_weightleft[8] = {-10,-10,-5,0,0,5,10,10};
int follow_weightright[8] = {10,10,5,0,0,-5,-10,-10};


/*
* Calibrate the sensors when we are near the wall.
* This allows us to get accurate readings of what we want to try and maintain
* during movement aside the wall.
*/
void follow_sensor_calibrate() 
{
	btcomSendString( "[WALL_FOLLOW] Starting calibration. \r\n" );

	// Iterator Variables
	int i, j;
	
	// Proximity sensor values.
	long sensor[8];

	for (i=0; i<8; i++) 
	{
		sensor[i]=0;
	}
	
	for ( j = 0; j < 32; j++ ) 
	{
		for (i=0; i<8; i++) 
		{
			sensor[i] += e_get_prox( i );
		}
		wait( 10000 );
	}

	for ( i = 0; i < 8; i++ ) 
	{
		follow_sensorzero[i] = ( sensor[i] >> 5 );
		btcomSendInt( follow_sensorzero[i] );
	}

	btcomSendString( "[WALL_FOLLOW] Calibration done. \r\n" );
	wait(100000);
}



/* 
* read sensor prox and return values in a int array
* return values from 0x0000 to 0x00FF (from 0 to 255)
*/
void followGetSensorValues(int *sensorTable) 
{
	unsigned int i;
	
	for ( i = 0; i < NB_SENSORS; i++ ) 
	{
		sensorTable[i] = e_get_prox( i ) - follow_sensorzero[i];
	}		
}



/*
* set robot speed
* speeds: from -MAXSPEED to MAXSPEED
*/
void followsetSpeed( int LeftSpeed, int RightSpeed ) 
{
	if ( LeftSpeed < -MAXSPEED ) 
	{
		LeftSpeed = -MAXSPEED;
	}
	if ( LeftSpeed >  MAXSPEED ) 
	{
		LeftSpeed =  MAXSPEED;
	}
	if ( RightSpeed < -MAXSPEED ) 
	{
		RightSpeed = -MAXSPEED;
	}
	if ( RightSpeed >  MAXSPEED ) 
	{
		RightSpeed =  MAXSPEED; 
	}
	set_wheel_speeds( LeftSpeed, RightSpeed ); 
}



/*
* Run the wall follow behaviour for so many steps,
* and with understanding of what side the wall is on.
*/
void runWallFollow( int side, int stepsToFollowFor ) 
{
	set_wheel_speeds( 0, 0 );

	int leftwheel, rightwheel;		// motor speed left and right
	int distances[NB_SENSORS];		// array keeping the distance sensor readings
	int i;							// FOR-loop counters
	int gostraight;
	int loopcount;
	
	btcomSendString( "[WALL_FOLLOW] Begin." );

	follow_sensor_calibrate();
	
	loopcount = 0;

	clearSteps();

	while( stepsOver( stepsToFollowFor ) == 0 ) 
	{
		followGetSensorValues( distances ); // read sensor values

		gostraight = 0;

		// Following RIGHT wall.
		if ( ( side == RIGHT  ) ) 
		{
			e_set_led( 6, 0 );
			e_set_led( 2, 1 );
	
			for ( i = 0; i < 8; i++ ) 
			{
				if( distances[i] > 50 ) 
				{ 
					break;
				}
			}
		
			if ( i == 8 )
			{
				gostraight = 1;
			}
			else 
			{
				follow_weightleft[0] =- 10;
				follow_weightleft[7] =- 10;
				follow_weightright[0] = 10;
				follow_weightright[7] = 10;
				if ( distances[2] > 300 ) 
				{
					distances[1] -= 200;
					distances[2] -= 600;
					distances[3] -= 100;
				}
			}
		} 
		// Following LEFT wall
		else
		{
			e_set_led( 6, 1 );
			e_set_led( 2, 0 );

			for ( i = 0; i < 8; i++ ) 
			{
				if ( distances[i] > 50 )
				{
					break;
				}
			}
	
			if ( i == 8 ) 
			{
				gostraight = 1;
			} 
			else 
			{
				follow_weightleft[0] = 10;
				follow_weightleft[7] = 10;
				follow_weightright[0] =- 10;
				follow_weightright[7] =- 10;
				if ( distances[5] > 300 ) 
				{
					distances[4] -= 100;
					distances[5] -= 600;
					distances[6] -= 200;
				}
			}
		}

		leftwheel = BIAS_SPEED;
		rightwheel = BIAS_SPEED;
		if ( gostraight == 0 ) 
		{
			for ( i = 0; i < 8; i++ ) 
			{
				leftwheel += follow_weightleft[i] * ( distances[i] >> 4 );
				rightwheel += follow_weightright[i] * ( distances[i] >> 4 );
			}
		}

		// set robot speed
		followsetSpeed( leftwheel, rightwheel );

		wait(15000);
	}	
}
