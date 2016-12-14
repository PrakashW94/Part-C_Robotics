/********************************************************************************

			Programm to follow a wall								      
			Version 1.0 août 2007				                          
			Michael Bonani, Jonathan Besuchet


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2004-2007 Michael Bonani, Jonathan Besuchet

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

**********************************************************************************/

/*! \file
 * \brief Follow a wall
 * \section sect1 Introduction
 * With this program, the e-puck will follow a wall.
 * 
 * \section sect_sound2 Playing the demo
 * First of all, move the selector to the position 2 and reset the e-puck.
 * \n The e-puck will now follow the first wall he finds. You can change
 * the side on which the e-puck must follow the wall with the selector.
 *
 * \section sect_sound3 Video of the demo
 * The video of this demo: http://www.youtube.com/watch?v=xaqpoQ_XGbU
 *
 * \author Code: Michael Bonani, Jonathan Besuchet \n Doc: Jonathan Besuchet
 */

#include "p30f6014A.h"

#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "btcom/btcom.h"

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_motors.h"

#include "uart/e_uart_char.h"

#include "ircom/e_ad_conv.h"

#include "high_level/global.h"
#include "high_level/wall_follow.h"

#include "custom_util/utility.h"
#include "custom_util/motor_control.h"

#include "positionAroundObject.h"

#define LEFT_FOLLOW			0		// behaviors IDs	
#define RIGHT_FOLLOW		1 

#define NB_SENSORS          8		// number of sensors
#define BIAS_SPEED      	350		// robot bias speed
#define SENSOR_THRESHOLD	300		// discount sensor noise below threshold
#define MAXSPEED 			400		// maximum robot speed


int position_sensorzero[8];
int position_weightleft[8] = {-10,-10,-5,0,0,5,10,10};
int position_weightright[8] = {10,10,5,0,0,-5,-10,-10};


void _waitForSteps(int steps)
{
 	int startSteps = e_get_steps_left();
 	e_set_steps_left(0);
 	
 	int endSteps = e_get_steps_left() + steps;
 	while( abs(e_get_steps_left()) < endSteps );
 	
	e_set_steps_left(startSteps);
}


/*! \breif Read the sensors proxymities
 * \param sensorTable Where the value must be stocked
 */
void _followGetSensorValues(int *sensorTable) {
	unsigned int i;

//	btcomSendString( "=== SENSORS ===" );
	for (i=0; i < NB_SENSORS; i++) {
		sensorTable[i] = e_get_calibrated_prox(i); //e_get_prox(i) - follow_sensorzero[i];
//		btcomSendInt( sensorTable[i] );
	}	
//	btcomSendString( "=============== \r\n\r\n" );	
}

void initBoxFollow( int useMasterCheck )
{
	global.phase = PHASE_BOX_FOLLOW;
	
	positionAroundObject( useMasterCheck );

	global.phase = PHASE_BOX_FOLLOW_COMPLETE;
}

/*! \brief The "main" function of the program */
void positionAroundObject( int useMasterCheck ) 
{
	int leftwheel, rightwheel;		// motor speed left and right
	int distances[NB_SENSORS];		// array keeping the distance sensor readings
	int i;							// FOR-loop counters
	int gostraight;

	int turningRight = 0;
	int finishedRight = 0;
	int turningLeft = 0;
	int finishedLeft = 0;
	int lineDist = 0;
	
	int firstRobot;

	// firstRobot = global.isMaster;
	if( useMasterCheck == 1 )
	{
		firstRobot = global.isMaster;
	}
	else
	{
		firstRobot = 1;
	}
	
	int movingDown = global.traverseDirection;

/*	if( movingDown == LEFT )
	{
		turn90DegreesTo( RIGHT );
	}
	else
	{
		turn90DegreesTo( LEFT );
	}
*/	
	
	while (1) {

		_followGetSensorValues(distances); // read sensor values

		gostraight=0;
		if ( movingDown == RIGHT ) 
		{
			for (i=0; i<8; i++) {
				if (distances[i]>50) {break;}
			}
			if (i==8) {
				gostraight=1;
			} else {
				position_weightleft[0]=-10;
				position_weightleft[7]=-10;
				position_weightright[0]=10;
				position_weightright[7]=10;
				if (distances[2]>300) {
					distances[1]-=200;
					distances[2]-=600;
					distances[3]-=100;
				}
			}

			leftwheel=BIAS_SPEED;
			rightwheel=BIAS_SPEED;

			if ( gostraight == 0 ) 
			{
				for (i=0; i<8; i++)
 				{
					leftwheel += position_weightleft[i] * ( distances[i] >> 4 );
					rightwheel += position_weightright[i] * ( distances[i] >> 4 );
				}
			}
	
			int currSteps = e_get_steps_left();
			int diff = leftwheel - rightwheel;
		
			//reportValue("Left", leftwheel );
			//reportValue("Right", rightwheel );
			//reportValue("Left-Right", diff );

		
			if ( diff > 350 )
			{
				turningRight = 1;
				if (finishedRight && abs(currSteps) > 400 && firstRobot)
				{
					// if first robot
					//reportValue("starting 2nd turn", 1);
					int lsteps = e_get_steps_left();
					int rsteps = e_get_steps_right();
					lineDist = sqrt(pow(lsteps, 2) + pow(rsteps, 2));
					//reportValue("left steps", lsteps);
					//reportValue("right steps", rsteps);
					//reportValue("lineDist", lineDist);
					finishedLeft = 0;
					followsetSpeed(0, 0);
					break;
				}				
			}
			else if (turningRight)
			{
				//reportValue("finished right", 1);

				clearSteps();			

				turningRight = 0;
				finishedRight = 1;		
			}
			else if (finishedRight && abs(currSteps) > 500 && !firstRobot)
			{
				followsetSpeed(0, 0);
				break;
			}		
			
		} 
		// Moving LEFT
		else 
		{			
			for (i=0; i<8; i++) {
				if (distances[i]>50) {break;}
			}
			if (i==8) {
				gostraight=1;
			} else {
				position_weightleft[0]=10;
				position_weightleft[7]=10;
				position_weightright[0]=-10;
				position_weightright[7]=-10;
				if (distances[5]>300) {
					distances[4]-=100;
					distances[5]-=600;
					distances[6]-=200;
				}
			}			

			leftwheel=BIAS_SPEED;
			rightwheel=BIAS_SPEED;
			if (gostraight==0) {
				for (i=0; i<8; i++) {
					leftwheel+=position_weightleft[i]*(distances[i]>>4);
					rightwheel+=position_weightright[i]*(distances[i]>>4);
				}
			}
	
			if (rightwheel < 0)
			{
				turningRight = 1;
			}
			else if (turningRight)
			{
				//reportValue("finished right", 1);
				turningRight = 0;
			}

			int currSteps = e_get_steps_left();
			if (rightwheel - leftwheel > 350 )
			{
				turningLeft = 1;
				if (finishedLeft && abs(currSteps) > 400 && firstRobot)
				{
					// if first robot
					//reportValue("starting 2nd turn", 1);
					int lsteps = e_get_steps_left();
					int rsteps = e_get_steps_right();
					lineDist = sqrt(pow(lsteps, 2) + pow(rsteps, 2));
					//reportValue("left steps", lsteps);
					//reportValue("right steps", rsteps);
					//reportValue("lineDist", lineDist);
					finishedLeft = 0;
					followsetSpeed(0, 0);
					break;
				}			
			}
			else if (turningLeft)
			{
				//reportValue("finished left", 1);

				e_set_steps_left(0);
				e_set_steps_right(0);			

				turningLeft = 0;
				finishedLeft = 1;		
			}
			else if (finishedLeft && abs(currSteps) > 500 && !firstRobot)
			{
				followsetSpeed(0, 0);
				break;
			}
		}		

		// set robot speed
		followsetSpeed(leftwheel, rightwheel);

		wait(15000);
	}	
	
	if (firstRobot)
	{
		// 	Get robot to move back 20% of box side length
		int firstRobotPos = lineDist * 0.2;
		followsetSpeed(-400, -400);		
		_waitForSteps(firstRobotPos);
		followsetSpeed(0, 0);	

		// Rotate to face box (roughly)
		if (movingDown)
		{
			followsetSpeed(300, -300);
		}
		else
		{
			followsetSpeed(-300, 300);
		}
		
		_waitForSteps(200);
		followsetSpeed(0, 0);

		// Move towards box
		followsetSpeed(200, 200);
		_followGetSensorValues(distances);
		while (distances[0] < 1400 && distances[7] < 1400)
		{
			_followGetSensorValues(distances);
			wait(15000);
		}
		followsetSpeed(0, 0);
		//reportValue("sensor0", distances[0]);
		//reportValue("sensor7", distances[7]);

		// Rotate towards box more exact
	/*	if (distances[7] > distances[0] + 500)
		{
			
			if (movingDown)
			{
				followsetSpeed(300, -300);
			}
			else
			{
				followsetSpeed(-300, 300);
			}
			_waitForSteps(100);
			followsetSpeed(0, 0);
		}
	*/
	}	
	else
	{
		// Second robot rotates 90 degrees to face box
		if (movingDown)
		{
			followsetSpeed(300, -300);
		}
		else
		{
			followsetSpeed(-300, 300);
		}
		_waitForSteps(333);
		followsetSpeed(0, 0);

		// Move towards box
		followsetSpeed(200, 200);
		_followGetSensorValues(distances);
		while (distances[0] < 1400 && distances[7] < 1400)
		{
			_followGetSensorValues(distances);
			wait(15000);
		}
		followsetSpeed(0, 0);
		//reportValue("sensor0", distances[0]);
		//reportValue("sensor7", distances[7]);
	}
}


void moveToObject()
{

	int SPEED = 400;
	int NEAR_OBJECT_THRESHOLD = 200;
	int APPROACH_OBJECT_THRESHOLD = 40;
	//int OBJECT_CLOSEST_THRESHOLD = 200;

	int found = 0;
	int left_speed = SPEED;
	int right_speed = SPEED;

	int front_right_prox = e_get_calibrated_prox( 0 );
	int front_left_prox = e_get_calibrated_prox( 7 );
	
	while( found == 0 )
	{
		if( front_right_prox > NEAR_OBJECT_THRESHOLD && front_left_prox > NEAR_OBJECT_THRESHOLD )
		{
			found = 1;
			break;
		}

		// If we are approaching an object, we want to move closer to it at a slow speed.
		if( front_right_prox > APPROACH_OBJECT_THRESHOLD || front_left_prox > APPROACH_OBJECT_THRESHOLD )
		{
			int diff = front_right_prox - front_left_prox;
	
			// Right prox is closer.
			if( diff > 0 )
			{
				// Left needs to go faster than right.
				right_speed = right_speed - ( diff * 2 ) ;
			}	
			// Left prox is closer.
			else if( diff < 0 )
			{
				// Right needs to go faster than left.
				left_speed = left_speed - ( -diff * 2 );
			}
		}
	} 
	set_wheel_speeds( 0, 0 );
}
