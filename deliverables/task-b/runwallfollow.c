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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_motors.h"
#include "uart/e_uart_char.h"
#include "a_d/advance_ad_scan/e_ad_conv.h"
#include "a_d/advance_ad_scan/e_prox.h"

#include "utility.h"
#include "runwallfollow.h"


#define LEFT_FOLLOW			0		// behaviors IDs	
#define RIGHT_FOLLOW		1 

#define NB_SENSORS          8		// number of sensors
#define BIAS_SPEED      	350		// robot bias speed
#define SENSOR_THRESHOLD	300		// discount sensor noise below threshold
#define MAXSPEED 			800		// maximum robot speed


int follow_sensorzero[8];
int follow_weightleft[8] = {-10,-10,-5,0,0,5,10,10};
int follow_weightright[8] = {10,10,5,0,0,-5,-10,-10};

/*! \breif Callibrate the proxymity sensor */
/*void follow_sensor_calibrate() {
	int i, j;
	char buffer[80];
	long sensor[8];

	for (i=0; i<8; i++) {
		sensor[i]=0;
	}
	
	for (j=0; j<32; j++) {
		for (i=0; i<8; i++) {
			sensor[i]+=e_get_prox(i);
		}
		wait(10000);
	}

	for (i=0; i<8; i++) {
		follow_sensorzero[i]=(sensor[i]>>5);
		sprintf(buffer, "%d, ", follow_sensorzero[i]);
		e_send_uart1_char(buffer, strlen(buffer));
	}

	sprintf(buffer, " calibration done\r\n");
	e_send_uart1_char(buffer, strlen(buffer));
	wait(100000);
}*/

/*! \brief Looking at the selector value
 * \return The selector value from 0 to 15
 */
int followgetSelectorValue() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void reportValue(char* title, int value)
{
	char uartbuffer[100];
	sprintf(uartbuffer, "(%s - %d)\r\n", title, value);
	int length = strlen(uartbuffer);
	e_send_uart1_char(uartbuffer, length);
	while(e_uart1_sending()){}
}


/*! \breif Read the sensors proxymities
 * \param sensorTable Where the value must be stocked
 */
void followGetSensorValues(int *sensorTable) {
	unsigned int i;
	for (i=0; i < NB_SENSORS; i++) {
		sensorTable[i] = e_get_calibrated_prox(i); //e_get_prox(i) - follow_sensorzero[i];
	}		
}

/*! \brief Set robot speed */
void followsetSpeed(int LeftSpeed, int RightSpeed) {
	if (LeftSpeed < -MAXSPEED) {LeftSpeed = -MAXSPEED;}
	if (LeftSpeed >  MAXSPEED) {LeftSpeed =  MAXSPEED;}
	if (RightSpeed < -MAXSPEED) {RightSpeed = -MAXSPEED;}
	if (RightSpeed >  MAXSPEED) {RightSpeed =  MAXSPEED;}
	e_set_speed_left(LeftSpeed);
	e_set_speed_right(RightSpeed); 
}

/*! \brief The "main" function of the program */
void run_wallfollow() {
	int leftwheel, rightwheel;		// motor speed left and right
	int distances[NB_SENSORS];		// array keeping the distance sensor readings
	int i;							// FOR-loop counters
	int gostraight;
	int loopcount;
	unsigned char selector_change;

	int turningRight = 0;
	int turningLeft = 0;
	int finishedLeft = 0;
	
	loopcount=0;
	selector_change = !(followgetSelectorValue() & 0x0001);

	while (1) {
		followGetSensorValues(distances); // read sensor values

		gostraight=0;
		if ((followgetSelectorValue() & 0x0001) == RIGHT_FOLLOW) {
			//reportValue("rightfollow", -1);
			if(selector_change == LEFT_FOLLOW) {
				selector_change = RIGHT_FOLLOW;
			}  
			for (i=0; i<8; i++) {
				if (distances[i]>50) {break;}
			}
			if (i==8) {
				gostraight=1;
			} else {
				follow_weightleft[0]=-10;
				follow_weightleft[7]=-10;
				follow_weightright[0]=10;
				follow_weightright[7]=10;
				if (distances[2]>300) {
					distances[1]-=200;
					distances[2]-=600;
					distances[3]-=100;
				}
			}
		} else {
			//reportValue("leftfollow", -1);
			if(selector_change == RIGHT_FOLLOW) {
				selector_change = LEFT_FOLLOW;
			}
			for (i=0; i<8; i++) {
				if (distances[i]>50) {break;}
			}
			if (i==8) {
				gostraight=1;
			} else {
				follow_weightleft[0]=10;
				follow_weightleft[7]=10;
				follow_weightright[0]=-10;
				follow_weightright[7]=-10;
				if (distances[5]>300) {
					distances[4]-=100;
					distances[5]-=600;
					distances[6]-=200;
				}
			}
		}

		leftwheel=BIAS_SPEED;
		rightwheel=BIAS_SPEED;
		if (gostraight==0) {
			for (i=0; i<8; i++) {
				leftwheel+=follow_weightleft[i]*(distances[i]>>4);
				rightwheel+=follow_weightright[i]*(distances[i]>>4);
			}
		}

		if (rightwheel < 0)
		{
			turningRight = 1;
		}
		else if (turningRight)
		{
			reportValue("finished right", 1);
			turningRight = 0;
		}	

		if (rightwheel - leftwheel > 210)
		{
			turningLeft = 1;
			if (finishedLeft && abs(e_get_steps_left()) > 100)
			{
				reportValue("starting 2nd turn", 1);
				int lsteps = e_get_steps_left();
				int rsteps = e_get_steps_right();
				reportValue("left steps", lsteps);
				reportValue("right steps", rsteps);
				finishedLeft = 0;
				followsetSpeed(0, 0);
				break;
			}
		}
		else if (turningLeft)
		{
			reportValue("finished left", 1);
			e_set_steps_left(0);
			e_set_steps_right(0);
			turningLeft = 0;
			finishedLeft = 1;		
		}

		// set robot speed
		followsetSpeed(leftwheel, rightwheel);

		wait(15000);
	}	
}
