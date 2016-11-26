#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"
#include "uart/e_uart_char.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "math.h"
#include "string.h"
#include <stdio.h>

#define s 300

#define delayTimer 500000

int getProx(int sensors[], int noOfSensors)
{
	int i;
	int prox;
	for (i = 0;i < noOfSensors; i++)
	{
		prox = prox + e_get_prox(sensors[i]);
	}
	return (int)(prox/noOfSensors);
}

void setLeds(int leds[], int noOfLeds)
{
	int i;
	for (i = 0; i < noOfLeds; i++)
	{
		e_set_led(leds[i], 2);
	}
}

void updateLeft(int speed)
{
	e_set_speed_left(speed);
}

void updateRight(int speed)
{
	e_set_speed_right(speed);
}

void wait(int timer)
{
	int i;
	for (i = 0; i < timer; i++)
	{
		asm("NOP");
	}	
}

void reportValue(char* title, int value)
{
	char uartbuffer[100];
	sprintf(uartbuffer, "(%s - %d)\r\n", title, value);
	int length = strlen(uartbuffer);
	e_send_uart1_char(uartbuffer, length);
	while(e_uart1_sending()){}
}

int testProx(int sensors[], int noOfSensors, char* direction)
{
	
	updateLeft(0);
	updateRight(0);
	int prox;
	while (1)
	{
		prox = getProx(sensors, noOfSensors);
		reportValue(direction, prox);
		wait(delayTimer);
	}
}

void avoidBoundary()
{
	int front[] = {7, 0};
	int frontLeds[] = {0};
	int frontright[] = {0, 1, 2};
	int rightLeds[] = {1, 2, 3};
	int frontleft[] = {0, 7, 6, 5};
	int leftLeds[] = {5, 6, 7};
	int frontProx = getProx(front, 2);
	
	//testProx(frontleft, 4, "frontleft");
	//left led on
	setLeds(leftLeds, 3);
	while (frontProx > 400)
	{
		//turn left until front prox doesn't detect object
		reportValue("turning left", frontProx);
		updateLeft(-s);
		updateRight(s);
		frontProx = getProx(front, 2);
		wait(delayTimer);
	}
	//left led off
	setLeds(leftLeds, 3);
	
	int frontrightProx = getProx(frontright, 3);
	wait(delayTimer); //wait to get correct prox value
	int frontleftProx = getProx(frontleft, 4);
	wait(delayTimer); //wait to get correct prox value
	while (frontrightProx > 350)
	{
		//go straight, if frontright senses object, turn left a bit
		switch(frontrightProx)
		{
			case 350 ... 650: 
			{//go straight, increasing frontright
				setLeds(frontLeds, 1);
				while ((frontrightProx < 650) && (frontrightProx > 350))
				{
					if(frontleftProx > 400)
					{
						reportValue("entering recursion loop", frontleftProx);
						avoidBoundary();
					}
					else
					{	
						reportValue("going straight, object on right", frontrightProx);
						updateLeft(s);
						updateRight(s);
					}
					frontrightProx = getProx(frontright, 3);
					wait(delayTimer); //wait to get correct prox value
					frontleftProx = getProx(frontleft, 4);
					wait(delayTimer); //wait to get correct prox value
				}
				setLeds(frontLeds, 1);
				break;
			}
			case 651 ... 2000:
			{//object found, turn left, decreasing frontright
				setLeds(leftLeds, 3);
				while (frontrightProx > 650)
				{
					reportValue("turning left a bit", frontrightProx);
					updateLeft(-s);
					updateRight(s);
					frontrightProx = getProx(frontright, 3);
				}
				setLeds(leftLeds, 3);
				break;
			}
			default: 
			{
				reportValue("outside range", frontrightProx);
				break;
			}
		}
		frontrightProx = getProx(frontright, 3);
		wait(delayTimer);
	}

	setLeds(rightLeds, 3);
	//this need to be initialised as 0 as first value is incorrect.
	frontleftProx = 0;
	while (frontleftProx < 150)
	{
		//turn right until frontleft senses object
		reportValue("turning right", frontleftProx);
		updateLeft(s);
		updateRight(-s);
		frontleftProx = getProx(frontleft, 4);
		wait(delayTimer);
		//TO DO: bug where this loop breaks prematurely?
	}
	//left leds off
	setLeds(rightLeds, 3);
}

int getSelector() {
	return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

int main()
{
	e_init_port();
	e_init_prox();
	e_init_motors();
	e_init_uart1();
	
	int selector = getSelector();
	if (selector > 0)
	{
		updateLeft(s);
		updateRight(s);
		int frontwide[] = {6, 7, 0, 1};
		int frontProx = getProx(frontwide, 4);
		while(1)
		{	
			reportValue("front", frontProx);
			if (frontProx > 400)
			{
				reportValue("Entering Main", frontProx);
				avoidBoundary();
			}
			updateLeft(s);
			updateRight(s);
			frontProx = getProx(frontwide, 4);
			wait(delayTimer);
		}
	}
	else
	{
		e_set_led(8,1);
		while(1);
	}
}
