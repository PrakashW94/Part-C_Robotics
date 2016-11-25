#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"
#include "uart/e_uart_char.h"

#include "a_d/e_ad_conv.h"
#include "a_d/e_prox.h"

#include "math.h"

#define s 300
#define spaceLimit 400

int getProx(int sensors[], int noOfSensors)
{
	int i;
	int prox;
	for (i = 0;i < noOfSensors; i++)
	{
		prox = prox + e_get_prox(sensors[i]);
	}
	return floor(prox/noOfSensors);
}

int getProxWeighted(int sensors[], int weightedSensor, int noOfSensors)
{
	int i;
	int prox, temp;
	for (i = 0;i < noOfSensors; i++)
	{
		if (sensors[i] == weightedSensor)
		{
			prox = prox + 2*e_get_prox(sensors[i]);
		}
		else
		{
			prox = prox + e_get_prox(sensors[i]);
		}
	}
	return floor(prox/(noOfSensors+1));
}

void updateLeft(int speed)
{
	e_set_speed_left(speed);
}

void updateRight(int speed)
{
	e_set_speed_right(speed);
}

void wait(int ms)
{
	int i;
	for (i = 0; i < ms; i++)
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

void avoidBoundary()
{
	int front[] = {7, 0};
	int frontright[] = {0, 1, 2};
	int frontleft[] = {0, 7, 6, 5};
	int frontProx = getProx(front, 2);
	while (frontProx > spaceLimit)
	{
		//turn left until front prox doesn't detect object
		reportValue("turning left", frontProx);
		updateLeft(-s);
		updateRight(s);
		frontProx = getProx(front, 2);
		wait(500000);
	}
	
	int frontrightProx = getProx(frontright, 3);
	while (frontrightProx > 200)
	{
		//go straight, if frontright senses object, turn left a bit
		switch(frontrightProx)
		{
			case 200 ... 700: 
			{//go straight, increasing frontright
				reportValue("going straight, object on right", frontrightProx);
				updateLeft(s);
				updateRight(s);
				break;
			}
			case 701 ... 2000:
			{//object found, turn left, decreasing frontright
				reportValue("turning left a bit", frontrightProx);
				updateLeft(-s);
				updateRight(s);
				break;
			}
			default: 
			{
				break;
			}
		}
		frontrightProx = getProx(frontright, 3);
		wait(500000);
	}

	int frontleftProx = 0;
	while (frontleftProx < 50)
	{
		//turn right until frontleft senses object
		reportValue("turning right", frontleftProx);
		updateLeft(s);
		updateRight(-s);
		frontleftProx = getProx(frontleft, 4);
		wait(500000);
	}
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
		int front[] = {6, 7, 0, 1};
		int frontProx = getProx(front, 4);
		while(1)
		{	
			reportValue("front", frontProx);
			if (frontProx > spaceLimit)
			{
				reportValue("Entering Main", frontProx);
				LED0 = 1;
				avoidBoundary();
			}
			updateLeft(s);
			updateRight(s);
			frontProx = getProx(front, 4);
			wait(500000);
		}
	}
	else
	{
		e_set_led(8,1);
		while(1);
	}
	
}